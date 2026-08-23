"""URDF parser — extracts joint/link/kinematic data from standard URDF files.

Supports:
  - Revolute, prismatic, continuous, fixed joints
  - Joint limits, axis, origin transforms
  - Link visual/collision/inertial properties
  - Kinematic tree (parent-child relationships)
  - Material parsing
"""

import xml.etree.ElementTree as ET
import numpy as np
import os


def _parse_vector(text, default=(0, 0, 0)):
    """Parse a space/comma-separated vector string like '0 0 1'."""
    try:
        parts = text.strip().replace(",", " ").split()
        return tuple(float(p) for p in parts[:3])
    except (ValueError, AttributeError):
        return tuple(default)


def _parse_rpy(text, default=(0, 0, 0)):
    """Parse roll-pitch-yaw string to XYZ Euler angles."""
    return _parse_vector(text, default)


def _parse_transform(origin_elem):
    """Parse <origin> element into position and rotation matrix."""
    if origin_elem is None:
        return np.eye(4)
    xyz = _parse_vector(origin_elem.get("xyz", "0 0 0"))
    rpy = _parse_rpy(origin_elem.get("rpy", "0 0 0"))
    T = np.eye(4)
    T[:3, 3] = xyz
    cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
    cp, sp = np.cos(rpy[1]), np.sin(rpy[1])
    cy, sy = np.cos(rpy[2]), np.sin(rpy[2])
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
    T[:3, :3] = R
    return T


JOINT_TYPE_MAP = {
    "revolute": "revolute",
    "continuous": "continuous",
    "prismatic": "prismatic",
    "fixed": "fixed",
    "floating": "floating",
    "planar": "planar",
}


class JointDef:
    """Definition of a single robot joint parsed from URDF."""

    def __init__(self, name, joint_type):
        self.name = name
        self.type = JOINT_TYPE_MAP.get(joint_type, "fixed")
        self.parent = ""
        self.child = ""
        self.origin = np.eye(4)         # 4x4 transform from parent to joint
        self.axis = (0, 0, 1)           # rotation/translation axis
        self.lower_limit = 0.0
        self.upper_limit = 0.0
        self.effort = 0.0
        self.velocity = 0.0
        self.damping = 0.0
        self.friction = 0.0
        self.mimic = None               # joint name to mimic
        self.mim_multiplier = 1.0       # multiplier for mimic joint
        self.mim_offset = 0.0

    @property
    def is_movable(self):
        return self.type in ("revolute", "continuous", "prismatic")

    @property
    def angle_range(self):
        """Returns (lower, upper) in radians. Continuous joints return (-pi, pi)."""
        if self.type == "continuous":
            return (-np.pi, np.pi)
        return (self.lower_limit, self.upper_limit)

    def to_dict(self):
        return {
            "name": self.name,
            "type": self.type,
            "parent": self.parent,
            "child": self.child,
            "origin_xyz": list(self.origin[:3, 3]),
            "origin_rpy": list(self._matrix_to_rpy(self.origin)),
            "axis": list(self.axis),
            "lower_limit": self.lower_limit,
            "upper_limit": self.upper_limit,
            "effort": self.effort,
            "velocity": self.velocity,
        }

    @staticmethod
    def _matrix_to_rpy(T):
        """Extract roll, pitch, yaw from rotation matrix."""
        r = T[:3, :3]
        sy = np.sqrt(r[0, 0]**2 + r[1, 0]**2)
        singular = sy < 1e-6
        if not singular:
            return (np.arctan2(r[2, 1], r[2, 2]),
                    np.arctan2(-r[2, 0], sy),
                    np.arctan2(r[1, 0], r[0, 0]))
        else:
            return (np.arctan2(-r[1, 2], r[1, 1]),
                    np.arctan2(-r[2, 0], sy),
                    0.0)


class LinkDef:
    """Definition of a single robot link parsed from URDF."""

    def __init__(self, name):
        self.name = name
        self.visual = {}
        self.collision = {}
        self.inertial = {}
        self.mass = 0.0

    def to_dict(self):
        return {
            "name": self.name,
            "mass": self.mass,
            "has_visual": bool(self.visual),
            "has_collision": bool(self.collision),
        }


class URDFModel:
    """Complete robot model parsed from a URDF file.

    Provides structured access to joints, links, and the kinematic tree.
    """

    def __init__(self):
        self.name = ""
        self.path = ""
        self.joints = {}       # name -> JointDef
        self.links = {}        # name -> LinkDef
        self.materials = {}    # name -> RGBA tuple
        self._joint_order = []  # ordered list of joint names (as they appear in URDF)

    @classmethod
    def load(cls, filepath):
        """Parse a URDF file and return a URDFModel instance."""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"URDF file not found: {filepath}")

        tree = ET.parse(filepath)
        root = tree.getroot()

        model = cls()
        model.path = os.path.abspath(filepath)
        model.name = root.get("name", "robot")

        # Parse materials
        for mat_elem in root.findall("material"):
            name = mat_elem.get("name", "")
            color = mat_elem.find("color")
            if color is not None:
                rgba = _parse_vector(color.get("rgba", "0.5 0.5 0.5 1.0"))
            else:
                rgba = (0.5, 0.5, 0.5, 1.0)
            model.materials[name] = rgba

        # Parse links
        for link_elem in root.findall("link"):
            link = LinkDef(link_elem.get("name", ""))
            # Visual
            vis = link_elem.find("visual")
            if vis is not None:
                link.visual["origin"] = _parse_transform(vis.find("origin"))
                geom = vis.find("geometry")
                if geom is not None:
                    for tag in ("box", "cylinder", "sphere", "mesh"):
                        child = geom.find(tag)
                        if child is not None:
                            link.visual["geometry"] = {tag: child.attrib}
                            break
                mat_ref = vis.find("material")
                if mat_ref is not None:
                    link.visual["material"] = mat_ref.get("name", "")
            # Collision
            col = link_elem.find("collision")
            if col is not None:
                link.collision["origin"] = _parse_transform(col.find("origin"))
                geom = col.find("geometry")
                if geom is not None:
                    for tag in ("box", "cylinder", "sphere", "mesh"):
                        child = geom.find(tag)
                        if child is not None:
                            link.collision["geometry"] = {tag: child.attrib}
                            break
            # Inertial
            inert = link_elem.find("inertial")
            if inert is not None:
                mass_elem = inert.find("mass")
                if mass_elem is not None:
                    try:
                        link.mass = float(mass_elem.get("value", 0))
                    except ValueError:
                        link.mass = 0.0
                in_elem = inert.find("inertia")
                if in_elem is not None:
                    link.inertial = {k: float(in_elem.get(k, 0))
                                     for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
                origin = inert.find("origin")
                if origin is not None:
                    link.inertial["origin"] = _parse_vector(origin.get("xyz", "0 0 0"))
            model.links[link.name] = link

        # Parse joints
        for joint_elem in root.findall("joint"):
            name = joint_elem.get("name", "")
            jtype = joint_elem.get("type", "fixed")
            joint = JointDef(name, jtype)
            joint.parent = joint_elem.findtext("parent", "").strip()
            # Handle parent with 'link' attribute
            parent_elem = joint_elem.find("parent")
            if parent_elem is not None:
                joint.parent = parent_elem.get("link", joint.parent)
            child_elem = joint_elem.find("child")
            if child_elem is not None:
                joint.child = child_elem.get("link", "")
            joint.origin = _parse_transform(joint_elem.find("origin"))
            ax = joint_elem.find("axis")
            if ax is not None:
                joint.axis = _parse_vector(ax.get("xyz", "0 0 1"))
            lim = joint_elem.find("limit")
            if lim is not None:
                try:
                    joint.lower_limit = float(lim.get("lower", "0"))
                    joint.upper_limit = float(lim.get("upper", "0"))
                except ValueError:
                    pass
                try:
                    joint.effort = float(lim.get("effort", "0"))
                    joint.velocity = float(lim.get("velocity", "0"))
                except ValueError:
                    pass
            dyn = joint_elem.find("dynamics")
            if dyn is not None:
                try:
                    joint.damping = float(dyn.get("damping", "0"))
                    joint.friction = float(dyn.get("friction", "0"))
                except ValueError:
                    pass
            model.joints[name] = joint
            model._joint_order.append(name)

        return model

    @property
    def num_joints(self):
        return len(self.joints)

    @property
    def movable_joints(self):
        """Return list of joints that are not fixed."""
        return [j for j in self.joints.values() if j.is_movable]

    @property
    def num_movable_joints(self):
        return len(self.movable_joints)

    def get_joint_chain(self, from_link="base_link", to_link=None):
        """Get ordered list of joints from from_link to to_link.

        Walks the kinematic tree from child to parent.
        If to_link is None, returns all joints in tree order.
        """
        # Build child -> parent mapping
        parent_of = {}
        joint_of_child = {}
        for name, joint in self.joints.items():
            parent_of[joint.child] = joint.parent
            joint_of_child[joint.child] = name

        if to_link is None:
            # Return all movable joints in order
            chain = []
            for name in self._joint_order:
                j = self.joints[name]
                if j.is_movable:
                    chain.append(name)
            return chain

        # Walk upward from target
        chain = []
        current = to_link
        while current in parent_of and current != from_link:
            if current in joint_of_child:
                chain.insert(0, joint_of_child[current])
            current = parent_of.get(current)
        return chain

    def get_joint_parent_link(self, joint_name):
        j = self.joints.get(joint_name)
        return j.parent if j else ""

    def get_joint_child_link(self, joint_name):
        j = self.joints.get(joint_name)
        return j.child if j else ""

    def to_dict(self):
        return {
            "name": self.name,
            "path": self.path,
            "num_joints": self.num_joints,
            "num_movable_joints": self.num_movable_joints,
            "joints": {n: j.to_dict() for n, j in self.joints.items()},
            "links": {n: l.to_dict() for n, l in self.links.items()},
            "joint_order": self._joint_order,
        }

    def __repr__(self):
        return (f"URDFModel(name='{self.name}', joints={self.num_joints}, "
                f"movable={self.num_movable_joints})")
