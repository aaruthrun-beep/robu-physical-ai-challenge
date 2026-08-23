"""Robot model with URDF loading, DH kinematics, and joint configuration."""

import logging
import numpy as np
import os

from .urdf_parser import URDFModel, JointDef
from .kinematics import DHArm, DHParameter, dh_transform, create_astra_dh

log = logging.getLogger("astra_studio.robot")


class RobotModel:
    """Full robot model integrating URDF data, DH kinematics, and joint configuration.

    Supports:
      - Loading from URDF files (any standard URDF)
      - Manual DH parameter configuration
      - Forward kinematics via DH parameters
      - Joint limits, gear ratios, home positions
      - End-effector pose computation
    """

    def __init__(self, name="astra"):
        self.name = name
        self.urdf_path = None
        self.urdf_model = None          # URDFModel instance
        self.dh_arm = None              # DHArm instance
        self.joints = []                # list of JointDef
        self.links = []                 # list of LinkDef
        self.num_joints = 6
        self.home_position = [0.0] * 6
        self.joint_limits = {
            "j1": (-180, 180),
            "j2": (-90, 90),
            "j3": (-135, 135),
            "j4": (-180, 180),
            "j5": (-120, 120),
            "j6": (-180, 180),
        }
        self.gear_ratios = [23.8, 45.0, 30.0, 24.75, 25.0, 25.0]
        self._pybullet_id = None

        # Initialize with defaults
        self.reset_astra_defaults()

    def reset_astra_defaults(self):
        """Reset to Astra 6-DOF defaults."""
        self.dh_arm = create_astra_dh()
        self.joint_limits = {}
        for i, name in enumerate(self.dh_arm.joint_names):
            if name in self.dh_arm.joint_limits:
                lo, hi = self.dh_arm.joint_limits[name]
                self.joint_limits[name] = (np.degrees(lo), np.degrees(hi))
        # Gear ratios come from the DH model (the hardware source of truth).
        self.gear_ratios = [self.dh_arm.gear_ratios.get(name, 1.0)
                            for name in self.dh_arm.joint_names]
        self.num_joints = len(self.dh_arm.joint_names)
        self.home_position = [0.0] * self.num_joints

    def load_urdf(self, path, guess_dh=True):
        """Load a URDF file and optionally estimate DH parameters.

        Args:
            path: path to URDF file
            guess_dh: if True, estimate DH parameters from joint origins/axes

        Returns:
            True on success, False if the file couldn't be loaded.
        """
        try:
            self.urdf_path = path
            self.urdf_model = URDFModel.load(path)
            self.name = self.urdf_model.name
        except Exception as e:
            log.error("Couldn't load the robot model from %s: %s", path, e)
            self.urdf_path = None
            self.urdf_model = None
            return False

        if guess_dh:
            self.dh_arm = DHArm.from_urdf(path)
            self.joints = self.urdf_model.joints
            self.links = self.urdf_model.links
            self.num_joints = self.dh_arm.num_joints

            # Update joint limits from URDF
            self.joint_limits = {}
            for name, jdef in self.urdf_model.joints.items():
                if jdef.is_movable:
                    lo, hi = np.degrees(jdef.lower_limit), np.degrees(jdef.upper_limit)
                    self.joint_limits[name] = (lo, hi)

            self.home_position = [0.0] * self.num_joints
        return True

    def set_dh_parameters(self, dh_params_list, joint_names, joint_limits=None,
                          gear_ratios=None, convention="standard"):
        """Manually set DH parameters for the robot.

        Args:
            dh_params_list: list of (a, alpha, d, theta_offset) tuples
            joint_names: list of joint names
            joint_limits: optional dict of name -> (lower_deg, upper_deg)
            gear_ratios: optional dict of name -> ratio
            convention: "standard" or "modified"
        """
        arm = DHArm(self.name, convention)
        for i, (a, alpha, d, theta_off) in enumerate(dh_params_list):
            name = joint_names[i] if i < len(joint_names) else f"j{i+1}"
            dh = DHParameter(a=a, alpha=alpha, d=d, theta=theta_off,
                             joint_name=name, convention=convention)
            limits = joint_limits.get(name, (-180, 180)) if joint_limits else (-180, 180)
            limits_rad = (np.radians(limits[0]), np.radians(limits[1]))
            gear = gear_ratios.get(name, 1.0) if gear_ratios else 1.0
            arm.add_joint(dh, name=name, limits=limits_rad, gear_ratio=gear, home=0.0)

        self.dh_arm = arm
        self.num_joints = arm.num_joints
        self.joint_limits = {n: (np.degrees(v[0]), np.degrees(v[1]))
                            for n, v in arm.joint_limits.items()}
        self.home_position = [0.0] * self.num_joints

    def fk(self, joint_angles):
        """Forward kinematics: compute end-effector pose from joint angles.

        Args:
            joint_angles: list of joint angles in degrees

        Returns:
            4x4 numpy array (homogeneous transform)
        """
        if self.dh_arm is None:
            return np.eye(4)
        angles = np.radians(joint_angles) if max(abs(a) for a in joint_angles) > 6.28 else np.array(joint_angles)
        return self.dh_arm.forward(angles)

    def ik(self, target_pos, target_orient=None, guess=None):
        """Inverse kinematics — uses Jacobian-transpose method.

        Args:
            target_pos: [x, y, z] target position
            target_orient: optional 3x3 rotation matrix
            guess: initial joint angle guess (degrees)

        Returns:
            List of joint angles (degrees) or None if no solution found.
        """
        if self.dh_arm is None:
            return self.home_position[:]

        guess_rad = np.radians(guess) if guess is not None else None
        result = self.dh_arm.compute_ik(target_pos, target_orient,
                                         joint_angles=guess_rad)
        if result is not None:
            return list(np.degrees(result))
        return None

    def position(self, joint_angles):
        """Get end-effector position only."""
        T = self.fk(joint_angles)
        return T[:3, 3]

    def get_joint_limits_list(self):
        """Get joint limits as a list of (lower, upper) tuples in degrees."""
        limits = []
        if self.dh_arm:
            for name in self.dh_arm.joint_names:
                if name in self.joint_limits:
                    limits.append(self.joint_limits[name])
                else:
                    limits.append((-180.0, 180.0))
        else:
            for i in range(self.num_joints):
                jname = f"j{i+1}"
                limits.append(self.joint_limits.get(jname, (-180.0, 180.0)))
        return limits

    def get_joint_names(self):
        """Get list of joint names in order."""
        if self.dh_arm and self.dh_arm.joint_names:
            return list(self.dh_arm.joint_names)
        return [f"j{i+1}" for i in range(self.num_joints)]

    def get_dh_table(self):
        """Get DH parameter table as a list of dicts."""
        if self.dh_arm is None:
            return []
        return [p.to_dict() for p in self.dh_arm.dh_params]

    def to_dict(self):
        """Serialize robot model configuration."""
        data = {
            "name": self.name,
            "urdf_path": self.urdf_path or "",
            "num_joints": self.num_joints,
            "home_position": list(self.home_position),
            "joint_limits": {k: list(v) for k, v in self.joint_limits.items()},
            "gear_ratios": list(self.gear_ratios),
        }
        if self.dh_arm:
            data["dh"] = self.dh_arm.to_dict()
        return data

    def save_config(self, path):
        """Save robot configuration to JSON file."""
        import json
        try:
            with open(path, "w") as f:
                json.dump(self.to_dict(), f, indent=2)
        except Exception as e:
            log.error("Couldn't save the robot configuration to %s: %s", path, e)
            raise

    def load_config(self, path):
        """Load robot configuration from JSON file."""
        import json
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            log.error("Couldn't load the robot configuration from %s: %s", path, e)
            raise
        self.name = data.get("name", self.name)
        self.urdf_path = data.get("urdf_path", None)
        self.num_joints = data.get("num_joints", 6)
        self.home_position = data.get("home_position", [0.0] * self.num_joints)

        limits = data.get("joint_limits", {})
        self.joint_limits = {k: tuple(v) for k, v in limits.items()}
        self.gear_ratios = data.get("gear_ratios", [1.0] * self.num_joints)

        dh_data = data.get("dh")
        if dh_data:
            self.dh_arm = DHArm.load_from_dict(dh_data)

    @property
    def pybullet_id(self):
        return self._pybullet_id

    @pybullet_id.setter
    def pybullet_id(self, val):
        self._pybullet_id = val
