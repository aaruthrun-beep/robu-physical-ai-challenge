"""Mesh loader for STL, STEP, and other 3D formats."""

import os
import numpy as np


class LoadedMesh:
    """A loaded 3D mesh with vertices, faces, and normals."""

    def __init__(self, vertices, faces, normals=None, vertex_normals=None,
                 name="Untitled", color=None):
        self.vertices = np.array(vertices, dtype=np.float64)
        self.faces = np.array(faces, dtype=np.int32)
        if normals is not None:
            self.normals = np.array(normals, dtype=np.float64)
        else:
            self.normals = self._compute_face_normals()
        if vertex_normals is not None:
            self.vertex_normals = np.array(vertex_normals, dtype=np.float64)
        else:
            self.vertex_normals = self._compute_vertex_normals()
        self.name = name
        self.color = color or [120, 140, 180, 255]
        self.transform = np.eye(4)
        self.visible = True

    def _compute_face_normals(self):
        v0 = self.vertices[self.faces[:, 0]]
        v1 = self.vertices[self.faces[:, 1]]
        v2 = self.vertices[self.faces[:, 2]]
        normals = np.cross(v1 - v0, v2 - v0)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        return normals / norms

    def _compute_vertex_normals(self):
        """Smooth per-vertex normals: accumulate face normals into vertices.

        Mirrors Arctos's ``compute_normals`` (tinygizmo/geometry.py) — face
        normals are summed at each vertex then normalized, giving smooth
        shading on curved/rounded geometry.
        """
        vn = np.zeros_like(self.vertices)
        v0 = self.vertices[self.faces[:, 0]]
        v1 = self.vertices[self.faces[:, 1]]
        v2 = self.vertices[self.faces[:, 2]]
        face_n = np.cross(v1 - v0, v2 - v0)
        norms = np.linalg.norm(face_n, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        face_n = face_n / norms
        for i in range(3):
            np.add.at(vn, self.faces[:, i], face_n)
        vn_norms = np.linalg.norm(vn, axis=1, keepdims=True)
        vn_norms[vn_norms == 0] = 1.0
        return vn / vn_norms

    def compute_vertex_colors(self, color=None):
        """Uniform RGBA color per vertex (Arctos per-vertex color model)."""
        c = color or self.color
        arr = np.empty((len(self.vertices), 4), dtype=np.float32)
        arr[:, 0] = c[0] / 255.0
        arr[:, 1] = c[1] / 255.0
        arr[:, 2] = c[2] / 255.0
        arr[:, 3] = (c[3] if len(c) > 3 else 255) / 255.0
        return arr

    @property
    def center(self):
        return self.vertices.mean(axis=0)

    @property
    def bounding_size(self):
        mins = self.vertices.min(axis=0)
        maxs = self.vertices.max(axis=0)
        return float(np.max(maxs - mins))

    def apply_transform(self, mat4):
        self.transform = mat4 @ self.transform

    def _scale_to_fit(self, target_size=60.0):
        """Scale + center the mesh, computing vertex normals ONCE."""
        size = self.bounding_size
        if size <= 0:
            return
        scale = target_size / size
        center = self.center
        # Single pass: scale around origin, then recenter.
        self.vertices = (self.vertices - center) * scale
        self.normals = self._compute_face_normals()
        self.vertex_normals = self._compute_vertex_normals()

    def get_transformed_vertices(self):
        ones = np.ones((len(self.vertices), 1))
        verts = np.hstack([self.vertices, ones])
        transformed = (self.transform @ verts.T).T[:, :3]
        return transformed

    def get_sorted_faces(self, view_dir):
        transformed = self.get_transformed_vertices()
        centers = transformed[self.faces].mean(axis=1)
        depth = np.dot(centers, view_dir)
        order = np.argsort(depth)
        return order, transformed


def load_mesh(filepath, name=None, color=None):
    ext = os.path.splitext(filepath)[1].lower()

    if ext == ".stl":
        return _load_stl(filepath, name, color)
    elif ext in (".step", ".stp"):
        return _load_step(filepath, name, color)
    elif ext == ".obj":
        return _load_obj(filepath, name, color)
    else:
        try:
            return _load_trimesh(filepath, name, color)
        except Exception as e:
            raise ValueError(f"Unsupported file format '{ext}': {e}")


def _load_stl(filepath, name, color):
    import trimesh
    mesh = trimesh.load(filepath, force="mesh")
    if name is None:
        name = os.path.splitext(os.path.basename(filepath))[0]
    return LoadedMesh(
        vertices=mesh.vertices,
        faces=mesh.faces,
        normals=mesh.face_normals if hasattr(mesh, "face_normals") else None,
        vertex_normals=mesh.vertex_normals if hasattr(mesh, "vertex_normals") else None,
        name=name,
        color=color,
    )


def _load_step(filepath, name, color):
    import cadquery as cq
    shape = cq.importers.importStep(filepath)
    if name is None:
        name = os.path.splitext(os.path.basename(filepath))[0]

    vertices = []
    faces = []
    for solid in shape.solids().vals():
        tess = solid.tessellate(0.1, 0.5)
        v_offset = len(vertices)
        vertices.extend([[v.x, v.y, v.z] for v in tess[0]])
        faces.extend([[f[0] + v_offset, f[1] + v_offset, f[2] + v_offset] for f in tess[1]])

    if not vertices:
        raise ValueError("No geometry found in STEP file")

    return LoadedMesh(
        vertices=np.array(vertices),
        faces=np.array(faces, dtype=np.int32),
        name=name,
        color=color,
    )


def _load_obj(filepath, name, color):
    import trimesh
    mesh = trimesh.load(filepath, force="mesh")
    if name is None:
        name = os.path.splitext(os.path.basename(filepath))[0]
    return LoadedMesh(
        vertices=mesh.vertices,
        faces=mesh.faces,
        normals=mesh.face_normals if hasattr(mesh, "face_normals") else None,
        vertex_normals=mesh.vertex_normals if hasattr(mesh, "vertex_normals") else None,
        name=name,
        color=color,
    )


def _load_trimesh(filepath, name, color):
    import trimesh
    mesh = trimesh.load(filepath, force="mesh")
    if name is None:
        name = os.path.splitext(os.path.basename(filepath))[0]
    return LoadedMesh(
        vertices=mesh.vertices,
        faces=mesh.faces,
        normals=mesh.face_normals if hasattr(mesh, "face_normals") else None,
        vertex_normals=mesh.vertex_normals if hasattr(mesh, "vertex_normals") else None,
        name=name,
        color=color,
    )
