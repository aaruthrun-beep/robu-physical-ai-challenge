"""Professional 3D viewport using pyqtgraph GLViewWidget with mesh rendering."""

import math
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QFont

from pyqtgraph.opengl import GLViewWidget, GLMeshItem, MeshData, GLGridItem, GLAxisItem

try:
    from pyqtgraph.opengl import GLViewWidget as _GLVW
    import pyqtgraph as _pg
    _PG_VERSION = tuple(int(x) for x in _pg.__version__.split(".")[:2])
except Exception:
    _PG_VERSION = (0, 13)

from .gizmo import GizmoRenderer


# Segment lengths (m) for the built-in robot arm links.
_SEGMENT_LENGTHS = [0.075, 0.065, 0.045, 0.030, 0.025]


def _make_cylinder(radius, height, segments=16):
    theta = np.linspace(0, 2 * np.pi, segments, endpoint=False)
    verts = []
    faces = []
    # Side faces (triangles)
    for i in range(segments):
        j = (i + 1) % segments
        verts.append([np.cos(theta[i]) * radius, np.sin(theta[i]) * radius, -height / 2])
        verts.append([np.cos(theta[j]) * radius, np.sin(theta[j]) * radius, -height / 2])
        verts.append([np.cos(theta[i]) * radius, np.sin(theta[i]) * radius, height / 2])
        verts.append([np.cos(theta[j]) * radius, np.sin(theta[j]) * radius, height / 2])
        idx = i * 4
        faces.append([idx, idx + 1, idx + 2])
        faces.append([idx + 1, idx + 3, idx + 2])

    # Bottom cap (triangulate from center)
    center_bottom = len(verts)
    verts.append([0, 0, -height / 2])
    nv = len(verts)
    for i in range(segments):
        verts.append([np.cos(theta[i]) * radius, np.sin(theta[i]) * radius, -height / 2])
    for i in range(segments):
        j = (i + 1) % segments
        faces.append([center_bottom, nv + i, nv + j])

    # Top cap (triangulate from center)
    center_top = len(verts)
    verts.append([0, 0, height / 2])
    nv2 = len(verts)
    for i in range(segments):
        verts.append([np.cos(theta[i]) * radius, np.sin(theta[i]) * radius, height / 2])
    for i in range(segments):
        j = (i + 1) % segments
        faces.append([center_top, nv2 + j, nv2 + i])

    verts = np.array(verts, dtype=np.float32)
    faces = np.array(faces, dtype=np.uint32)
    return MeshData(vertexes=verts, faces=faces)


def _make_sphere(radius, rings=12, slices=12):
    verts = []
    faces = []
    for i in range(rings + 1):
        phi = np.pi * i / rings
        for j in range(slices):
            theta = 2 * np.pi * j / slices
            x = np.sin(phi) * np.cos(theta) * radius
            y = np.sin(phi) * np.sin(theta) * radius
            z = np.cos(phi) * radius
            verts.append([x, y, z])
    for i in range(rings):
        for j in range(slices):
            a = i * slices + j
            b = i * slices + (j + 1) % slices
            c = (i + 1) * slices + j
            d = (i + 1) * slices + (j + 1) % slices
            if i > 0:
                faces.append([a, b, c])
            if i < rings - 1:
                faces.append([b, d, c])

    verts = np.array(verts, dtype=np.float32)
    faces = np.array(faces, dtype=np.uint32)
    return MeshData(vertexes=verts, faces=faces)


def _compute_normals(verts, faces):
    normals = np.zeros_like(verts)
    for f in faces:
        v0, v1, v2 = verts[f[0]], verts[f[1]], verts[f[2]]
        n = np.cross(v1 - v0, v2 - v0)
        length = np.linalg.norm(n)
        if length > 1e-10:
            n /= length
        for idx in f:
            normals[idx] += n
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return normals / norms


class Viewport3D(QWidget):
    mouse_moved = pyqtSignal(float, float)
    zoom_changed = pyqtSignal(float)
    gizmo_axis_dragged = pyqtSignal(str, float)
    link_selected = pyqtSignal(int)
    mesh_selected = pyqtSignal(int)   # index into _imported_meshes

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 200)
        self.sim = None
        self.connection_manager = None

        self.gizmo = GizmoRenderer()
        self._imported_meshes = []
        self._mesh_color_idx = 0
        self._mesh_items = []
        self._mesh_tints = []
        self._selected_mesh = -1          # index into _imported_meshes
        self._drag_active = False         # dragging a selected mesh
        self._drag_last = None            # last drag position (ndc)
        self._robot_items = []            # robot link/joint GLMeshItems
        self._wireframe = False
        self._auto_rotate = False
        self._rotate_timer = QTimer()
        self._rotate_timer.timeout.connect(self._rotate_tick)

        self._setup_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self._render_frame)
        self.timer.start(33)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._gl = GLViewWidget()
        self._gl.setBackgroundColor(18, 20, 30)
        self._gl.setCameraPosition(distance=2.0, elevation=-30, azimuth=45)
        self._gl.opts['distance'] = 2.0
        self._gl.opts['elevation'] = -30
        self._gl.opts['azimuth'] = 45
        try:
            from PyQt5.QtGui import QVector3D
            self._gl.opts['center'] = QVector3D(0, 0, 0.5)
        except Exception:
            self._gl.opts['center'] = np.array([0, 0, 0.5])

        self._grid = GLGridItem()
        self._grid.setSize(12, 12)
        self._grid.setSpacing(0.05, 0.05)
        self._grid.setColor((40, 45, 60, 80))
        self._gl.addItem(self._grid)

        self._axis = GLAxisItem()
        self._axis.setSize(0.6, 0.6, 0.6)
        self._gl.addItem(self._axis)

        self._robot_items = []
        self._build_robot()
        self._gl.update()

        layout.addWidget(self._gl)

        # ── stl-orbit style view controls (bottom-left overlay) ──────
        from . import palette as P
        from PyQt5.QtWidgets import QPushButton
        self._view_toolbar = QWidget(self)
        tb_lay = QHBoxLayout(self._view_toolbar)
        tb_lay.setContentsMargins(6, 6, 6, 6)
        tb_lay.setSpacing(4)
        btn_specs = [
            ("Wire", self.set_wireframe, True, True),
            ("Grid", self.set_grid_visible, True, True),
            ("Rotate", self.set_auto_rotate, True, True),
            ("Fit", self.fit_view, False, False),
            ("Reset", self.reset_view, False, False),
        ]
        self._view_btns = {}
        for label, cb, checkable, start_checked in btn_specs:
            b = QPushButton(label)
            b.setCheckable(checkable)
            b.setChecked(start_checked)
            b.setFixedHeight(22)
            b.setCursor(__import__('PyQt5.QtCore', fromlist=['Qt']).Qt.PointingHandCursor)
            b.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=10, padding="0px 8px"))
            if checkable:
                b.toggled.connect(cb)
            else:
                b.clicked.connect(cb)
            tb_lay.addWidget(b)
            self._view_btns[label] = b
        self._view_toolbar.adjustSize()

        # Subtle hint overlay (kept out of the GL scene)
        self._hint_label = QLabel("G — gizmo   ·   Esc — cancel", self)
        self._hint_label.setStyleSheet(
            "color: rgba(153, 142, 132, 180); font-size: 10px;"
            "background: transparent; border: none; padding: 2px 6px;"
        )
        self._hint_label.adjustSize()

    def _build_robot(self):
        for item in self._robot_items:
            try:
                self._gl.removeItem(item)
            except Exception:
                pass
        self._robot_items.clear()

        self._base_item = GLMeshItem(
            meshdata=_make_cylinder(0.055, 0.018, 24),
            smooth=True, drawEdges=False,
            color=(0.22, 0.24, 0.32, 1.0),
        )
        self._base_item.translate(0, 0, 0.009)
        self._gl.addItem(self._base_item)
        self._robot_items.append(self._base_item)

        seg_defs = [
            (0.075, 0.014, (0.26, 0.30, 0.43, 1.0)),
            (0.065, 0.012, (0.22, 0.26, 0.40, 1.0)),
            (0.045, 0.010, (0.20, 0.52, 0.75, 1.0)),
            (0.030, 0.008, (0.16, 0.40, 0.65, 1.0)),
            (0.025, 0.006, (0.10, 0.36, 0.58, 1.0)),
        ]

        self._link_items = []
        self._joint_items = []
        z = 0.018
        for length, radius, color in seg_defs:
            link = GLMeshItem(
                meshdata=_make_cylinder(radius, length, 16),
                smooth=True, drawEdges=False,
                color=color,
            )
            link.translate(0, 0, z + length / 2)
            self._gl.addItem(link)
            self._robot_items.append(link)
            self._link_items.append(link)

            jnt = GLMeshItem(
                meshdata=_make_sphere(radius * 1.3, 12, 12),
                smooth=True, drawEdges=False,
                color=(0.28, 0.30, 0.42, 1.0),
            )
            jnt.translate(0, 0, z)
            self._gl.addItem(jnt)
            self._robot_items.append(jnt)
            self._joint_items.append(jnt)
            z += length

        self._tip_item = GLMeshItem(
            meshdata=_make_sphere(0.005, 8, 8),
            smooth=True, drawEdges=False,
            color=(0.28, 0.56, 0.88, 1.0),
        )
        self._tip_item.translate(0, 0, z + 0.02)
        self._gl.addItem(self._tip_item)
        self._robot_items.append(self._tip_item)

    def set_simulation(self, sim_engine):
        self.sim = sim_engine

    def set_connection_manager(self, cm):
        self.connection_manager = cm

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "_hint_label"):
            self._hint_label.move(8, self.height() - self._hint_label.height() - 6)
        # stl-orbit style view toolbar: bottom-left, above the hint.
        if hasattr(self, "_view_toolbar"):
            tb_h = self._view_toolbar.height()
            self._view_toolbar.move(8, self.height() - tb_h - 26)
        # The program overlay is a free-floating widget now — the user
        # positions/resizes it by dragging; don't re-pin it on viewport
        # resize. (Previously it was forced to (8,8) at 280px x 60%.)

    def set_theme(self, theme_name):
        from .themes import ThemeManager
        t = ThemeManager.get_theme(theme_name)
        bg = t.get("viewport_bg", "#12141E")
        qc = QColor(bg)
        self._gl.setBackgroundColor(qc.red(), qc.green(), qc.blue())
        # Tint the grid to match the theme
        colors = ThemeManager.get_viewport_colors(theme_name)
        grid = colors.get("grid", [0.2, 0.18, 0.15])
        self._grid.setColor((
            int(grid[0] * 255), int(grid[1] * 255), int(grid[2] * 255), 80
        ))

    def add_mesh(self, mesh):
        colors = [
            (0.47, 0.55, 0.71, 0.85),
            (0.71, 0.47, 0.47, 0.85),
            (0.47, 0.71, 0.47, 0.85),
            (0.71, 0.71, 0.47, 0.85),
            (0.71, 0.47, 0.71, 0.85),
            (0.47, 0.71, 0.71, 0.85),
            (0.71, 0.63, 0.39, 0.85),
        ]
        c = colors[self._mesh_color_idx % len(colors)]
        self._mesh_color_idx += 1
        # Keep the tint so we can highlight on selection
        self._mesh_tints.append(c)

        mesh._scale_to_fit()

        verts = mesh.vertices.astype(np.float32)
        faces = mesh.faces.astype(np.uint32)

        # Per-vertex colors (Arctos-style) so the mesh renders with its tint.
        vcolors = mesh.compute_vertex_colors(
            [int(c[0]*255), int(c[1]*255), int(c[2]*255), int(c[3]*255)])

        # Solid render — no wireframe edges (matches Arctos's clean look).
        # pyqtgraph computes vertex normals internally for smooth shading.
        md = MeshData(vertexes=verts, faces=faces, vertexColors=vcolors)
        item = GLMeshItem(
            meshdata=md, smooth=True, drawEdges=False,
            color=c,
            shader='normalColor' if False else None,
        )
        self._gl.addItem(item)
        self._mesh_items.append(item)
        self._imported_meshes.append(mesh)

    def _highlight_mesh(self, index):
        """Brighten the selected mesh, dim the others."""
        for i, item in enumerate(self._mesh_items):
            if i == index:
                c = self._mesh_tints[i]
                item.setColor(tuple(min(x * 1.4, 1.0) for x in c))
            else:
                item.setColor(self._mesh_tints[i])

    def pick_mesh_at(self, x, y):
        """Pick an imported mesh at screen (x, y) via ray-triangle test.

        Returns the index into ``_imported_meshes`` or -1 if nothing hit.
        Mirrors Arctos's ``intersect_ray_mesh`` (tinygizmo/geometry.py).
        """
        if not self._imported_meshes:
            return -1
        # Build a ray from the GL camera through the pixel.
        try:
            center = np.array(self._gl.opts.get('center', [0, 0, 0.5]), dtype=np.float64).reshape(3)
            cam = np.array(self._gl.cameraPosition(), dtype=np.float64).reshape(3)
            dist = self._gl.opts.get('distance', 2.0)
        except Exception:
            return -1

        # Pixel -> NDC
        w, h = max(self._gl.width(), 1), max(self._gl.height(), 1)
        ndc_x = 2.0 * x / w - 1.0
        ndc_y = 1.0 - 2.0 * y / h

        # Reconstruct the ray direction from camera orientation
        import math
        az = math.radians(self._gl.opts.get('azimuth', 45))
        el = math.radians(self._gl.opts.get('elevation', -30))
        # Camera is at distance along the view direction from center.
        view = np.array([
            math.cos(el) * math.cos(az),
            math.cos(el) * math.sin(az),
            math.sin(el),
        ])
        cam = center - view * dist
        right = np.array([-math.sin(az), math.cos(az), 0.0])
        up = np.cross(right, view)
        # Approximate screen-space spread at distance d
        fov = 60.0
        spread = 2.0 * dist * math.tan(math.radians(fov) / 2.0)
        origin = cam + right * (ndc_x * spread / 2.0) + up * (ndc_y * spread / 2.0)
        direction = (center - origin)
        direction /= np.linalg.norm(direction) or 1.0

        best = -1
        best_t = float('inf')
        for mi, mesh in enumerate(self._imported_meshes):
            verts = mesh.get_transformed_vertices()
            faces = mesh.faces
            v0 = verts[faces[:, 0]]
            v1 = verts[faces[:, 1]]
            v2 = verts[faces[:, 2]]
            e1 = v1 - v0
            e2 = v2 - v0
            hvec = np.cross(direction, e2)
            a = np.einsum('ij,ij->i', e1, hvec)
            # Skip near-parallel
            mask = np.abs(a) > 1e-8
            if not mask.any():
                continue
            inv = 1.0 / a[mask]
            s = origin - v0[mask]
            u = np.einsum('ij,ij->i', s, hvec[mask]) * inv
            ok = (u >= 0) & (u <= 1)
            if not ok.any():
                continue
            q = np.cross(s[ok], e1[mask][ok])
            v = np.einsum('ij,ij->i', direction, q) * inv[ok]
            ok2 = ok[ok] & (v >= 0) & (u[ok] + v <= 1)
            if not ok2.any():
                continue
            t = np.einsum('ij,ij->i', e2[mask][ok], q[ok2]) * inv[ok][ok2]
            tmin = t.min()
            if tmin < best_t:
                best_t = tmin
                best = mi

        if best >= 0:
            self._highlight_mesh(best)
            self.mesh_selected.emit(best)
        return best

    def remove_mesh(self, index):
        if 0 <= index < len(self._mesh_items):
            item = self._mesh_items.pop(index)
            self._gl.removeItem(item)
            self._imported_meshes.pop(index)
            if index < len(self._mesh_tints):
                self._mesh_tints.pop(index)

    def clear_meshes(self):
        for item in self._mesh_items:
            try:
                self._gl.removeItem(item)
            except Exception:
                pass
        self._mesh_items.clear()
        self._imported_meshes.clear()
        self._mesh_tints.clear()
        self._mesh_color_idx = 0

    def get_meshes(self):
        return list(self._imported_meshes)

    def start(self):
        if not self.timer.isActive():
            self.timer.start(33)

    def stop(self):
        self.timer.stop()

    def _render_frame(self):
        if self.sim and self.sim.running:
            self.sim.step()
        self._update_robot()
        self._gl.update()

    def _update_robot(self):
        joints = [0.0] * 6
        robot_name = None
        if self.sim and self.sim.robots:
            for name in self.sim.robots:
                robot_name = name
                break
        if self.sim and robot_name:
            try:
                raw = self.sim.get_joint_positions(robot_name)
                rev = self.sim.get_revolute_joints(robot_name)
                joints = [raw[j["index"]] for j in rev[:6]]
            except Exception:
                pass

        j = [math.radians(a) for a in joints]
        cum = [j[0], j[0] + j[1], j[0] + j[1] + j[2],
               j[0] + j[1] + j[2] + j[3], j[0] + j[1] + j[2] + j[3] + j[4],
               j[0] + j[1] + j[2] + j[3] + j[4] + j[5]]

        for i, item in enumerate(self._link_items):
            if i < len(cum):
                item.resetTransform()
                z = 0.018
                for k in range(i):
                    z += _SEGMENT_LENGTHS[k]
                item.translate(0, 0, z + _SEGMENT_LENGTHS[i] / 2)
                item.rotate(math.degrees(cum[i]), 0, 0, 1, local=True)

        for i, item in enumerate(self._joint_items):
            if i < len(cum):
                item.resetTransform()
                z = 0.018
                for k in range(i):
                    z += _SEGMENT_LENGTHS[k]
                item.translate(0, 0, z)
                if i > 0:
                    item.rotate(math.degrees(cum[i - 1]), 0, 0, 1, local=True)

        self._tip_item.resetTransform()
        z = 0.018 + sum(_SEGMENT_LENGTHS)
        self._tip_item.translate(0, 0, z + 0.02)

    def set_camera(self, distance=None, yaw=None, pitch=None, target=None):
        if distance is not None:
            self._gl.opts['distance'] = distance
        if yaw is not None:
            self._gl.opts['azimuth'] = yaw
        if pitch is not None:
            self._gl.opts['elevation'] = pitch
        if target is not None:
            try:
                from PyQt5.QtGui import QVector3D
                self._gl.opts['center'] = QVector3D(*target)
            except Exception:
                self._gl.opts['center'] = np.array(target)
        self._gl.update()

    def reset_view(self):
        self._gl.setCameraPosition(distance=2.0, elevation=-30, azimuth=45)
        try:
            from PyQt5.QtGui import QVector3D
            self._gl.opts['center'] = QVector3D(0, 0, 0.5)
        except Exception:
            self._gl.opts['center'] = np.array([0, 0, 0.5])
        self.gizmo.hide()
        self._gl.update()

    # ── stl-orbit style viewer controls ─────────────────────────────

    def set_wireframe(self, enabled):
        """Toggle wireframe edges on every mesh (robot + imported)."""
        for item in self._mesh_items + self._robot_items:
            try:
                item.opts['drawEdges'] = bool(enabled)
                item.update()
            except Exception:
                pass
        self._wireframe = bool(enabled)
        self._gl.update()

    def set_grid_visible(self, visible):
        self._grid.setVisible(visible)
        self._gl.update()

    def set_auto_rotate(self, enabled):
        self._auto_rotate = enabled
        if enabled and not self._rotate_timer.isActive():
            self._rotate_timer.start(50)

    def _rotate_tick(self):
        if not self._auto_rotate:
            self._rotate_timer.stop()
            return
        az = float(self._gl.opts.get('azimuth', 45))
        self._gl.opts['azimuth'] = (az + 0.3) % 360.0
        self._gl.update()

    def fit_view(self):
        """Frame the bounding box of every visible mesh (stl-orbit 'fit')."""
        verts_all = []
        for mesh in self._imported_meshes:
            try:
                verts_all.append(mesh.vertices.astype(np.float64))
            except Exception:
                pass
        # Include the robot links if present.
        for it in getattr(self, '_robot_items', []):
            md = it.opts.get('meshdata')
            if md is not None:
                try:
                    verts_all.append(np.asarray(md.vertexes(), dtype=np.float64).reshape(-1, 3))
                except Exception:
                    pass
        if not verts_all:
            self.reset_view()
            return
        v = np.vstack(verts_all)
        lo = v.min(axis=0)
        hi = v.max(axis=0)
        center = (lo + hi) / 2.0
        radius = float(np.linalg.norm(hi - lo) / 2.0) or 1.0
        try:
            from PyQt5.QtGui import QVector3D
            self._gl.opts['center'] = QVector3D(*center)
        except Exception:
            self._gl.opts['center'] = np.array(center)
        self._gl.opts['distance'] = radius * 3.5
        self._gl.update()

    def go_home(self):
        robot_name = None
        if self.sim and self.sim.robots:
            for name in self.sim.robots:
                robot_name = name
                break
        if self.sim and robot_name:
            n = len(self.sim.get_joint_positions(robot_name))
            self.sim.set_joint_positions(robot_name, [0.0] * n)

    def stop_sim(self):
        if self.sim:
            self.sim.running = False

    def start_sim(self):
        if self.sim:
            self.sim.running = True

    def select_end_effector(self):
        robot_name = None
        if self.sim and self.sim.robots:
            for name in self.sim.robots:
                robot_name = name
                break
        if self.sim and robot_name:
            pose = self.sim.get_endeffector_pose(robot_name)
            if pose:
                self.gizmo.show_at(pose["position"])
                self._selected_link = -2

    def mousePressEvent(self, event):
        # Forward to GL view for orbit/pan/zoom, then pick on left-click.
        super().mousePressEvent(event)
        if event.button() == Qt.LeftButton and self._imported_meshes:
            # Convert widget coords to GL view coords (offset by layout margin)
            gl_pos = self._gl.mapFrom(self, event.pos())
            idx = self.pick_mesh_at(gl_pos.x(), gl_pos.y())
            self._select_mesh(idx)
            # Begin drag-translate if we grabbed a mesh and Ctrl is held.
            if idx >= 0 and (event.modifiers() & Qt.ControlModifier):
                self._drag_active = True
                self._drag_last = self._to_ndc(gl_pos.x(), gl_pos.y())
                self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag_active and self._selected_mesh >= 0:
            gl_pos = self._gl.mapFrom(self, event.pos())
            ndc = self._to_ndc(gl_pos.x(), gl_pos.y())
            if self._drag_last is not None:
                dx = ndc[0] - self._drag_last[0]
                dy = ndc[1] - self._drag_last[1]
                self._translate_selected_mesh(dx, -dy)
            self._drag_last = ndc
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._drag_active:
            self._drag_active = False
            self._drag_last = None
            self.setCursor(Qt.ArrowCursor)
            return
        super().mouseReleaseEvent(event)

    def _select_mesh(self, index):
        """Select a mesh (index into _imported_meshes, -1 = deselect)."""
        self._selected_mesh = index
        self._highlight_mesh(index)
        self.gizmo.show_at(self._mesh_center(index) if index >= 0 else [0, 0, 0.5])

    def gizmo_target(self):
        """Return the selected imported mesh, or None."""
        if 0 <= self._selected_mesh < len(self._imported_meshes):
            return self._imported_meshes[self._selected_mesh]
        return None

    def _mesh_center(self, index):
        if 0 <= index < len(self._imported_meshes):
            return self._imported_meshes[index].get_transformed_vertices().mean(axis=0)
        return [0, 0, 0]

    def _to_ndc(self, x, y):
        w = max(self._gl.width(), 1)
        h = max(self._gl.height(), 1)
        return (2.0 * x / w - 1.0, 2.0 * y / h - 1.0)

    def _translate_selected_mesh(self, dx, dy):
        """Drag-translate the selected imported mesh in camera space."""
        if not (0 <= self._selected_mesh < len(self._imported_meshes)):
            return
        mesh = self._imported_meshes[self._selected_mesh]
        # Scale the screen delta by the view distance for consistent feel.
        dist = float(self._gl.opts.get("distance", 2.0))
        step = dist * 0.35
        tx, ty = dx * step, dy * step
        # Apply in world space (screen X/Y roughly map to world X/Y here).
        mat = np.eye(4)
        mat[0, 3] = tx
        mat[1, 3] = ty
        mesh.apply_transform(mat)
        # Update the GL item geometry.
        item = self._mesh_items[self._selected_mesh]
        verts = mesh.get_transformed_vertices().astype(np.float32)
        item.setMeshData(
            vertexes=verts,
            faces=mesh.faces.astype(np.uint32),
            vertexColors=mesh.compute_vertex_colors(
                [int(c * 255) for c in self._mesh_tints[self._selected_mesh]]),
        )
        self.gizmo.show_at(self._mesh_center(self._selected_mesh))

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_G:
            self.gizmo.toggle_mode()
        elif event.key() == Qt.Key_Escape:
            self.gizmo.hide()
            self.link_selected.emit(-1)
        else:
            super().keyPressEvent(event)
