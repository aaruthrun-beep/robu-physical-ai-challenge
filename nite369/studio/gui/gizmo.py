"""3D manipulation gizmo for end-effector positioning.

Renders colored arrows/arcs for translation/rotation gizmo
and handles hit-testing for interactive dragging.
"""

import math
import numpy as np
from enum import Enum


class GizmoMode(Enum):
    TRANSLATE = "translate"
    ROTATE = "rotate"


class GizmoAxis(Enum):
    NONE = "none"
    X = "x"
    Y = "y"
    Z = "z"


AXIS_COLORS = {
    GizmoAxis.X: (1.0, 0.2, 0.2),
    GizmoAxis.Y: (0.2, 1.0, 0.2),
    GizmoAxis.Z: (0.2, 0.2, 1.0),
}

AXIS_COLORS_HOVER = {
    GizmoAxis.X: (1.0, 0.5, 0.5),
    GizmoAxis.Y: (0.5, 1.0, 0.5),
    GizmoAxis.Z: (0.5, 0.5, 1.0),
}


class GizmoRenderer:
    """3D manipulation gizmo for end-effector positioning."""

    def __init__(self):
        self.visible = False
        self.position = [0, 0, 0]
        self.mode = GizmoMode.TRANSLATE
        self._hover_axis = GizmoAxis.NONE
        self._active_axis = GizmoAxis.NONE
        self._drag_start = None
        self._drag_value_start = 0.0
        self._axis_length = 0.15
        self._arrow_head_length = 0.03
        self._circle_radius = 0.12

    def show_at(self, position):
        self.position = list(position)
        self.visible = True
        self._active_axis = GizmoAxis.NONE

    def hide(self):
        self.visible = False
        self._active_axis = GizmoAxis.NONE
        self._hover_axis = GizmoAxis.NONE

    def toggle_mode(self):
        self.mode = (
            GizmoMode.ROTATE if self.mode == GizmoMode.TRANSLATE else GizmoMode.TRANSLATE
        )

    def set_hover_axis(self, axis: GizmoAxis):
        self._hover_axis = axis

    def hit_test(self, screen_x, screen_y, view_matrix, proj_matrix, viewport_size, client_id):
        """Test if screen position hits a gizmo axis. Returns axis or None."""
        if not self.visible:
            return GizmoAxis.NONE

        try:
            import pybullet as p
        except ImportError:
            return GizmoAxis.NONE

        world_pos = np.array(self.position)
        best_axis = GizmoAxis.NONE
        best_dist = 20.0

        for axis in [GizmoAxis.X, GizmoAxis.Y, GizmoAxis.Z]:
            axis_point = self._get_axis_tip(axis)
            screen_point = self._world_to_screen(
                axis_point, view_matrix, proj_matrix, viewport_size, client_id
            )
            if screen_point is None:
                continue
            sx, sy = screen_point
            dist = math.sqrt((sx - screen_x) ** 2 + (sy - screen_y) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_axis = axis

        origin_screen = self._world_to_screen(
            world_pos, view_matrix, proj_matrix, viewport_size, client_id
        )
        if origin_screen is not None:
            ox, oy = origin_screen
            dist = math.sqrt((ox - screen_x) ** 2 + (oy - screen_y) ** 2)
            if dist < 15.0:
                return GizmoAxis.NONE

        return best_axis if best_dist < 20.0 else GizmoAxis.NONE

    def _get_axis_tip(self, axis: GizmoAxis) -> list:
        pos = np.array(self.position)
        length = self._axis_length
        if axis == GizmoAxis.X:
            return pos + np.array([length, 0, 0])
        elif axis == GizmoAxis.Y:
            return pos + np.array([0, length, 0])
        elif axis == GizmoAxis.Z:
            return pos + np.array([0, 0, length])
        return pos

    def _world_to_screen(self, world_pos, view_matrix, proj_matrix, viewport_size, client_id):
        try:
            import pybullet as p
            screen = p.getDebugVisualizerCamera(client_id)
            w, h = viewport_size
            cam = p.getDebugVisualizerCamera(client_id)
            view_m = list(cam[1])
            proj_m = list(cam[2])
            viewproj = np.dot(np.array(proj_m).reshape(4, 4), np.array(view_m).reshape(4, 4))
            point = np.array([world_pos[0], world_pos[1], world_pos[2], 1.0])
            clip = viewproj @ point
            if abs(clip[3]) < 1e-6:
                return None
            ndc = clip[:3] / clip[3]
            screen_x = (ndc[0] * 0.5 + 0.5) * w
            screen_y = (-ndc[1] * 0.5 + 0.5) * h
            return (screen_x, screen_y)
        except Exception:
            return None

    def render(self, client_id):
        """Render gizmo axes using PyBullet debug lines."""
        if not self.visible:
            return

        try:
            import pybullet as p
        except ImportError:
            return

        pos = self.position
        length = self._axis_length
        ahl = self._arrow_head_length

        for axis, color_key in [
            (GizmoAxis.X, (1.0, 0.2, 0.2)),
            (GizmoAxis.Y, (0.2, 1.0, 0.2)),
            (GizmoAxis.Z, (0.2, 0.2, 1.0)),
        ]:
            is_hovered = self._hover_axis == axis
            is_active = self._active_axis == axis
            color = AXIS_COLORS_HOVER.get(axis, color_key) if is_hovered else color_key
            if is_active:
                color = tuple(min(1.0, c + 0.3) for c in color)

            if axis == GizmoAxis.X:
                tip = [pos[0] + length, pos[1], pos[2]]
                p.addUserDebugLine(pos, tip, color, lineWidth=3, lifeTime=0)
                p.addUserDebugLine(
                    tip,
                    [pos[0] + length - ahl, pos[1] + ahl * 0.4, pos[2]],
                    color, lineWidth=2, lifeTime=0,
                )
                p.addUserDebugLine(
                    tip,
                    [pos[0] + length - ahl, pos[1] - ahl * 0.4, pos[2]],
                    color, lineWidth=2, lifeTime=0,
                )
            elif axis == GizmoAxis.Y:
                tip = [pos[0], pos[1] + length, pos[2]]
                p.addUserDebugLine(pos, tip, color, lineWidth=3, lifeTime=0)
                p.addUserDebugLine(
                    tip,
                    [pos[0] + ahl * 0.4, pos[1] + length - ahl, pos[2]],
                    color, lineWidth=2, lifeTime=0,
                )
                p.addUserDebugLine(
                    tip,
                    [pos[0] - ahl * 0.4, pos[1] + length - ahl, pos[2]],
                    color, lineWidth=2, lifeTime=0,
                )
            elif axis == GizmoAxis.Z:
                tip = [pos[0], pos[1], pos[2] + length]
                p.addUserDebugLine(pos, tip, color, lineWidth=3, lifeTime=0)
                p.addUserDebugLine(
                    tip,
                    [pos[0] + ahl * 0.4, pos[1], pos[2] + length - ahl],
                    color, lineWidth=2, lifeTime=0,
                )
                p.addUserDebugLine(
                    tip,
                    [pos[0] - ahl * 0.4, pos[1], pos[2] + length - ahl],
                    color, lineWidth=2, lifeTime=0,
                )

        if self.mode == GizmoMode.ROTATE:
            self._render_rotation_circles(client_id)

    def _render_rotation_circles(self, client_id):
        try:
            import pybullet as p
        except ImportError:
            return

        pos = self.position
        radius = self._circle_radius
        segments = 24

        for axis, color in [
            (GizmoAxis.X, (1.0, 0.2, 0.2)),
            (GizmoAxis.Y, (0.2, 1.0, 0.2)),
            (GizmoAxis.Z, (0.2, 0.2, 1.0)),
        ]:
            for i in range(segments):
                angle1 = (i / segments) * 2 * math.pi
                angle2 = ((i + 1) / segments) * 2 * math.pi

                if axis == GizmoAxis.X:
                    p1 = [pos[0], pos[1] + radius * math.cos(angle1), pos[2] + radius * math.sin(angle1)]
                    p2 = [pos[0], pos[1] + radius * math.cos(angle2), pos[2] + radius * math.sin(angle2)]
                elif axis == GizmoAxis.Y:
                    p1 = [pos[0] + radius * math.cos(angle1), pos[1], pos[2] + radius * math.sin(angle1)]
                    p2 = [pos[0] + radius * math.cos(angle2), pos[1], pos[2] + radius * math.sin(angle2)]
                else:
                    p1 = [pos[0] + radius * math.cos(angle1), pos[1] + radius * math.sin(angle1), pos[2]]
                    p2 = [pos[0] + radius * math.cos(angle2), pos[1] + radius * math.sin(angle2), pos[2]]

                p.addUserDebugLine(p1, p2, color, lineWidth=2, lifeTime=0)
