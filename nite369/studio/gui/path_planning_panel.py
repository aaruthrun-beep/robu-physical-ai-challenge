"""Path Planning Panel — professional industrial robot trajectory programming interface.

Designed for efficiency: waypoint table at top, preview below, controls on the right.
Inspired by FANUC TP / KUKA KRC / ABB RAPID programming workflow.
"""

import os
import math
import json
import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QFileDialog, QMessageBox,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QDoubleSpinBox, QSpinBox, QComboBox, QCheckBox,
    QFrame, QSplitter, QListWidget,
    QListWidgetItem, QMenu, QInputDialog, QAbstractItemView,
    QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QRectF, QPointF
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPainterPath, QLinearGradient

from ..core.path_planning import (
    Waypoint, Trajectory, TrajectoryPoint, TrajectoryType,
    CollisionSphere, CollisionScene,
    generate_joint_cubic, generate_joint_quintic,
    generate_joint_trapezoidal, generate_multi_waypoint_trajectory,
    compute_path_length, smooth_trajectory,
)
from . import palette as P


# ═══════════════════════════════════════════════════════════════════════
# Unified palette — dark, consistent with gui/palette.py
# ═══════════════════════════════════════════════════════════════════════

BG_DARK = P.DARK_BG
BG_PANEL = P.DARK_PANEL
BG_CARD = P.DARK_ROW_ALT
BG_INPUT = P.DARK_INPUT
BORDER = P.DARK_BORDER
BORDER_FOCUS = P.DARK_ACCENT
TEXT_PRIMARY = P.DARK_TEXT
TEXT_SECONDARY = P.DARK_TEXT_DIM
TEXT_MUTED = P.DARK_TEXT_MUTED
ACCENT = P.DARK_ACCENT
ACCENT_HOVER = P.DARK_ACCENT_HOVER
GREEN = P.DARK_SUCCESS
GREEN_DIM = P.DARK_SUCCESS_DIM
AMBER = P.DARK_WARNING
RED = P.DARK_ERROR
RED_DIM = P.DARK_ERROR_DIM
PURPLE = P.DARK_ACCENT
ORANGE = P.DARK_WARNING

# Joint colors: brand green/blue family so every joint trace reads on-brand.
JOINT_COLORS = ["#7CB342", "#1E88E5", "#8bc34a", "#3a9af5", "#2ECC71", "#4A9BE8"]

SECTION_STYLE = f"""
    QGroupBox {{
        color: {TEXT_SECONDARY}; font-size: 12px; font-weight: bold;
        border: 1px solid {BORDER}; border-radius: 6px;
        margin-top: 10px; padding: 14px 10px 10px 10px;
        background: {BG_PANEL};
    }}
    QGroupBox::title {{
        subcontrol-origin: margin; left: 12px; padding: 0 6px;
        color: {ACCENT};
    }}
"""

BUTTON = f"""
    QPushButton {{
        background: {P.DARK_BUTTON}; color: {TEXT_PRIMARY};
        border: 1px solid {BORDER}; border-radius: 4px;
        padding: 6px 14px; font-size: 12px; font-weight: bold;
    }}
    QPushButton:hover {{ background: {P.DARK_BUTTON_HOVER}; border: 1px solid {ACCENT}; }}
    QPushButton:pressed {{ background: {P.DARK_ACCENT}; color: #1a1a16; padding-top: 2px; }}
    QPushButton:disabled {{ background: {P.DARK_BUTTON}; color: {TEXT_MUTED}; border: 1px solid {P.DARK_BORDER_SOFT}; }}
"""

BUTTON_ACCENT = f"""
    QPushButton {{
        background: {ACCENT}; color: #1a1a16;
        border: none; border-radius: 4px;
        padding: 6px 14px; font-size: 12px; font-weight: bold;
    }}
    QPushButton:hover {{ background: {ACCENT_HOVER}; }}
    QPushButton:pressed {{ background: {P.DARK_ACCENT}; padding-top: 2px; }}
"""

BUTTON_GREEN = f"""
    QPushButton {{
        background: {GREEN}; color: #1a1a16;
        border: 1px solid {P.lighten(GREEN, 20)}; border-radius: 4px;
        padding: 6px 14px; font-size: 12px; font-weight: bold;
    }}
    QPushButton:hover {{ background: {P.lighten(GREEN, 15)}; }}
    QPushButton:pressed {{ background: {GREEN_DIM}; padding-top: 2px; }}
"""

BUTTON_RED = f"""
    QPushButton {{
        background: {RED}; color: #fff;
        border: 1px solid {P.lighten(RED, 20)}; border-radius: 4px;
        padding: 6px 14px; font-size: 12px; font-weight: bold;
    }}
    QPushButton:hover {{ background: {P.lighten(RED, 15)}; }}
    QPushButton:pressed {{ background: {RED_DIM}; padding-top: 2px; }}
"""

BUTTON_AMBER = f"""
    QPushButton {{
        background: {AMBER}; color: #1a1a16;
        border: 1px solid {P.lighten(AMBER, 20)}; border-radius: 4px;
        padding: 6px 14px; font-size: 12px; font-weight: bold;
    }}
    QPushButton:hover {{ background: {P.lighten(AMBER, 15)}; }}
    QPushButton:pressed {{ background: {P.DARK_WARNING}; padding-top: 2px; }}
"""

SPINBOX = f"""
    QDoubleSpinBox, QSpinBox {{
        background: {BG_INPUT}; color: {TEXT_PRIMARY};
        border: 1px solid {BORDER}; border-radius: 3px;
        padding: 4px 8px; font-size: 12px; font-weight: bold;
    }}
    QDoubleSpinBox:hover, QSpinBox:hover {{ border: 1px solid {ACCENT}; }}
    QDoubleSpinBox:focus, QSpinBox:focus {{ border: 1px solid {ACCENT}; }}
"""

COMBO_STYLE = f"""
    QComboBox {{
        background: {BG_INPUT}; color: {TEXT_PRIMARY};
        border: 1px solid {BORDER}; border-radius: 3px;
        padding: 4px 8px; font-size: 12px;
    }}
    QComboBox:hover {{ border: 1px solid {ACCENT}; }}
    QComboBox::drop-down {{ border: none; width: 20px; }}
    QComboBox QAbstractItemView {{
        background: {BG_PANEL}; color: {TEXT_PRIMARY};
        border: 1px solid {BORDER}; selection-background-color: {ACCENT};
        selection-color: #1a1a16; font-size: 12px;
    }}
"""

SMALL_BTN = f"""
    QPushButton {{
        background: {P.DARK_BUTTON}; color: {TEXT_SECONDARY};
        border: 1px solid {BORDER}; border-radius: 3px;
        padding: 3px 8px; font-size: 10px; font-weight: bold;
    }}
    QPushButton:hover {{ background: {P.DARK_BUTTON_HOVER}; border: 1px solid {ACCENT}; color: {TEXT_PRIMARY}; }}
    QPushButton:pressed {{ background: {P.DARK_BUTTON_ACTIVE}; color: #1a1a16; }}
"""


# ═══════════════════════════════════════════════════════════════════════
# Trajectory Preview — polished industrial chart widget
# ═══════════════════════════════════════════════════════════════════════

class TrajectoryPreview(QWidget):
    """Professional trajectory profile chart with grid, shading, and joint color legend."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(200)
        self.trajectory = None
        self.setStyleSheet(f"background: {BG_DARK}; border: 1px solid {BORDER}; border-radius: 4px;")

    def set_trajectory(self, traj):
        self.trajectory = traj
        self.update()

    def clear(self):
        self.trajectory = None
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # ── Background ──────────────────────────────────────────────
        bg = QLinearGradient(0, 0, 0, h)
        bg.setColorAt(0, QColor(10, 10, 12))
        bg.setColorAt(1, QColor(14, 14, 18))
        p.fillRect(0, 0, w, h, bg)

        # ── Border ──────────────────────────────────────────────────
        p.setPen(QPen(QColor(BORDER if self.trajectory else 40, 40, 50), 1))
        p.drawRoundedRect(1, 1, w - 2, h - 2, 3, 3)

        # ── Empty state ─────────────────────────────────────────────
        if self.trajectory is None or len(self.trajectory.points) < 2:
            p.setPen(QColor(TEXT_MUTED))
            font = QFont("Segoe UI", 11)
            p.setFont(font)
            p.drawText(self.rect(), Qt.AlignCenter,
                       "Set waypoints and press Generate to preview trajectory")
            p.end()
            return

        # ── Layout ──────────────────────────────────────────────────
        ml, mr, mt, mb = 55, 16, 24, 38
        pw = w - ml - mr
        ph = h - mt - mb

        times = self.trajectory.get_time_array()
        positions = self.trajectory.get_position_matrix()
        if positions.size == 0:
            p.end()
            return

        n_joints = positions.shape[1]
        t_min, t_max = times[0], times[-1]
        p_min = float(np.min(positions))
        p_max = float(np.max(positions))
        prange = max(p_max - p_min, 0.1)

        # Add 10% padding to Y range
        ypad = prange * 0.1
        p_min -= ypad
        p_max += ypad
        prange = p_max - p_min

        def to_screen(t, val):
            x = ml + (t - t_min) / (t_max - t_min) * pw
            y = mt + ph - (val - p_min) / prange * ph
            return QPointF(x, y)

        # ── Grid ────────────────────────────────────────────────────
        p.setPen(QPen(QColor(22, 22, 28), 1))
        num_v = 5
        for i in range(num_v):
            y = mt + ph * i / (num_v - 1)
            p.drawLine(QPointF(ml, y), QPointF(ml + pw, y))
        num_h = 6
        for i in range(num_h):
            x = ml + pw * i / (num_h - 1)
            p.drawLine(QPointF(x, mt), QPointF(x, mt + ph))

        # ── Axes ────────────────────────────────────────────────────
        p.setPen(QPen(QColor(50, 50, 70), 1))
        p.drawLine(QPointF(ml, mt), QPointF(ml, mt + ph))
        p.drawLine(QPointF(ml, mt + ph), QPointF(ml + pw, mt + ph))

        # ── Tick labels ─────────────────────────────────────────────
        font = QFont("Segoe UI", 8)
        p.setFont(font)
        for i in range(num_v):
            val = p_min + prange * i / (num_v - 1)
            y = mt + ph - ph * i / (num_v - 1)
            p.setPen(QPen(QColor(40, 40, 55), 1))
            p.drawLine(QPointF(ml - 3, y), QPointF(ml, y))
            p.setPen(QColor(TEXT_MUTED))
            p.drawText(QRectF(0, y - 7, ml - 6, 14), Qt.AlignRight | Qt.AlignVCenter, f"{val:.1f}")

        for i in range(num_h):
            val = t_min + (t_max - t_min) * i / (num_h - 1)
            x = ml + pw * i / (num_h - 1)
            p.setPen(QPen(QColor(40, 40, 55), 1))
            p.drawLine(QPointF(x, mt + ph), QPointF(x, mt + ph + 3))
            p.setPen(QColor(TEXT_MUTED))
            p.drawText(QRectF(x - 18, mt + ph + 5, 36, 14), Qt.AlignCenter, f"{val:.1f}")

        # ── Axis titles ─────────────────────────────────────────────
        p.setPen(QColor(TEXT_SECONDARY))
        font_small = QFont("Segoe UI", 7)
        p.setFont(font_small)
        p.drawText(QRectF(ml, h - 16, pw, 14), Qt.AlignCenter, "Time (s)")
        p.save()
        p.translate(12, mt + ph / 2)
        p.rotate(-90)
        p.drawText(QRectF(-40, -7, 80, 14), Qt.AlignCenter, "Joint angle (rad)")
        p.restore()

        # ── Trajectory lines ────────────────────────────────────────
        p.setRenderHint(QPainter.Antialiasing)
        for j in range(n_joints):
            color = QColor(JOINT_COLORS[j % 6])
            pen = QPen(color, 1.8)
            pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen)

            path = QPainterPath()
            for i in range(len(times)):
                pt = to_screen(times[i], positions[i, j])
                path.lineTo(pt) if i else path.moveTo(pt)
            p.drawPath(path)

        # ── Joint legend ────────────────────────────────────────────
        lx = ml + 6
        ly = mt + 4
        for j in range(min(n_joints, 6)):
            color = QColor(JOINT_COLORS[j % 6])
            # Small colored dot
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(color))
            p.drawRoundedRect(lx, ly, 7, 7, 1, 1)
            # Label
            p.setPen(QColor(TEXT_PRIMARY))
            font_l = QFont("Segoe UI", 7, QFont.Bold)
            p.setFont(font_l)
            p.drawText(QRectF(lx + 9, ly - 2, 24, 12), Qt.AlignLeft | Qt.AlignVCenter,
                       f"J{j+1}")
            lx += 36

        p.end()


# ═══════════════════════════════════════════════════════════════════════
# StepBar — visual waypoint step indicator widget
# ═══════════════════════════════════════════════════════════════════════

class StepBar(QWidget):
    """Horizontal step indicator showing waypoint sequence as numbered circles."""

    selected = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(28)
        self._count = 0
        self._selected = -1
        self._labels = []
        self._collision_wps = set()

    def set_waypoints(self, count, labels=None, collision_set=None):
        self._count = count
        self._labels = list(labels) if labels else []
        self._collision_wps = set(collision_set) if collision_set else set()
        self.update()

    def set_selected(self, idx):
        self._selected = idx
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        if self._count < 2:
            p.setPen(QColor(TEXT_MUTED))
            font = QFont("Segoe UI", 10)
            p.setFont(font)
            p.drawText(self.rect(), Qt.AlignCenter,
                       "Add 2+ waypoints to generate trajectory")
            p.end()
            return

        spacing = min(48, (w - 20) / max(self._count, 1))
        start_x = (w - spacing * (self._count - 1)) / 2
        dot_r = 7
        conn_y = h / 2

        # Connection lines
        p.setPen(QPen(QColor(BORDER), 1.5))
        for i in range(self._count - 1):
            x1 = start_x + i * spacing
            x2 = start_x + (i + 1) * spacing
            p.drawLine(QPointF(x1, conn_y), QPointF(x2, conn_y))

        # Dots
        for i in range(self._count):
            cx = start_x + i * spacing
            label = self._labels[i] if i < len(self._labels) else f"WP{i+1}"

            # Collision highlight
            in_collision = i in self._collision_wps

            # Dot fill
            if i == self._selected:
                fill = QColor(ACCENT)
                border_c = QColor(ACCENT_HOVER)
                r = dot_r + 2
            elif in_collision:
                fill = QColor(RED)
                border_c = QColor("#ff4466")
                r = dot_r + 1
            else:
                fill = QColor(BG_INPUT)
                border_c = QColor(BORDER)
                r = dot_r

            p.setPen(QPen(border_c, 1.5))
            p.setBrush(QBrush(fill))
            p.drawEllipse(QPointF(cx, conn_y), r, r)

            # Number
            p.setPen(QColor(TEXT_PRIMARY) if i == self._selected else QColor(TEXT_SECONDARY))
            font = QFont("Segoe UI", 7, QFont.Bold)
            p.setFont(font)
            p.drawText(QRectF(cx - 10, conn_y - 8, 20, 16), Qt.AlignCenter, str(i + 1))

            # Label below
            p.setPen(QColor(TEXT_MUTED))
            font_s = QFont("Segoe UI", 7)
            p.setFont(font_s)
            label_short = label[:8] + ".." if len(label) > 10 else label
            p.drawText(QRectF(cx - 20, conn_y + r + 3, 40, 14), Qt.AlignCenter, label_short)


# ═══════════════════════════════════════════════════════════════════════
# Main Path Planning Panel
# ═══════════════════════════════════════════════════════════════════════

class PathPlanningPanel(QWidget):
    """Path planning interface — industrial robot programming workflow."""

    trajectory_generated = pyqtSignal(object)
    trajectory_execute = pyqtSignal(object)
    waypoints_updated = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._dh_arm = None
        self._waypoints = []
        self._trajectory = None
        self._collision_scene = CollisionScene()
        self._obstacles = []
        self._collision_wps = set()
        self._setup_ui()

    def set_dh_arm(self, dh_arm):
        self._dh_arm = dh_arm
        self.robot_indicator.setText(
            f"Robot: {dh_arm.name}" if dh_arm else "(no robot loaded)"
        )

    def get_dh_arm(self):
        return self._dh_arm

    def get_trajectory(self):
        return self._trajectory

    def get_waypoints(self):
        return list(self._waypoints)

    def add_waypoint_from_joints(self, joint_angles, speed=50.0, name=""):
        wp = Waypoint(
            joint_angles=list(joint_angles), speed=speed,
            name=name or f"WP {len(self._waypoints) + 1}"
        )
        self._waypoints.append(wp)
        self._refresh_wp_ui()
        self.waypoints_updated.emit(self._waypoints)

    def clear_waypoints(self):
        self._waypoints = []
        self._refresh_wp_ui()
        self.waypoints_updated.emit(self._waypoints)

    # ── UI Construction ─────────────────────────────────────────────

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── Header bar ──────────────────────────────────────────────
        header = QWidget()
        header.setStyleSheet(f"background: {BG_PANEL}; border-radius: 4px;")
        hdr = QHBoxLayout(header)
        hdr.setContentsMargins(10, 6, 10, 6)
        hdr.setSpacing(8)

        title = QLabel("PATH PLANNING")
        title.setStyleSheet(f"color: {ACCENT}; font-size: 16px; font-weight: bold;")
        hdr.addWidget(title)

        self.robot_indicator = QLabel("(no robot loaded)")
        self.robot_indicator.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 12px;")
        hdr.addWidget(self.robot_indicator)

        hdr.addStretch()

        # Status pill
        self.status_pill = QLabel("Ready")
        self.status_pill.setStyleSheet(f"""
            color: {TEXT_MUTED}; font-size: 11px; font-weight: bold;
            background: {BG_INPUT}; border: 1px solid {BORDER};
            border-radius: 10px; padding: 3px 14px;
        """)
        hdr.addWidget(self.status_pill)

        root.addWidget(header)

        # ── Step bar ────────────────────────────────────────────────
        self.step_bar = StepBar()
        self.step_bar.setStyleSheet(f"background: {BG_PANEL}; border-radius: 4px;")
        root.addWidget(self.step_bar)

        # ── Main content: splitter ──────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(2)
        splitter.setStyleSheet(f"QSplitter::handle {{ background: {BORDER}; }}")

        # ═══ LEFT: Waypoint table + controls ════════════════════════
        left = QWidget()
        lo = QVBoxLayout(left)
        lo.setContentsMargins(0, 0, 6, 0)
        lo.setSpacing(5)

        # -- Waypoint table --
        wp_label = QLabel("Waypoint Sequence")
        wp_label.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 11px; "
                               "font-weight: bold; letter-spacing: 1px;")
        lo.addWidget(wp_label)

        self.wp_table = QTableWidget()
        self.wp_table.setColumnCount(4)
        self.wp_table.setHorizontalHeaderLabels(["#", "Name", "Angles (deg)", "Speed"])
        self.wp_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
        self.wp_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.wp_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.wp_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Fixed)
        self.wp_table.setColumnWidth(0, 28)
        self.wp_table.setColumnWidth(3, 60)
        self.wp_table.verticalHeader().hide()
        self.wp_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.wp_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.wp_table.setDragDropMode(QAbstractItemView.InternalMove)
        self.wp_table.setDragDropOverwriteMode(False)
        self.wp_table.setAlternatingRowColors(True)
        self.wp_table.setStyleSheet(f"""
            QTableWidget {{
                background: {BG_INPUT}; color: {TEXT_PRIMARY};
                border: 1px solid {BORDER}; border-radius: 4px;
                gridline-color: {P.DARK_BORDER_SOFT}; font-size: 12px;
                selection-background-color: {ACCENT}; selection-color: #1a1a16;
                outline: none;
            }}
            QTableWidget::item {{ padding: 6px 8px; }}
            QTableWidget::item:alternate {{ background: {P.DARK_ROW_ALT}; }}
            QHeaderView::section {{
                background: {BG_PANEL}; color: {TEXT_SECONDARY};
                border: 1px solid {BORDER}; padding: 5px 8px;
                font-size: 10px; font-weight: bold; text-transform: uppercase;
            }}
        """)
        self.wp_table.cellClicked.connect(self._on_wp_table_select)
        self.wp_table.cellChanged.connect(self._on_wp_table_edit)
        # Detect drag-drop reorder
        self.wp_table.model().rowsMoved.connect(self._on_wp_reordered)
        lo.addWidget(self.wp_table, 1)

        # -- Waypoint action buttons --
        wp_actions = QHBoxLayout()
        wp_actions.setSpacing(4)
        btn_configs = [
            ("Record", BUTTON_ACCENT, self._on_record_wp),
            ("Add", BUTTON_GREEN, self._on_add_empty_wp),
            ("Delete", BUTTON_RED, self._on_delete_wp),
            ("Duplicate", BUTTON_AMBER, self._on_duplicate_wp),
            ("Clear All", BUTTON, self._on_clear_all_wp),
        ]
        for text, style, cb in btn_configs:
            btn = QPushButton(text)
            btn.setStyleSheet(style)
            btn.setFixedHeight(30)
            btn.clicked.connect(cb)
            wp_actions.addWidget(btn)
        lo.addLayout(wp_actions)

        # -- IK waypoint row: Cartesian target -> IK -> waypoint --
        ik_row = QHBoxLayout()
        ik_row.setSpacing(4)
        ik_label = QLabel("IK Waypoint:")
        ik_label.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 11px; font-weight: bold;")
        ik_row.addWidget(ik_label)
        self.ik_x = QDoubleSpinBox(); self.ik_x.setRange(-2000, 2000); self.ik_x.setDecimals(1)
        self.ik_x.setValue(300.0); self.ik_x.setFixedWidth(70); self.ik_x.setStyleSheet(SPINBOX)
        self.ik_y = QDoubleSpinBox(); self.ik_y.setRange(-2000, 2000); self.ik_y.setDecimals(1)
        self.ik_y.setValue(0.0); self.ik_y.setFixedWidth(70); self.ik_y.setStyleSheet(SPINBOX)
        self.ik_z = QDoubleSpinBox(); self.ik_z.setRange(-2000, 2000); self.ik_z.setDecimals(1)
        self.ik_z.setValue(500.0); self.ik_z.setFixedWidth(70); self.ik_z.setStyleSheet(SPINBOX)
        self.ik_rx = QDoubleSpinBox(); self.ik_rx.setRange(-180, 180); self.ik_rx.setDecimals(1)
        self.ik_rx.setValue(0.0); self.ik_rx.setFixedWidth(60); self.ik_rx.setStyleSheet(SPINBOX)
        self.ik_ry = QDoubleSpinBox(); self.ik_ry.setRange(-180, 180); self.ik_ry.setDecimals(1)
        self.ik_ry.setValue(0.0); self.ik_ry.setFixedWidth(60); self.ik_ry.setStyleSheet(SPINBOX)
        self.ik_rz = QDoubleSpinBox(); self.ik_rz.setRange(-180, 180); self.ik_rz.setDecimals(1)
        self.ik_rz.setValue(0.0); self.ik_rz.setFixedWidth(60); self.ik_rz.setStyleSheet(SPINBOX)
        ik_row.addWidget(self.ik_x); ik_row.addWidget(self.ik_y); ik_row.addWidget(self.ik_z)
        ik_row.addWidget(self.ik_rx); ik_row.addWidget(self.ik_ry); ik_row.addWidget(self.ik_rz)
        self.ik_add_btn = QPushButton("Solve + Add")
        self.ik_add_btn.setStyleSheet(BUTTON_ACCENT)
        self.ik_add_btn.setFixedHeight(26)
        self.ik_add_btn.clicked.connect(self._on_ik_waypoint)
        ik_row.addWidget(self.ik_add_btn)
        ik_row.addStretch()
        lo.addLayout(ik_row)

        # -- Trajectory type selector --
        traj_row = QHBoxLayout()
        traj_row.setSpacing(6)
        lbl_type = QLabel("Type:")
        lbl_type.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 12px; font-weight: bold;")
        traj_row.addWidget(lbl_type)
        self.traj_type = QComboBox()
        self.traj_type.addItems([
            "Joint Cubic (PTP)",
            "Joint Quintic",
            "Joint Trapezoidal",
            "Cartesian Linear",
        ])
        self.traj_type.setStyleSheet(COMBO_STYLE)
        self.traj_type.setFixedWidth(190)
        traj_row.addWidget(self.traj_type)

        lbl_dur = QLabel("Duration per segment:")
        lbl_dur.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 12px;")
        traj_row.addWidget(lbl_dur)
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.2, 60)
        self.duration_spin.setValue(2.0)
        self.duration_spin.setSingleStep(0.5)
        self.duration_spin.setFixedWidth(70)
        self.duration_spin.setStyleSheet(SPINBOX)
        traj_row.addWidget(self.duration_spin)
        dur_unit = QLabel("s")
        dur_unit.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 11px; background: transparent; border: none;")
        traj_row.addWidget(dur_unit)

        lbl_pts = QLabel("Points per segment:")
        lbl_pts.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 12px;")
        traj_row.addWidget(lbl_pts)
        self.points_spin = QSpinBox()
        self.points_spin.setRange(10, 500)
        self.points_spin.setValue(50)
        self.points_spin.setSingleStep(10)
        self.points_spin.setFixedWidth(60)
        self.points_spin.setStyleSheet(SPINBOX)
        traj_row.addWidget(self.points_spin)

        traj_row.addStretch()
        lo.addLayout(traj_row)

        left.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        splitter.addWidget(left)

        # ═══ RIGHT: Preview + Info + Controls ═══════════════════════
        right = QWidget()
        ro = QVBoxLayout(right)
        ro.setContentsMargins(6, 0, 0, 0)
        ro.setSpacing(5)

        # -- Trajectory info strip --
        info_strip = QWidget()
        info_strip.setStyleSheet(f"background: {BG_PANEL}; border: 1px solid {BORDER}; border-radius: 4px;")
        info = QHBoxLayout(info_strip)
        info.setContentsMargins(10, 4, 10, 4)
        info.setSpacing(16)

        self._info_labels = {}
        for key, label, default in [
            ("pts", "Points", "0"),
            ("time", "Duration", "0.0 s"),
            ("wps", "Waypoints", "0"),
            ("length", "Path length", "0.000 m"),
            ("collision", "Collision", "-"),
        ]:
            lbl = QLabel(f"{label}: {default}")
            lbl.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 11px;")
            self._info_labels[key] = lbl
            info.addWidget(lbl)
            if key != "collision":
                sep = QLabel("•")
                sep.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 10px;")
                info.addWidget(sep)
        info.addStretch()
        ro.addWidget(info_strip)

        # -- Trajectory preview chart --
        chart_group = QGroupBox("Trajectory Profile")
        chart_group.setStyleSheet(SECTION_STYLE)
        chart_lo = QVBoxLayout(chart_group)
        chart_lo.setContentsMargins(4, 4, 4, 4)
        self.preview = TrajectoryPreview()
        chart_lo.addWidget(self.preview, 1)
        ro.addWidget(chart_group, 1)

        # -- End-effector pose row --
        ee_row = QWidget()
        ee_row.setStyleSheet(f"background: {BG_PANEL}; border: 1px solid {BORDER}; border-radius: 4px;")
        ee = QHBoxLayout(ee_row)
        ee.setContentsMargins(10, 6, 10, 6)
        ee.setSpacing(20)
        self.start_pose = QLabel("Start:\n  (0.000, 0.000, 0.000)")
        self.start_pose.setStyleSheet(f"color: {GREEN}; font-size: 12px; font-family: monospace; font-weight: bold; background: transparent; border: none;")
        ee.addWidget(self.start_pose)
        sep_v = QLabel("")
        sep_v.setFixedWidth(1)
        sep_v.setStyleSheet(f"background: {BORDER};")
        sep_v.setFixedHeight(24)
        ee.addWidget(sep_v)
        self.end_pose = QLabel("End:\n  (0.000, 0.000, 0.000)")
        self.end_pose.setStyleSheet(f"color: {ACCENT}; font-size: 12px; font-family: monospace; font-weight: bold; background: transparent; border: none;")
        ee.addWidget(self.end_pose)
        ee.addStretch()
        ro.addWidget(ee_row)

        # -- Action buttons --
        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        for text, style, cb in [
            ("Generate", BUTTON_GREEN, self._on_generate),
            ("Execute", BUTTON_ACCENT, self._on_execute),
            ("Smooth", BUTTON_AMBER, self._on_smooth),
            ("Save", BUTTON, self._on_save_trajectory),
            ("Load", BUTTON, self._on_load_trajectory),
        ]:
            btn = QPushButton(text)
            btn.setStyleSheet(style)
            btn.setFixedHeight(30)
            btn.clicked.connect(cb)
            btn_row.addWidget(btn)
        btn_row.addStretch()
        ro.addLayout(btn_row)

        # -- Collision section (collapsed by default) --
        self.collision_check = QCheckBox("Enable collision checking")
        self.collision_check.setChecked(True)
        self.collision_check.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 12px; spacing: 8px; background: transparent; border: none;")
        ro.addWidget(self.collision_check)

        obs_lo = QHBoxLayout()
        obs_lo.setSpacing(4)
        self.obs_list = QListWidget()
        self.obs_list.setMaximumHeight(80)
        self.obs_list.setStyleSheet(f"""
            QListWidget {{
                background: {BG_INPUT}; color: {TEXT_SECONDARY};
                border: 1px solid {BORDER}; border-radius: 3px;
                font-size: 10px; outline: none;
            }}
            QListWidget::item {{ padding: 2px 6px; }}
        """)
        obs_lo.addWidget(self.obs_list, 1)
        for text, cb in [("+ Obstacle", self._on_add_obstacle), ("Clear", self._on_clear_obstacles)]:
            btn = QPushButton(text)
            btn.setStyleSheet(SMALL_BTN)
            btn.clicked.connect(cb)
            obs_lo.addWidget(btn)
        ro.addLayout(obs_lo)

        ro.addStretch()
        right.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        splitter.addWidget(right)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, 1)

    # ── Waypoint Table Management ───────────────────────────────────

    def _refresh_wp_ui(self):
        """Refresh waypoint table, step bar, and detail."""
        self.wp_table.blockSignals(True)
        self.wp_table.setRowCount(len(self._waypoints))
        for i, wp in enumerate(self._waypoints):
            idx_item = QTableWidgetItem(str(i + 1))
            idx_item.setFlags(idx_item.flags() & ~Qt.ItemIsEditable)
            idx_item.setTextAlignment(Qt.AlignCenter)
            self.wp_table.setItem(i, 0, idx_item)

            name_item = QTableWidgetItem(wp.name or f"WP {i+1}")
            self.wp_table.setItem(i, 1, name_item)

            joints_str = ", ".join(f"{j:.1f}" for j in wp.joint_angles[:6])
            joint_item = QTableWidgetItem(f"[{joints_str}]")
            joint_item.setFlags(joint_item.flags() & ~Qt.ItemIsEditable)
            joint_item.setToolTip(f"Joint angles: {joints_str}°")
            joint_item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.wp_table.setItem(i, 2, joint_item)

            speed_item = QTableWidgetItem(f"{wp.speed:.0f}%")
            speed_item.setTextAlignment(Qt.AlignCenter)
            self.wp_table.setItem(i, 3, speed_item)

            # Color rows
            for c in range(4):
                item = self.wp_table.item(i, c)
                if item:
                    bg = QColor(P.DARK_ROW_ALT) if i in self._collision_wps else QColor("transparent")
                    item.setBackground(bg)
                    if i in self._collision_wps:
                        item.setForeground(QColor(RED))

        self.wp_table.blockSignals(False)
        self._update_step_bar()

    def _update_step_bar(self):
        labels = [wp.name or f"WP {i+1}" for i, wp in enumerate(self._waypoints)]
        self.step_bar.set_waypoints(len(self._waypoints), labels, self._collision_wps)
        info_wps = self._info_labels.get("wps")
        if info_wps:
            info_wps.setText(f"Waypoints: {len(self._waypoints)}")

    def _on_wp_table_select(self, row, col):
        self.step_bar.set_selected(row)

    def _on_wp_table_edit(self, row, col):
        """Handle inline edits to waypoint name or speed."""
        if col == 1:
            item = self.wp_table.item(row, 1)
            if item and row < len(self._waypoints):
                self._waypoints[row].name = item.text()
        elif col == 3:
            item = self.wp_table.item(row, 3)
            if item and row < len(self._waypoints):
                try:
                    self._waypoints[row].speed = float(item.text().replace("%", ""))
                except ValueError:
                    pass

    def _on_wp_reordered(self):
        """Rebuild waypoint list from table order after drag-drop."""
        new_order = []
        for i in range(self.wp_table.rowCount()):
            idx = self.wp_table.item(i, 0)
            if idx:
                try:
                    orig_idx = int(idx.text()) - 1
                    if 0 <= orig_idx < len(self._waypoints):
                        new_order.append(self._waypoints[orig_idx])
                except ValueError:
                    continue
        if len(new_order) == len(self._waypoints):
            self._waypoints = new_order
            self._refresh_wp_ui()
            self.waypoints_updated.emit(self._waypoints)

    def _on_record_wp(self):
        if self._dh_arm is None:
            QMessageBox.warning(self, "No Kinematics",
                                "Load a robot model in the Kinematics tab first.")
            return
        angles = self._dh_arm.get_thetas(degrees=True)
        name, ok = QInputDialog.getText(
            self, "Record Waypoint", "Name:",
            text=f"WP {len(self._waypoints) + 1}"
        )
        if ok:
            self.add_waypoint_from_joints(angles, speed=50.0,
                                          name=name or f"WP {len(self._waypoints) + 1}")

    def _on_add_empty_wp(self):
        n = self._dh_arm.num_joints if self._dh_arm else 6
        self.add_waypoint_from_joints([0.0] * n, speed=50.0)

    def _on_ik_waypoint(self):
        """Solve IK for the Cartesian target and add the result as a waypoint.

        Uses the same solver stack as the Kinematics panel: built-in LM,
        then (if available) the ported repo IK / AR4 analytic as alternates.
        The seed is the current waypoint's joints (or the last waypoint),
        so consecutive IK waypoints track a smooth solution branch.
        """
        if self._dh_arm is None:
            QMessageBox.warning(self, "No Kinematics",
                                "Load a robot model in the Kinematics tab first.")
            return
        target = [self.ik_x.value(), self.ik_y.value(), self.ik_z.value()]
        rx = math.radians(self.ik_rx.value())
        ry = math.radians(self.ik_ry.value())
        rz = math.radians(self.ik_rz.value())
        target_orient = None
        if abs(rx) > 1e-6 or abs(ry) > 1e-6 or abs(rz) > 1e-6:
            cx, sx = math.cos(rx), math.sin(rx)
            cy, sy = math.cos(ry), math.sin(ry)
            cz, sz = math.cos(rz), math.sin(rz)
            Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            target_orient = Rz @ Ry @ Rx

        # Seed: current robot joints, else last waypoint, else zeros.
        try:
            seed = self._dh_arm.get_thetas(degrees=True)
        except Exception:
            seed = None
        if not seed:
            if self._waypoints:
                seed = list(self._waypoints[-1].joint_angles)
            else:
                seed = [0.0] * (self._dh_arm.num_joints or 6)
        seed_rad = [math.radians(a) for a in seed]

        # 1) Built-in LM solver (the studio's DHArm).
        result = None
        solver_name = "Built-in"
        try:
            result = self._dh_arm.compute_ik(target, target_orient,
                                             joint_angles=seed_rad)
        except Exception:
            result = None

        # 2) AR4 analytic (closed-form, verified) as an alternative.
        if result is None:
            try:
                from .kinematic_config import _ar4_ik, _ar4_fk, AR4_IK_AVAILABLE
                if AR4_IK_AVAILABLE:
                    # Convert orientation matrix to a rotation vector (deg).
                    R = np.eye(3) if target_orient is None else target_orient
                    cos_a = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
                    ang = math.acos(cos_a)
                    if ang < 1e-6:
                        rv = [0.0, 0.0, 0.0]
                    else:
                        rv = [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                              R[1, 0] - R[0, 1]]
                        rv = [math.degrees(v * ang / (2.0 * math.sin(ang)))
                              for v in rv]
                    deg = _ar4_ik(target + rv, seed)
                    if deg is not None:
                        chk = _ar4_fk([float(a) for a in deg])
                        pos_err = math.sqrt(
                            (chk[0] - target[0]) ** 2 +
                            (chk[1] - target[1]) ** 2 +
                            (chk[2] - target[2]) ** 2)
                        if pos_err < 1.0:
                            result = [math.radians(a) for a in deg]
                            solver_name = "AR4"
            except Exception:
                result = None

        # 3) Ported repo IK as a last alternative.
        if result is None:
            try:
                from .kinematic_config import (_ported_ik_raw, _repo_fk,
                                               PORTED_IK_AVAILABLE)
                if PORTED_IK_AVAILABLE and _repo_fk is not None:
                    def _studio_fk(q_deg):
                        q_rad = [math.radians(a) for a in q_deg]
                        self._dh_arm.set_thetas(q_rad)
                        T = self._dh_arm.forward()
                        return T[0, 3], T[1, 3], T[2, 3], T[:3, :3]
                    limits = []
                    for i in range(min(6, self._dh_arm.num_joints)):
                        name = (self._dh_arm.joint_names[i]
                                if i < len(self._dh_arm.joint_names)
                                else f"j{i}")
                        if name in self._dh_arm.joint_limits:
                            lo, hi = self._dh_arm.joint_limits[name]
                            limits.append((math.degrees(lo), math.degrees(hi)))
                        else:
                            limits.append((-180.0, 180.0))
                    deg = _ported_ik_raw(target, target_orient, seed=seed,
                                         joint_limits=limits, retries=4,
                                         fk_func=_studio_fk)
                    if deg is not None:
                        result = [math.radians(a) for a in deg]
                        solver_name = "Ported"
            except Exception:
                result = None

        if result is None:
            QMessageBox.warning(
                self, "IK Failed",
                "No IK solution for the target pose — it may be out of "
                "reach. Adjust X/Y/Z/R and try again.")
            return

        deg = [math.degrees(a) for a in result]
        # Always lock orientation: if no rotation was entered, use the
        # CURRENT robot orientation at the target so the path stays
        # orientation-constrained (no drift off the straight line).
        if target_orient is None:
            try:
                arm_q = self._dh_arm.get_thetas(degrees=True)
                self._dh_arm.set_thetas([math.radians(a) for a in arm_q])
                T = self._dh_arm.forward()
                target_orient = T[:3, :3]
            except Exception:
                target_orient = np.eye(3)
        quat = None
        if target_orient is not None:
            # rotation matrix -> quaternion [x, y, z, w]
            tr = np.trace(target_orient)
            if tr > 0:
                s = math.sqrt(tr + 1.0) * 2.0
                quat = [(target_orient[2, 1] - target_orient[1, 2]) / s,
                        (target_orient[0, 2] - target_orient[2, 0]) / s,
                        (target_orient[1, 0] - target_orient[0, 1]) / s,
                        0.25 * s]
            else:
                i = int(np.argmax(np.diag(target_orient)))
                j = (i + 1) % 3
                k = (j + 1) % 3
                s = math.sqrt(1.0 + target_orient[i, i] -
                              target_orient[j, j] - target_orient[k, k]) * 2.0
                quat = [(target_orient[i, j] + target_orient[j, i]) / s,
                        (target_orient[i, k] + target_orient[k, i]) / s,
                        (target_orient[j, k] + target_orient[k, j]) / s,
                        0.25 * s]
            # normalize
            n = math.sqrt(sum(v * v for v in quat))
            quat = [v / n for v in quat]

        wp = Waypoint(deg, speed=50.0,
                      position=list(target),
                      orientation=quat,
                      name=f"IK {self.ik_x.value():.0f},{self.ik_y.value():.0f},"
                           f"{self.ik_z.value():.0f} ({solver_name})")
        # Make the waypoint SELF-CONSISTENT: its position must be the FK of
        # the stored joints (the IK can land on a branch whose actual FK
        # differs slightly from the requested target). This keeps the
        # Cartesian path generation on a true straight line.
        try:
            self._dh_arm.set_thetas([math.radians(a) for a in deg])
            T = self._dh_arm.forward()
            wp.position = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
        except Exception:
            pass
        self._waypoints.append(wp)
        self._refresh_wp_ui()
        self.waypoints_updated.emit(self._waypoints)
        self.status_pill.setText(f"IK waypoint added ({solver_name})")

    def _on_delete_wp(self):
        row = self.wp_table.currentRow()
        if 0 <= row < len(self._waypoints):
            self._waypoints.pop(row)
            self._refresh_wp_ui()
            self.waypoints_updated.emit(self._waypoints)

    def _on_duplicate_wp(self):
        row = self.wp_table.currentRow()
        if 0 <= row < len(self._waypoints):
            wp = self._waypoints[row]
            dup = Waypoint(list(wp.joint_angles), speed=wp.speed,
                          name=f"{wp.name} (copy)")
            self._waypoints.insert(row + 1, dup)
            self._refresh_wp_ui()
            self.waypoints_updated.emit(self._waypoints)

    def _on_clear_all_wp(self):
        self._waypoints = []
        self._collision_wps = set()
        self._trajectory = None
        self._refresh_wp_ui()
        self.preview.clear()
        self.waypoints_updated.emit(self._waypoints)

    # ── Obstacle Management ─────────────────────────────────────────

    def _on_add_obstacle(self):
        dlg = QInputDialog(self)
        dlg.setWindowTitle("Add Collision Obstacle")
        dlg.setLabelText("Enter position (x, y, z) and radius:\nFormat: x, y, z, radius")
        dlg.setTextValue("0.3, 0.3, 0.5, 0.08")
        dlg.setStyleSheet(f"""
            QInputDialog {{ background: {BG_PANEL}; }}
            QLabel {{ color: {TEXT_PRIMARY}; font-size: 12px; }}
            QLineEdit {{ background: {BG_INPUT}; color: {TEXT_PRIMARY};
                       border: 1px solid {BORDER}; border-radius: 3px;
                       padding: 6px 10px; font-size: 12px; }}
        """)
        if dlg.exec_() == QInputDialog.Accepted:
            try:
                parts = [float(x.strip()) for x in dlg.textValue().split(",")]
                if len(parts) == 4:
                    x, y, z, r = parts
                    self._obstacles.append(CollisionSphere([x, y, z], r))
                    self._collision_scene.add_obstacle([x, y, z], r,
                                                       f"Obs {len(self._obstacles)}")
                    self._refresh_obs_list()
            except (ValueError, IndexError):
                QMessageBox.warning(self, "Invalid", "Format: x, y, z, radius")

    def _on_clear_obstacles(self):
        self._obstacles = []
        self._collision_scene.clear_obstacles()
        self.obs_list.clear()

    def _refresh_obs_list(self):
        self.obs_list.clear()
        for i, obs in enumerate(self._obstacles):
            c = obs.center
            self.obs_list.addItem(f"Obs {i+1}  ({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})  r={obs.radius:.3f}")

    # ── Trajectory Generation ───────────────────────────────────────

    def _on_generate(self):
        if len(self._waypoints) < 2:
            QMessageBox.warning(self, "Insufficient Waypoints",
                                "Add at least 2 waypoints to generate a trajectory.")
            return
        if self._dh_arm is None:
            QMessageBox.warning(self, "No Kinematics",
                                "Load a robot model in the Kinematics tab first.")
            return

        traj_type = self.traj_type.currentIndex()
        duration = self.duration_spin.value()
        points_per_seg = int(self.points_spin.value())

        type_map = {
            0: TrajectoryType.JOINT_CUBIC,
            1: TrajectoryType.JOINT_QUINTIC,
            2: TrajectoryType.JOINT_TRAPEZOID,
            3: TrajectoryType.JOINT_CUBIC,
        }
        tt = type_map.get(traj_type, TrajectoryType.JOINT_CUBIC)

        try:
            if traj_type == 3:
                # TRUE Cartesian linear: interpolate the straight line in
                # Cartesian space (position + slerp orientation) and solve IK
                # at every intermediate point. This LOCKS orientation along
                # the path — the robot tracks the line instead of sweeping.
                self._trajectory = self._generate_cartesian_ik_path(
                    duration, points_per_seg)
            else:
                self._trajectory = generate_multi_waypoint_trajectory(
                    self._waypoints, duration, points_per_seg, tt
                )

            # Post-process Cartesian LIN: compute Cartesian poses via FK
            if traj_type == 3:
                self._compute_cartesian_along_trajectory()

            self._collision_wps = set()
            collision_label = "Safe"
            col_color = GREEN

            if self.collision_check.isChecked():
                cols = self._collision_scene.check_trajectory_safety(
                    self._trajectory, self._dh_arm, stride=3
                )
                if cols:
                    # Find which waypoints are near collisions
                    for t, _ in cols:
                        for i, wp in enumerate(self._waypoints):
                            if i not in self._collision_wps:
                                self._collision_wps.add(i)
                    n = len(cols)
                    collision_label = f"{n} collision{'s' if n != 1 else ''} detected"
                    col_color = RED
                    self._status_pill("Collision!", RED)
                else:
                    self._status_pill("Safe", GREEN)
            else:
                self._status_pill("Generated", ACCENT)

            # Update info
            info = self._info_labels
            info["pts"].setText(f"Points: {self._trajectory.num_points}")
            info["time"].setText(f"Duration: {self._trajectory.total_time:.1f} s")
            info["length"].setText(f"Path length: {compute_path_length(self._trajectory):.4f} m")
            info["collision"].setText(f"Collision: {collision_label}")
            info["collision"].setStyleSheet(
                f"color: {col_color}; font-size: 11px; font-weight: bold;"
            )

            # EE pose
            self._update_ee_pose()

            # Preview
            self.preview.set_trajectory(self._trajectory)
            self._refresh_wp_ui()
            self.trajectory_generated.emit(self._trajectory)

        except Exception as e:
            QMessageBox.critical(self, "Generation Error", str(e))

    def _rot_slerp(self, R0, R1, t):
        """Slerp between two rotation matrices via axis-angle.

        Returns a 3x3 rotation matrix at fraction t along the shortest
        rotation from R0 to R1. Convention matches numpy row-major (what
        DHArm.forward() returns), so it plugs straight into compute_ik.
        """
        R0 = np.asarray(R0, dtype=float)
        R1 = np.asarray(R1, dtype=float)
        dR = R0.T @ R1
        # axis-angle of dR
        cos_a = max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0))
        ang = math.acos(cos_a)
        if ang < 1e-9:
            return R0
        axis = np.array([dR[2, 1] - dR[1, 2],
                         dR[0, 2] - dR[2, 0],
                         dR[1, 0] - dR[0, 1]])
        axis = axis / (2.0 * math.sin(ang))
        # rotate R0 by t*ang about axis
        half = t * ang / 2.0
        qw = math.cos(half)
        qx, qy, qz = axis * math.sin(half)
        # quaternion -> rotation matrix (numpy row-major)
        R_delta = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),   1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw),   1 - 2*(qx*qx + qy*qy)],
        ])
        return R0 @ R_delta

    def _slerp(self, qa, qb, t):
        """Spherical linear interpolation between quaternions [x,y,z,w]."""
        if qa is None or qb is None:
            return None
        qa = np.array(qa, dtype=float)
        qb = np.array(qb, dtype=float)
        dot = float(np.clip(np.dot(qa, qb), -1.0, 1.0))
        if dot < 0.0:
            qb = -qb
            dot = -dot
        if dot > 0.9995:
            q = qa + t * (qb - qa)
        else:
            theta = math.acos(dot)
            q = (math.sin((1 - t) * theta) * qa +
                 math.sin(t * theta) * qb) / math.sin(theta)
        n = np.linalg.norm(q)
        return (q / n).tolist() if n > 0 else qa.tolist()

    def _generate_cartesian_ik_path(self, duration, points_per_seg):
        """Build an orientation-locked Cartesian path via per-point IK.

        Waypoints must carry position (+ optional orientation quaternion).
        Between consecutive waypoints the line is interpolated in Cartesian
        space and IK is solved at each sample, seeded from the previous
        sample so the solution branch stays continuous.
        """
        from ..core.path_planning import Trajectory, TrajectoryPoint
        wps = self._waypoints
        if len(wps) < 2:
            raise ValueError("Need at least 2 waypoints")
        if not wps[0].position:
            raise ValueError("IK waypoints need a Cartesian position — "
                             "use 'Solve + Add' to create them")

        n_joints = self._dh_arm.num_joints or len(wps[0].joint_angles)
        traj = Trajectory(TrajectoryType.CARTESIAN_LIN)
        traj.num_joints = n_joints
        traj.total_time = duration * (len(wps) - 1)

        seed = None
        # Start from the FIRST waypoint's joints — the path begins exactly
        # there, so the seed must match it to stay on the same branch.
        if wps[0].joint_angles:
            seed = list(wps[0].joint_angles)
        if not seed:
            try:
                seed = self._dh_arm.get_thetas(degrees=True)
            except Exception:
                seed = None
        if not seed:
            seed = [0.0] * n_joints

        current_time = 0.0
        # Precompute each waypoint's exact orientation matrix from its FK —
        # self-consistent with the joints we stored (the stored quaternion
        # can be from a different solution branch).
        wp_orients = []
        for wp in wps:
            try:
                self._dh_arm.set_thetas([math.radians(a) for a in wp.joint_angles])
                T = self._dh_arm.forward()
                wp_orients.append(T[:3, :3].copy())
            except Exception:
                wp_orients.append(None)

        for seg in range(len(wps) - 1):
            p0 = wps[seg].position
            p1 = wps[seg + 1].position
            R0 = wp_orients[seg]
            R1 = wp_orients[seg + 1]
            # Verified joint anchors for fallback interpolation.
            j0 = np.array(wps[seg].joint_angles, dtype=float)
            j1 = np.array(wps[seg + 1].joint_angles, dtype=float)
            for k in range(points_per_seg + 1):
                t = k / points_per_seg
                pos = [p0[i] + t * (p1[i] - p0[i]) for i in range(3)]
                # Interpolate the orientation matrix directly (rotation-vector
                # slerp) — no quaternion convention ambiguity.
                target_orient = None
                if R0 is not None and R1 is not None:
                    target_orient = self._rot_slerp(R0, R1, t)
                # Solve IK (position-only if no orientation given). The
                # base_weight regularizer keeps J1/J2/J3 near the seed so the
                # arm base orientation stays FIXED and the wrist (J4/J5/J6)
                # absorbs reorientation — orientation-lock behavior.
                seed_rad = [math.radians(a) for a in seed]
                sol = None
                try:
                    sol = self._dh_arm.compute_ik(
                        pos, target_orient, joint_angles=seed_rad,
                        base_weight=2.0)
                except Exception:
                    sol = None
                # VERIFY the solution actually reaches the target pose. If the
                # solver landed on a 180° wrist-flip branch (invisible to its
                # own axis-vector residual), retry with a flipped J5/J6 seed.
                def _verify_sol(s):
                    try:
                        self._dh_arm.set_thetas(s)
                        T = self._dh_arm.forward()
                        pos_ok = np.linalg.norm(T[:3, 3] - np.array(pos)) < 0.005
                        orient_ok = True
                        if target_orient is not None:
                            cos_a = np.clip(
                                (np.trace(target_orient.T @ T[:3, :3]) - 1.0) / 2.0,
                                -1.0, 1.0)
                            orient_ok = math.degrees(math.acos(cos_a)) < 2.0
                        return pos_ok and orient_ok
                    except Exception:
                        return False

                verified = _verify_sol(sol) if sol is not None else False
                if not verified:
                    # Retry from a wrist-flipped seed: J5 += 180, J6 += 180
                    # reaches the same position with the OTHER orientation.
                    flip_seed = list(seed)
                    if len(flip_seed) >= 6:
                        flip_seed[4] = (flip_seed[4] + 180.0) % 360.0
                        flip_seed[5] = (flip_seed[5] + 180.0) % 360.0
                    try:
                        sol2 = self._dh_arm.compute_ik(
                            pos, target_orient,
                            joint_angles=[math.radians(a) for a in flip_seed])
                        if _verify_sol(sol2):
                            sol = sol2
                            verified = True
                    except Exception:
                        pass
                if sol is None or not verified:
                    # joint-space fallback: linear blend of the anchors
                    deg = [j0[i] + t * (j1[i] - j0[i]) for i in range(len(j0))]
                    seed = deg
                else:
                    deg = [math.degrees(a) for a in sol]
                    seed = deg
                traj.add_point(TrajectoryPoint(
                    time=current_time + t * duration,
                    positions=np.array(deg, dtype=float),
                    position_cartesian=list(pos),
                    orientation_cartesian=target_orient,
                ))
            current_time += duration
        return traj

    def _compute_cartesian_along_trajectory(self):
        """Compute Cartesian poses for each point via FK."""
        if self._dh_arm is None:
            return
        for pt in self._trajectory.points:
            # pt.positions are DEGREES; set_thetas needs them converted.
            self._dh_arm.set_thetas(pt.positions, degrees=True)
            T = self._dh_arm.forward()
            pt.position_cartesian = list(T[:3, 3])
            # Keep the actual rotation matrix as the orientation reference.
            pt.orientation_cartesian = T[:3, :3].copy()

    def _on_smooth(self):
        if self._trajectory is None or len(self._trajectory.points) < 5:
            return
        self._trajectory = smooth_trajectory(self._trajectory, 5)
        self.preview.set_trajectory(self._trajectory)
        info = self._info_labels
        info["pts"].setText(f"Points: {self._trajectory.num_points}")
        info["length"].setText(f"Path length: {compute_path_length(self._trajectory):.4f} m")

    def _on_execute(self):
        if self._trajectory is None:
            QMessageBox.warning(self, "No Trajectory", "Generate a trajectory first.")
            return
        self.trajectory_execute.emit(self._trajectory)
        self._status_pill("Executing...", GREEN)

    def _status_pill(self, text, color=ACCENT):
        self.status_pill.setText(text)
        self.status_pill.setStyleSheet(f"""
            color: {color}; font-size: 11px; font-weight: bold;
            background: {BG_INPUT}; border: 1px solid {color};
            border-radius: 10px; padding: 3px 14px;
        """)

    def _update_ee_pose(self):
        if not self._trajectory or len(self._trajectory.points) < 1 or self._dh_arm is None:
            return
        start_q = self._trajectory.points[0].positions
        self._dh_arm.set_thetas(start_q)
        Ts = self._dh_arm.forward()
        ps = Ts[:3, 3]
        self.start_pose.setText(
            f"Start:\n  ({ps[0]:.4f}, {ps[1]:.4f}, {ps[2]:.4f})"
        )

        end_q = self._trajectory.points[-1].positions
        self._dh_arm.set_thetas(end_q)
        Te = self._dh_arm.forward()
        pe = Te[:3, 3]
        self.end_pose.setText(
            f"End:\n  ({pe[0]:.4f}, {pe[1]:.4f}, {pe[2]:.4f})"
        )

    # ── Save / Load ─────────────────────────────────────────────────

    def _on_save_trajectory(self):
        if self._trajectory is None:
            QMessageBox.warning(self, "No Trajectory", "Generate a trajectory first.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Trajectory", "",
            "Trajectory (*.traj.json);;JSON (*.json);;All (*.*)"
        )
        if path:
            data = {
                "trajectory": self._trajectory.to_dict(),
                "waypoints": [wp.to_dict() for wp in self._waypoints],
                "traj_type_idx": self.traj_type.currentIndex(),
                "duration": self.duration_spin.value(),
                "points_per_seg": int(self.points_spin.value()),
            }
            with open(path, "w") as f:
                json.dump(data, f, indent=2)

    def _on_load_trajectory(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Trajectory", "",
            "Trajectory (*.traj.json *.json);;All (*.*)"
        )
        if path:
            try:
                with open(path) as f:
                    data = json.load(f)
                td = data.get("trajectory", {})
                self._trajectory = Trajectory(TrajectoryType.JOINT_CUBIC)
                for pt in td.get("points", []):
                    self._trajectory.add_point(TrajectoryPoint(
                        time=pt["time"], positions=pt["positions"],
                        velocities=pt.get("velocities"),
                        accelerations=pt.get("accelerations"),
                    ))
                self._trajectory.total_time = td.get("total_time", 0)
                self._trajectory.num_joints = td.get("num_joints", 0)
                self._waypoints = [Waypoint.from_dict(wp) for wp in data.get("waypoints", [])]
                self.traj_type.setCurrentIndex(data.get("traj_type_idx", 0))
                self.duration_spin.setValue(data.get("duration", 2.0))
                self.points_spin.setValue(data.get("points_per_seg", 50))
                self._refresh_wp_ui()
                self.preview.set_trajectory(self._trajectory)
                info = self._info_labels
                info["pts"].setText(f"Points: {self._trajectory.num_points}")
                info["time"].setText(f"Duration: {self._trajectory.total_time:.1f} s")
                info["length"].setText(f"Path length: {compute_path_length(self._trajectory):.4f} m")
                self._update_ee_pose()
            except Exception as e:
                QMessageBox.critical(self, "Load Error", str(e))
