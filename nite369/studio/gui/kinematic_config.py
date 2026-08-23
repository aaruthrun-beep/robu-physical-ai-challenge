"""Kinematic Configuration Panel — URDF loading, DH parameters, joint limits, FK preview.

Provides a complete GUI for:
  - Loading any standard URDF robot model
  - Viewing/editing DH parameters per joint
  - Configuring joint limits, gear ratios, home positions
  - Precise spinbox editing for all numeric values
  - FK preview with end-effector position display
  - Save/load robot configuration to JSON
  - Automatic detection of kinematic chain from URDF
"""

import os
import math
import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QFileDialog, QMessageBox, QTabWidget,
    QTableWidget, QTableWidgetItem, QHeaderView, QSpinBox,
    QDoubleSpinBox, QComboBox, QCheckBox, QGridLayout,
    QScrollArea, QFrame, QSizePolicy, QSplitter,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont, QDoubleValidator

from ..core.urdf_parser import URDFModel
from ..core.kinematics import DHArm, DHParameter, dh_transform
from ..core.robot import RobotModel
from . import palette as P

# Ported from Walid-khaled/6DOF-Robot-Trajectory-Planning (MATLAB) — a
# self-contained numerical IK for a 6-DOF arm. Used as an alternate solver
# in the Kinematics panel.  When a URDF is loaded, a custom FK wrapper is
# passed so the solver matches the studio's arm, not just the repo's.
try:
    import os as _os, sys as _sys
    _repo_root = _os.path.dirname(_os.path.dirname(
        _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))
    _repo_ik = _os.path.join(_repo_root, "trajectory_planning")
    if _repo_ik not in _sys.path:
        _sys.path.insert(0, _repo_ik)
    from trajectory_planning import inverse_kinematics as _ported_ik_raw
    from trajectory_planning import fk as _repo_fk
    PORTED_IK_AVAILABLE = True
except ImportError:
    _repo_fk = None
    PORTED_IK_AVAILABLE = False

# AR4-MK3 analytic IK — ported from Annin Robotics' HMI
# (ARrobots/src/kinematics.cpp). Closed-form, sweeps J5 for solutions.
try:
    from ar4_kinematics import (solve_inverse_kinematics as _ar4_ik,
                                forward_kinematics_xyzuvw as _ar4_fk)
    AR4_IK_AVAILABLE = True
except ImportError:
    AR4_IK_AVAILABLE = False


# ── Style Constants ───────────────────────────────────────────────────

SECTION_STYLE = P.groupbox_style(font_size=12, padding="14px 12px 12px 12px")

BUTTON_STYLE = P.btn_style(P.DARK_BUTTON, font_size=12, padding="6px 16px")

ACCENT_BUTTON = P.accent_btn_style(font_size=12, padding="6px 16px")

SUCCESS_BUTTON = P.success_btn_style(font_size=12, padding="6px 16px")

TABLE_STYLE = f"""
    QTableWidget {{
        background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
        border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
        gridline-color: {P.DARK_BORDER_SOFT}; font-size: 12px;
        selection-background-color: {P.DARK_ACCENT};
        selection-color: #1a1a16;
    }}
    QTableWidget::item {{ padding: 4px 6px; }}
    QHeaderView::section {{
        background: {P.DARK_PANEL}; color: {P.DARK_TEXT};
        border: 1px solid {P.DARK_BORDER}; padding: 4px 6px;
        font-size: 11px; font-weight: bold;
    }}
    QTableWidget::item:alternate {{ background: {P.DARK_ROW_ALT}; }}
"""

SPINBOX_STYLE = P.input_style(font_size=12)

COMBO_STYLE = f"""
    QComboBox {{
        background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
        border: 1px solid {P.DARK_BORDER}; border-radius: 4px;
        padding: 2px 6px; font-size: 12px;
    }}
    QComboBox::drop-down {{ border: none; width: 16px; }}
    QComboBox QAbstractItemView {{
        background: {P.DARK_PANEL}; color: {P.DARK_TEXT};
        border: 1px solid {P.DARK_BORDER};
        selection-background-color: {P.DARK_ACCENT};
    }}
"""


class DHParamEditor(QWidget):
    """Spinbox editor for a single DH parameter row."""

    changed = pyqtSignal()

    def __init__(self, label_text, value, unit="", decimals=4, step=0.01, range_=(-10, 10),
                 parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(8)

        self.label = QLabel(label_text)
        self.label.setFixedWidth(100)
        self.label.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        layout.addWidget(self.label)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(range_[0], range_[1])
        self.spin.setDecimals(decimals)
        self.spin.setSingleStep(step)
        self.spin.setValue(value)
        self.spin.setFixedWidth(120)
        self.spin.setStyleSheet(SPINBOX_STYLE)
        self.spin.valueChanged.connect(self.changed.emit)
        layout.addWidget(self.spin)

        self.unit_label = QLabel(unit)
        self.unit_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; background: transparent; border: none;")
        layout.addWidget(self.unit_label)

        layout.addStretch()

    def value(self):
        return self.spin.value()

    def set_value(self, v):
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)


class JointParamWidget(QWidget):
    """Parameter controls for a single joint (limits, gear ratio, home)."""

    changed = pyqtSignal()

    def __init__(self, joint_name, index, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.index = index

        layout = QGridLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(8)
        layout.setVerticalSpacing(6)

        # Joint name header
        name_label = QLabel(f"#{index+1}  {joint_name}")
        name_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
        layout.addWidget(name_label, 0, 0, 1, 4)

        # Joint limits
        layout.addWidget(QLabel("Lower Limit (deg):"), 1, 0)
        self.lower_spin = QDoubleSpinBox()
        self.lower_spin.setRange(-360, 360)
        self.lower_spin.setDecimals(2)
        self.lower_spin.setSingleStep(5.0)
        self.lower_spin.setValue(-180.0)
        self.lower_spin.setStyleSheet(SPINBOX_STYLE)
        self.lower_spin.valueChanged.connect(self.changed.emit)
        layout.addWidget(self.lower_spin, 1, 1)

        layout.addWidget(QLabel("Upper Limit (deg):"), 1, 2)
        self.upper_spin = QDoubleSpinBox()
        self.upper_spin.setRange(-360, 360)
        self.upper_spin.setDecimals(2)
        self.upper_spin.setSingleStep(5.0)
        self.upper_spin.setValue(180.0)
        self.upper_spin.setStyleSheet(SPINBOX_STYLE)
        self.upper_spin.valueChanged.connect(self.changed.emit)
        layout.addWidget(self.upper_spin, 1, 3)

        # Gear ratio
        layout.addWidget(QLabel("Gear Ratio:"), 2, 0)
        self.gear_spin = QDoubleSpinBox()
        self.gear_spin.setRange(0.1, 500.0)
        self.gear_spin.setDecimals(2)
        self.gear_spin.setSingleStep(1.0)
        self.gear_spin.setValue(1.0)
        self.gear_spin.setStyleSheet(SPINBOX_STYLE)
        self.gear_spin.valueChanged.connect(self.changed.emit)
        layout.addWidget(self.gear_spin, 2, 1)

        layout.addWidget(QLabel("Home (deg):"), 2, 2)
        self.home_spin = QDoubleSpinBox()
        self.home_spin.setRange(-360, 360)
        self.home_spin.setDecimals(2)
        self.home_spin.setSingleStep(10.0)
        self.home_spin.setValue(0.0)
        self.home_spin.setStyleSheet(SPINBOX_STYLE)
        self.home_spin.valueChanged.connect(self.changed.emit)
        layout.addWidget(self.home_spin, 2, 3)

        self.setStyleSheet("""
            QWidget { background: transparent; }
            QLabel { color: """ + P.DARK_TEXT_DIM + """; font-size: 12px; background: transparent; border: none; }
        """)


class DHParamTableWidget(QWidget):
    """Table-based editor for DH parameters of a single joint."""

    changed = pyqtSignal()

    def __init__(self, joint_name, index, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.index = index

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 4, 12, 4)
        layout.setSpacing(4)

        # Header
        hdr = QHBoxLayout()
        name_label = QLabel(f"#{index+1}  {joint_name}  —  DH Parameters")
        name_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
        hdr.addWidget(name_label)
        hdr.addStretch()
        layout.addLayout(hdr)

        # DH editor grid
        grid = QGridLayout()
        grid.setSpacing(6)

        # Row for each DH parameter
        self.a_edit = QDoubleSpinBox()
        self.a_edit.setRange(-5.0, 5.0)
        self.a_edit.setDecimals(4)
        self.a_edit.setSingleStep(0.01)
        self.a_edit.setValue(0.0)
        self.a_edit.setStyleSheet(SPINBOX_STYLE)
        self.a_edit.valueChanged.connect(self.changed.emit)

        self.alpha_edit = QDoubleSpinBox()
        self.alpha_edit.setRange(-6.2832, 6.2832)
        self.alpha_edit.setDecimals(4)
        self.alpha_edit.setSingleStep(0.01)
        self.alpha_edit.setValue(0.0)
        self.alpha_edit.setStyleSheet(SPINBOX_STYLE)
        self.alpha_edit.valueChanged.connect(self.changed.emit)

        self.d_edit = QDoubleSpinBox()
        self.d_edit.setRange(-5.0, 5.0)
        self.d_edit.setDecimals(4)
        self.d_edit.setSingleStep(0.01)
        self.d_edit.setValue(0.0)
        self.d_edit.setStyleSheet(SPINBOX_STYLE)
        self.d_edit.valueChanged.connect(self.changed.emit)

        self.theta_edit = QDoubleSpinBox()
        self.theta_edit.setRange(-6.2832, 6.2832)
        self.theta_edit.setDecimals(4)
        self.theta_edit.setSingleStep(0.01)
        self.theta_edit.setValue(0.0)
        self.theta_edit.setStyleSheet(SPINBOX_STYLE)
        self.theta_edit.valueChanged.connect(self.changed.emit)

        labels = [
            ("a  (link length)", self.a_edit, "m"),
            ("alpha  (link twist)", self.alpha_edit, "rad"),
            ("d  (link offset)", self.d_edit, "m"),
            ("theta  (joint angle)", self.theta_edit, "rad"),
        ]

        for row, (lbl, spin, unit) in enumerate(labels):
            label_w = QLabel(lbl)
            label_w.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
            label_w.setFixedWidth(170)
            grid.addWidget(label_w, row, 0)

            spin.setFixedWidth(140)
            grid.addWidget(spin, row, 1)

            unit_w = QLabel(unit)
            unit_w.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; background: transparent; border: none;")
            grid.addWidget(unit_w, row, 2)

            # FK preview for this joint
            if row == 0:
                self.fk_label = QLabel("Transform: —")
                self.fk_label.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
                grid.addWidget(self.fk_label, 0, 3, 4, 2)

        layout.addLayout(grid)

    def get_dh(self):
        return {
            "a": self.a_edit.value(),
            "alpha": self.alpha_edit.value(),
            "d": self.d_edit.value(),
            "theta": self.theta_edit.value(),
        }

    def set_dh(self, a=0, alpha=0, d=0, theta=0):
        self.a_edit.blockSignals(True)
        self.alpha_edit.blockSignals(True)
        self.d_edit.blockSignals(True)
        self.theta_edit.blockSignals(True)
        self.a_edit.setValue(a)
        self.alpha_edit.setValue(alpha)
        self.d_edit.setValue(d)
        self.theta_edit.setValue(theta)
        self.a_edit.blockSignals(False)
        self.alpha_edit.blockSignals(False)
        self.d_edit.blockSignals(False)
        self.theta_edit.blockSignals(False)
        self._update_fk_preview()

    def _update_fk_preview(self):
        try:
            T = dh_transform(self.a_edit.value(), self.alpha_edit.value(),
                             self.d_edit.value(), self.theta_edit.value())
            pos = T[:3, 3]
            text = f"Transform: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
            self.fk_label.setText(text)
        except Exception:
            pass


class KinematicConfigPanel(QWidget):
    """Main kinematic configuration panel — URDF loading, DH params, joint config, FK preview.

    Signals:
        robot_loaded: emitted when a robot model is loaded or configuration changes
    """

    robot_loaded = pyqtSignal(object)   # RobotModel instance
    fk_updated = pyqtSignal(list)       # joint angles (degrees)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._robot_model = None
        self._urdf_model = None
        self._dh_arm = None
        self._robodk = None
        try:
            from ..robodk_bridge import RoboDKBridge
            self._robodk = RoboDKBridge()
        except Exception:
            self._robodk = None

        self._joint_widgets = []        # list of JointParamWidget
        self._dh_widgets = []           # list of DHParamTableWidget
        self._current_joint_angles = []

        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # ── Top: Robot info and load controls ──────────────────────
        top_frame = QWidget()
        top_layout = QHBoxLayout(top_frame)
        top_layout.setContentsMargins(4, 4, 4, 4)

        self.robot_name_label = QLabel("No robot loaded")
        self.robot_name_label.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 15px; font-weight: bold; background: transparent; border: none;")
        top_layout.addWidget(self.robot_name_label)

        self.joint_count_label = QLabel("")
        self.joint_count_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; background: transparent; border: none;")
        top_layout.addWidget(self.joint_count_label)

        top_layout.addStretch()

        self.load_btn = QPushButton("Load URDF")
        self.load_btn.setStyleSheet(ACCENT_BUTTON)
        self.load_btn.clicked.connect(self._on_load_urdf)
        top_layout.addWidget(self.load_btn)

        self.save_config_btn = QPushButton("Save Config")
        self.save_config_btn.setStyleSheet(BUTTON_STYLE)
        self.save_config_btn.clicked.connect(self._on_save_config)
        top_layout.addWidget(self.save_config_btn)

        self.load_config_btn = QPushButton("Load Config")
        self.load_config_btn.setStyleSheet(BUTTON_STYLE)
        self.load_config_btn.clicked.connect(self._on_load_config)
        top_layout.addWidget(self.load_config_btn)

        self.reset_btn = QPushButton("Load Default (Astra)")
        self.reset_btn.setStyleSheet(BUTTON_STYLE)
        self.reset_btn.setToolTip("Restore the built-in Astra DH configuration")
        self.reset_btn.clicked.connect(self._on_reset_astra)
        top_layout.addWidget(self.reset_btn)

        main_layout.addWidget(top_frame)

        # ── Splitter: Parameter tabs + FK preview ──────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(3)
        splitter.setStyleSheet(f"QSplitter::handle {{ background: {P.DARK_BORDER}; }}")

        # Left side: Tabbed parameter panels
        left_panel = QTabWidget()
        left_panel.setStyleSheet(f"""
            QTabWidget::pane {{ border: 1px solid {P.DARK_BORDER}; background: {P.DARK_PANEL}; }}
            QTabBar::tab {{
                background: {P.DARK_BUTTON}; color: {P.DARK_TEXT_DIM};
                border: 1px solid {P.DARK_BORDER}; padding: 6px 16px;
                font-size: 12px; font-weight: bold; margin-right: 2px;
            }}
            QTabBar::tab:selected {{ background: {P.DARK_PANEL}; color: {P.DARK_ACCENT}; border-bottom: 2px solid {P.DARK_ACCENT}; }}
            QTabBar::tab:hover:!selected {{ background: {P.DARK_BUTTON_HOVER}; color: {P.DARK_TEXT}; }}
        """)

        # Tab 1: Joint Limits & Gear Ratios
        self.joint_scroll = QScrollArea()
        self.joint_scroll.setWidgetResizable(True)
        self.joint_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        self.joint_container = QWidget()
        self.joint_container.setStyleSheet("background: transparent;")
        self.joint_layout = QVBoxLayout(self.joint_container)
        self.joint_layout.setContentsMargins(4, 4, 4, 4)
        self.joint_layout.setSpacing(4)
        self.joint_layout.addStretch()
        self.joint_scroll.setWidget(self.joint_container)
        left_panel.addTab(self.joint_scroll, "Joint Configuration")

        # Tab 2: DH Parameters
        self.dh_scroll = QScrollArea()
        self.dh_scroll.setWidgetResizable(True)
        self.dh_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        self.dh_container = QWidget()
        self.dh_container.setStyleSheet("background: transparent;")
        self.dh_layout = QVBoxLayout(self.dh_container)
        self.dh_layout.setContentsMargins(4, 4, 4, 4)
        self.dh_layout.setSpacing(4)
        self.dh_layout.addStretch()
        self.dh_scroll.setWidget(self.dh_container)
        left_panel.addTab(self.dh_scroll, "DH Parameters")

        splitter.addWidget(left_panel)

        # Right side: FK Preview + Info
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(8)

        # FK Preview section
        fk_group = QGroupBox("Forward Kinematics Preview")
        fk_group.setStyleSheet(SECTION_STYLE)
        fk_layout = QVBoxLayout(fk_group)
        fk_layout.setSpacing(6)

        self.fk_pos_label = QLabel("EE Position:  (0.000, 0.000, 0.000)  m")
        self.fk_pos_label.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
        fk_layout.addWidget(self.fk_pos_label)

        # Joint angle test inputs
        angle_grid = QGridLayout()
        angle_grid.setSpacing(6)
        self._angle_spins = []
        angle_grid.addWidget(QLabel("Test Angles (deg):"), 0, 0, 1, 8)
        for i in range(6):
            spin = QDoubleSpinBox()
            spin.setRange(-360, 360)
            spin.setDecimals(1)
            spin.setSingleStep(15.0)
            spin.setValue(0.0)
            spin.setFixedWidth(80)
            spin.setStyleSheet(SPINBOX_STYLE)
            spin.valueChanged.connect(self._on_test_angles_changed)
            angle_grid.addWidget(QLabel(f"J{i+1}:"), 0, i*2+1)
            angle_grid.addWidget(spin, 0, i*2+2)
            self._angle_spins.append(spin)
        fk_layout.addLayout(angle_grid)

        self.fk_matrix_label = QLabel("")
        self.fk_matrix_label.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
        fk_layout.addWidget(self.fk_matrix_label)

        # FK action buttons row
        fk_btn_row = QHBoxLayout()
        fk_btn_row.setSpacing(8)

        self.apply_fk_btn = QPushButton("Apply to Simulation")
        self.apply_fk_btn.setStyleSheet(SUCCESS_BUTTON)
        self.apply_fk_btn.clicked.connect(self._on_apply_fk)
        fk_btn_row.addWidget(self.apply_fk_btn)

        self.ik_solve_btn = QPushButton("Solve IK")
        self.ik_solve_btn.setStyleSheet(ACCENT_BUTTON)
        self.ik_solve_btn.clicked.connect(self._on_solve_ik)
        fk_btn_row.addWidget(self.ik_solve_btn)

        if PORTED_IK_AVAILABLE:
            self.verify_fk_btn = QPushButton("Verify FK")
            self.verify_fk_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=10, padding="2px 10px"))
            self.verify_fk_btn.setToolTip("Compare studio FK vs ported repo FK at current angles")
            self.verify_fk_btn.clicked.connect(self._on_verify_fk)
            fk_btn_row.addWidget(self.verify_fk_btn)

        # IK target group
        ik_group = QGroupBox("Inverse Kinematics")
        ik_group.setStyleSheet(SECTION_STYLE)
        ik_layout = QGridLayout(ik_group)
        ik_layout.setSpacing(4)

        ik_layout.addWidget(QLabel("Target X:"), 0, 0)
        self.ik_x_spin = QDoubleSpinBox()
        self.ik_x_spin.setRange(-2.0, 2.0)
        self.ik_x_spin.setDecimals(3)
        self.ik_x_spin.setSingleStep(0.05)
        self.ik_x_spin.setValue(0.4)
        self.ik_x_spin.setStyleSheet(SPINBOX_STYLE)
        self.ik_x_spin.setFixedWidth(90)
        ik_layout.addWidget(self.ik_x_spin, 0, 1)

        ik_layout.addWidget(QLabel("Target Y:"), 0, 2)
        self.ik_y_spin = QDoubleSpinBox()
        self.ik_y_spin.setRange(-2.0, 2.0)
        self.ik_y_spin.setDecimals(3)
        self.ik_y_spin.setSingleStep(0.05)
        self.ik_y_spin.setValue(0.0)
        self.ik_y_spin.setStyleSheet(SPINBOX_STYLE)
        self.ik_y_spin.setFixedWidth(90)
        ik_layout.addWidget(self.ik_y_spin, 0, 3)

        ik_layout.addWidget(QLabel("Target Z:"), 0, 4)
        self.ik_z_spin = QDoubleSpinBox()
        self.ik_z_spin.setRange(-2.0, 2.0)
        self.ik_z_spin.setDecimals(3)
        self.ik_z_spin.setSingleStep(0.05)
        self.ik_z_spin.setValue(0.5)
        self.ik_z_spin.setStyleSheet(SPINBOX_STYLE)
        self.ik_z_spin.setFixedWidth(90)
        ik_layout.addWidget(self.ik_z_spin, 0, 5)

        # Orientation targets (degrees) — optional, full-pose IK when set
        ik_layout.addWidget(QLabel("Rot X°:"), 1, 0)
        self.ik_rx_spin = QDoubleSpinBox()
        self.ik_rx_spin.setRange(-180.0, 180.0)
        self.ik_rx_spin.setDecimals(1)
        self.ik_rx_spin.setValue(0.0)
        self.ik_rx_spin.setStyleSheet(SPINBOX_STYLE)
        self.ik_rx_spin.setFixedWidth(90)
        ik_layout.addWidget(self.ik_rx_spin, 1, 1)

        ik_layout.addWidget(QLabel("Rot Y°:"), 1, 2)
        self.ik_ry_spin = QDoubleSpinBox()
        self.ik_ry_spin.setRange(-180.0, 180.0)
        self.ik_ry_spin.setDecimals(1)
        self.ik_ry_spin.setValue(0.0)
        self.ik_ry_spin.setStyleSheet(SPINBOX_STYLE)
        self.ik_ry_spin.setFixedWidth(90)
        ik_layout.addWidget(self.ik_ry_spin, 1, 3)

        ik_layout.addWidget(QLabel("Rot Z°:"), 1, 4)
        self.ik_rz_spin = QDoubleSpinBox()
        self.ik_rz_spin.setRange(-180.0, 180.0)
        self.ik_rz_spin.setDecimals(1)
        self.ik_rz_spin.setValue(0.0)
        self.ik_rz_spin.setStyleSheet(SPINBOX_STYLE)
        self.ik_rz_spin.setFixedWidth(90)
        ik_layout.addWidget(self.ik_rz_spin, 1, 5)

        # IK solver selector: Built-in LM | RoboDK | Ported repo IK
        ik_tools_row = QHBoxLayout()
        ik_tools_row.addWidget(QLabel("Solver:"))
        self.ik_solver_combo = QComboBox()
        self.ik_solver_combo.addItem("Built-in (LM)")
        if PORTED_IK_AVAILABLE:
            self.ik_solver_combo.addItem("Ported (repo IK)")
        if AR4_IK_AVAILABLE:
            self.ik_solver_combo.addItem("AR4 analytic")
        self.ik_solver_combo.addItem("RoboDK")
        self.ik_solver_combo.setStyleSheet(COMBO_STYLE)
        self.ik_solver_combo.setFixedHeight(26)
        ik_tools_row.addWidget(self.ik_solver_combo)
        self.ik_send_robodk_btn = QPushButton("Send joints → RoboDK")
        self.ik_send_robodk_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=10, padding="2px 10px"))
        self.ik_send_robodk_btn.setFixedHeight(26)
        self.ik_send_robodk_btn.clicked.connect(self._on_send_robodk)
        ik_tools_row.addWidget(self.ik_send_robodk_btn)
        ik_tools_row.addStretch()
        ik_layout.addLayout(ik_tools_row, 2, 0, 1, 6)

        self.ik_result_label = QLabel("No IK solution computed yet")
        self.ik_result_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
        ik_layout.addWidget(self.ik_result_label, 3, 0, 1, 6)

        fk_layout.addLayout(fk_btn_row)
        fk_layout.addWidget(ik_group)

        right_layout.addWidget(fk_group)

        # URDF Model Info
        info_group = QGroupBox("URDF Model Information")
        info_group.setStyleSheet(SECTION_STYLE)
        info_layout = QVBoxLayout(info_group)
        info_layout.setSpacing(4)

        self.info_text = QLabel("Load a URDF file to see model information.")
        self.info_text.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; background: transparent; border: none;")
        self.info_text.setWordWrap(True)
        info_layout.addWidget(self.info_text)

        right_layout.addWidget(info_group)
        right_layout.addStretch()

        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)

        main_layout.addWidget(splitter, 1)

    # ── Public API ─────────────────────────────────────────────────

    def get_robot_model(self):
        return self._robot_model

    def get_dh_arm(self):
        return self._dh_arm

    def load_robot_from_urdf(self, urdf_path):
        """Load a robot model from a URDF file (public API)."""
        self._load_urdf(urdf_path)

    def set_dh_from_external(self, dh_arm):
        """Set DH arm from an external source (e.g. loaded config)."""
        self._dh_arm = dh_arm
        self._rebuild_ui_from_dh()

    def get_current_joint_angles(self):
        """Get the current joint angles from the test angle spins (degrees)."""
        return [s.value() for s in self._angle_spins]

    # ── Internal Methods ─────────────────────────────────────────────

    def _on_load_urdf(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Robot URDF", "", "URDF Files (*.urdf *.URDF);;All Files (*.*)"
        )
        if path:
            self._load_urdf(path)

    def _load_urdf(self, path):
        try:
            self._urdf_model = URDFModel.load(path)
            self._dh_arm = DHArm.from_urdf(path)

            # Create RobotModel and wire it up
            self._robot_model = RobotModel(self._urdf_model.name)
            self._robot_model.urdf_path = path
            self._robot_model.dh_arm = self._dh_arm
            self._robot_model.num_joints = self._dh_arm.num_joints
            self._robot_model.joint_limits = {}
            for name, jdef in self._urdf_model.joints.items():
                if jdef.is_movable:
                    lo, hi = math.degrees(jdef.lower_limit), math.degrees(jdef.upper_limit)
                    self._robot_model.joint_limits[name] = (lo, hi)

            # Update display
            self.robot_name_label.setText(self._urdf_model.name)
            movable = self._urdf_model.num_movable_joints
            total = self._urdf_model.num_joints
            self.joint_count_label.setText(f"{movable} movable / {total} total joints")

            # Update info text
            info_lines = [
                f"File: {os.path.basename(path)}",
                f"Path: {path}",
                f"Total joints: {total}",
                f"Movable joints: {movable}",
                f"Total links: {len(self._urdf_model.links)}",
                "",
                "Joints:",
            ]
            for jname in self._urdf_model._joint_order:
                j = self._urdf_model.joints[jname]
                if j.is_movable:
                    lo, hi = math.degrees(j.lower_limit), math.degrees(j.upper_limit)
                    info_lines.append(f"  {jname}: {j.type}, limits=[{lo:.1f}, {hi:.1f}] deg")
                else:
                    info_lines.append(f"  {jname}: {j.type}")

            self.info_text.setText("\n".join(info_lines))

            # Rebuild parameter editors
            self._rebuild_ui_from_dh()

            # Emit signal (always emit now that _robot_model is set)
            self.robot_loaded.emit(self._robot_model)

        except Exception as e:
            QMessageBox.critical(self, "URDF Load Error",
                                 f"Failed to load URDF:\n{str(e)}")

    def _rebuild_ui_from_dh(self):
        """Rebuild the joint parameter and DH parameter editors from the current DH arm."""
        # Clear existing widgets
        self._clear_layout(self.joint_layout)
        self._clear_layout(self.dh_layout)

        self._joint_widgets = []
        self._dh_widgets = []

        if self._dh_arm is None:
            self.joint_layout.addStretch()
            self.dh_layout.addStretch()
            return

        for i, dh in enumerate(self._dh_arm.dh_params):
            name = (self._dh_arm.joint_names[i]
                    if i < len(self._dh_arm.joint_names)
                    else f"Joint {i+1}")

            # Joint config widget (limits, gear, home)
            jw = JointParamWidget(name, i)
            limits = self._dh_arm.joint_limits.get(name, (-math.pi, math.pi))
            jw.lower_spin.setValue(math.degrees(limits[0]))
            jw.upper_spin.setValue(math.degrees(limits[1]))
            gear = self._dh_arm.gear_ratios.get(name, 1.0)
            jw.gear_spin.setValue(gear)
            home = self._dh_arm.home_position.get(name, 0.0)
            jw.home_spin.setValue(math.degrees(home))
            jw.changed.connect(self._on_param_changed)
            self._joint_widgets.append(jw)
            self.joint_layout.addWidget(jw)

            # DH parameter widget
            dw = DHParamTableWidget(name, i)
            dw.set_dh(a=dh.a, alpha=dh.alpha, d=dh.d, theta=dh.theta)
            dw.changed.connect(self._on_dh_changed)
            self._dh_widgets.append(dw)
            self.dh_layout.addWidget(dw)

            # Separator
            if i < len(self._dh_arm.dh_params) - 1:
                sep = QFrame()
                sep.setFrameShape(QFrame.HLine)
                sep.setStyleSheet(f"background: {P.DARK_BORDER}; border: none; max-height: 1px;")
                self.joint_layout.addWidget(sep)
                self.dh_layout.addWidget(sep)

        self.joint_layout.addStretch()
        self.dh_layout.addStretch()

        # Update FK preview
        self._update_fk()

    def _clear_layout(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
            else:
                # It's a stretch/layout item
                pass

    def _on_param_changed(self):
        """Called when a joint parameter (limits, gear, home) changes."""
        if self._dh_arm is None:
            return
        for jw in self._joint_widgets:
            name = jw.joint_name
            lower = math.radians(jw.lower_spin.value())
            upper = math.radians(jw.upper_spin.value())
            self._dh_arm.joint_limits[name] = (lower, upper)
            self._dh_arm.gear_ratios[name] = jw.gear_spin.value()
            self._dh_arm.home_position[name] = math.radians(jw.home_spin.value())

    def _on_dh_changed(self):
        """Called when DH parameter values change."""
        if self._dh_arm is None:
            return
        for i, dw in enumerate(self._dh_widgets):
            if i < len(self._dh_arm.dh_params):
                dh = self._dh_arm.dh_params[i]
                vals = dw.get_dh()
                dh.a = vals["a"]
                dh.alpha = vals["alpha"]
                dh.d = vals["d"]
                dh.theta = vals["theta"]
        self._update_fk()

    def _on_test_angles_changed(self):
        """Called when any test angle spinbox changes."""
        self._update_fk()

    def _update_fk(self):
        """Update the FK preview with current test angles."""
        if self._dh_arm is None:
            return

        angles = [s.value() for s in self._angle_spins[:self._dh_arm.num_joints]]
        angles_rad = [math.radians(a) for a in angles]

        self._dh_arm.set_thetas(angles_rad)
        T = self._dh_arm.forward()

        pos = T[:3, 3]
        self.fk_pos_label.setText(f"EE Position:  ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})  m")

        # Show matrix
        mat_lines = []
        for r in range(4):
            row_str = "  ".join(f"{T[r, c]:8.4f}" for c in range(4))
            mat_lines.append(row_str)
        self.fk_matrix_label.setText("\n".join(mat_lines))

    def _on_apply_fk(self):
        """Apply current FK joint angles to the simulation."""
        if self._dh_arm is None:
            return
        angles = [s.value() for s in self._angle_spins[:self._dh_arm.num_joints]]
        self.fk_updated.emit(angles)

    def _on_solve_ik(self):
        """Solve inverse kinematics for the target Cartesian pose."""
        if self._dh_arm is None:
            self.ik_result_label.setText("Load a robot model first")
            self.ik_result_label.setStyleSheet(f"color: {P.DARK_WARNING}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
            return
        target = [
            self.ik_x_spin.value(),
            self.ik_y_spin.value(),
            self.ik_z_spin.value(),
        ]
        # Optional orientation (full-pose IK) — only when any is non-zero.
        rx = math.radians(self.ik_rx_spin.value())
        ry = math.radians(self.ik_ry_spin.value())
        rz = math.radians(self.ik_rz_spin.value())
        target_orient = None
        if abs(rx) > 1e-6 or abs(ry) > 1e-6 or abs(rz) > 1e-6:
            cx, sx = math.cos(rx), math.sin(rx)
            cy, sy = math.cos(ry), math.sin(ry)
            cz, sz = math.cos(rz), math.sin(rz)
            Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            target_orient = Rz @ Ry @ Rx

        try:
            current = [s.value() for s in self._angle_spins[:self._dh_arm.num_joints]]
            current_rad = [math.radians(a) for a in current]

            # Solver selection from the dropdown.
            result = None
            solver_name = "Built-in"
            use_rdk = self.ik_solver_combo.currentText() == "RoboDK"
            use_ported = self.ik_solver_combo.currentText() == "Ported (repo IK)"
            use_ar4 = self.ik_solver_combo.currentText() == "AR4 analytic"

            if use_ar4 and AR4_IK_AVAILABLE:
                # AR4-MK3 closed-form analytic IK (ported from Annin Robotics'
                # kinematics.cpp). Input: xyzuvw (mm, deg rotation vector) +
                # joint estimate (deg). Returns deg. Works in the AR4's DHM
                # model — best when the loaded robot IS the AR4 geometry.
                try:
                    deg_target = [math.degrees(a) for a in current]
                    seed = deg_target if len(deg_target) >= 6 else \
                        (deg_target + [0.0] * (6 - len(deg_target)))[:6]
                    # Convert the target orientation (3x3) to a rotation
                    # vector (u,v,w in deg) for the AR4 solver.
                    R = np.eye(3) if target_orient is None else target_orient
                    cos_a = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
                    ang = math.acos(cos_a)
                    if ang < 1e-6:
                        rv = [0.0, 0.0, 0.0]
                    else:
                        rv = [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                              R[1, 0] - R[0, 1]]
                        rv = [v * ang / (2.0 * math.sin(ang)) for v in rv]
                        rv = [math.degrees(v) for v in rv]
                    ar4_deg = _ar4_ik([target[0], target[1], target[2],
                                       rv[0], rv[1], rv[2]], seed)
                    if ar4_deg is not None:
                        # VERIFY the analytic result actually reproduces the
                        # target pose. The AR4 solver assumes a specific
                        # reachable workspace; outside it, the returned
                        # solution can be wrong. Only accept a verified hit.
                        try:
                            chk = _ar4_fk([float(a) for a in ar4_deg])
                            pos_err = math.sqrt(
                                (chk[0] - target[0]) ** 2 +
                                (chk[1] - target[1]) ** 2 +
                                (chk[2] - target[2]) ** 2)
                            if pos_err < 1.0:   # within 1 mm
                                result = [math.radians(a) for a in ar4_deg]
                                solver_name = "AR4"
                        except Exception:
                            result = None
                except Exception:
                    result = None

            if use_ported and PORTED_IK_AVAILABLE:
                # Ported solver: numerical IK using LM with random restarts.
                # When the studio has a URDF-loaded DHArm, create a custom FK
                # wrapper so the solver matches THIS arm, not the hardcoded
                # repo model.
                try:
                    deg_target = [math.degrees(a) for a in current]
                    limits = None
                    if self._dh_arm.joint_limits:
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
                    seed = deg_target if len(deg_target) >= 6 else \
                        (deg_target + [0.0] * (6 - len(deg_target)))[:6]

                    # Build a custom FK that uses the studio's DHArm so the
                    # solver matches the loaded URDF, not the repo's hardcoded
                    # DH table.
                    def _studio_fk(q_deg):
                        """FK wrapper: degrees in -> (X, Y, Z, R(3,3))."""
                        q_rad = [math.radians(a) for a in q_deg]
                        self._dh_arm.set_thetas(q_rad)
                        T = self._dh_arm.forward()
                        return T[0, 3], T[1, 3], T[2, 3], T[:3, :3]

                    fk_to_use = _studio_fk
                    ported_deg = _ported_ik_raw(target, target_orient,
                                               seed=seed, joint_limits=limits,
                                               retries=4, fk_func=fk_to_use)
                    if ported_deg is not None:
                        result = [math.radians(a) for a in ported_deg]
                        solver_name = "Ported"
                except Exception:
                    # fall through to built-in
                    result = None

            if result is None and use_rdk and self._robodk is not None:
                # The bridge returns degrees; convert to radians so the
                # downstream math.degrees() path works uniformly.
                pose = np.eye(4)
                pose[:3, 3] = target
                if target_orient is not None:
                    pose[:3, :3] = target_orient
                rdk_deg = self._robodk.ik(pose, initial_guess=current)
                if rdk_deg is not None:
                    result = [math.radians(a) for a in rdk_deg]
                    solver_name = "RoboDK"

            if result is None:
                result = self._dh_arm.compute_ik(target, target_orient,
                                                 joint_angles=current_rad)
            if result is not None:
                result_deg = [math.degrees(a) for a in result]
                # Update test angle spins
                for i in range(min(len(result_deg), len(self._angle_spins))):
                    self._angle_spins[i].blockSignals(True)
                    self._angle_spins[i].setValue(round(result_deg[i], 1))
                    self._angle_spins[i].blockSignals(False)
                self._update_fk()
                result_str = ", ".join(f"{a:.1f}" for a in result_deg)
                self.ik_result_label.setText(f"IK solved ({solver_name}) — joints: [{result_str}]°")
                self.ik_result_label.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
                # Also emit FK update to sync jog panel
                self.fk_updated.emit(result_deg)
            else:
                self.ik_result_label.setText("No IK solution found — target may be out of reach")
                self.ik_result_label.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
        except Exception as e:
            self.ik_result_label.setText(f"IK calculation failed: {e}")
            self.ik_result_label.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; font-family: monospace; background: transparent; border: none;")

    def _on_verify_fk(self):
        """Compare studio FK vs ported repo FK at the current test angles."""
        if self._dh_arm is None or not PORTED_IK_AVAILABLE:
            return
        angles = [s.value() for s in self._angle_spins[:self._dh_arm.num_joints]]
        angles_rad = [math.radians(a) for a in angles]

        # Studio FK
        self._dh_arm.set_thetas(angles_rad)
        T_studio = self._dh_arm.forward()
        sx, sy, sz = T_studio[0, 3], T_studio[1, 3], T_studio[2, 3]

        # Ported FK (hardcoded repo DH)
        deg_padded = (angles + [0.0] * 6)[:6]
        px, py, pz, _ = _repo_fk(deg_padded)

        err = math.sqrt((sx - px)**2 + (sy - py)**2 + (sz - pz)**2)
        msg = (f"Studio FK: ({sx:.4f}, {sy:.4f}, {sz:.4f})  "
               f"Ported FK: ({px:.4f}, {py:.4f}, {pz:.4f})  "
               f"Error: {err:.6f} m")
        color = P.DARK_SUCCESS if err < 1e-4 else P.DARK_WARNING
        self.ik_result_label.setText(msg)
        self.ik_result_label.setStyleSheet(
            f"color: {color}; font-size: 12px; font-family: monospace; "
            f"background: transparent; border: none;")

    def _on_send_robodk(self):
        """Send current joint angles to RoboDK for display."""
        if self._dh_arm is None:
            return
        if not self._robodk:
            self.ik_result_label.setText("RoboDK bridge unavailable")
            self.ik_result_label.setStyleSheet(f"color: {P.DARK_WARNING}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
            return
        if not self._robodk.connected and not self._robodk.ping():
            self.ik_result_label.setText("RoboDK not connected — start RoboDK first")
            self.ik_result_label.setStyleSheet(f"color: {P.DARK_WARNING}; font-size: 12px; font-family: monospace; background: transparent; border: none;")
            return
        current = [s.value() for s in self._angle_spins[:self._dh_arm.num_joints]]
        self._robodk.move_joints(current)
        self.ik_result_label.setText("Sent joints to RoboDK")
        self.ik_result_label.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; font-family: monospace; background: transparent; border: none;")

    def _on_save_config(self):
        """Save DH configuration to JSON."""
        if self._dh_arm is None:
            QMessageBox.warning(self, "No Configuration",
                                "Load a robot model first.")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "Save Robot Configuration", "",
            "Robot Config (*.robot.json);;JSON Files (*.json);;All Files (*.*)"
        )
        if path:
            self._dh_arm.save(path)

    def _on_load_config(self):
        """Load DH configuration from JSON."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Robot Configuration", "",
            "Robot Config (*.robot.json *.json);;All Files (*.*)"
        )
        if path:
            try:
                self._dh_arm = DHArm.load(path)
                self.robot_name_label.setText(self._dh_arm.name)
                self.joint_count_label.setText(f"{self._dh_arm.num_joints} joints")
                self._rebuild_ui_from_dh()
            except Exception as e:
                QMessageBox.critical(self, "Config Load Error",
                                     f"Failed to load configuration:\n{str(e)}")

    def _on_reset_astra(self):
        """Reset to the Astra 6-DOF default configuration."""
        from ..core.kinematics import create_astra_dh
        self._dh_arm = create_astra_dh()
        self.robot_name_label.setText("Astra 6-DOF")
        self.joint_count_label.setText("6 joints")
        self.info_text.setText("Default Astra 6-DOF arm configuration loaded.")
        self._rebuild_ui_from_dh()
