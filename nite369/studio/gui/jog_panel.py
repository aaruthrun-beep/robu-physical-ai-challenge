"""Professional joint control panel with dynamic robot model integration.

Features:
  - Dynamic joint names and limits from loaded robot model (URDF/DH arm)
  - 6 joint sliders with real-time simulation sync
  - Incremental jog buttons (+ / -) for precise step control
  - Joint position readout from simulation/encoder feedback
  - Jog speed slider with percentage indicator
  - Gripper control with open/close buttons
  - Feed rate input
  - Direct Control toggle for real robot movement
  - Cartesian jog mode (tool-frame)
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSlider, QDoubleSpinBox, QCheckBox, QSpinBox, QFrame, QComboBox,
    QGroupBox, QGridLayout, QScrollArea, QTabWidget,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont
import math
import numpy as np

from . import palette as P


JOG_STEP_SIZES = [0.1, 0.5, 1.0, 5.0, 10.0]  # degrees


class JogButton(QPushButton):
    """Touch-friendly jog button with hold-to-jog (press & hold = continuous).

    Emits ``pressed`` for a hold (continuous motion) and ``single_step`` for a
    short tap. Distinguishes tap vs hold with a QTimer: if released before the
    hold threshold, it's a tap (one step); otherwise continuous jog started on
    press keeps running until release.
    """

    # (signal payloads are set by the parent via .step_callback / .hold_callback)
    HOLD_MS = 250

    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
        self._pressed = False
        self._hold_timer = QTimer(self)
        self._hold_timer.setSingleShot(True)
        self._hold_timer.setInterval(self.HOLD_MS)
        self._hold_timer.timeout.connect(self._on_hold)
        self._held = False
        self.step_callback = None   # callable() for a tap
        self.hold_callback = None   # callable(start=True/False) for hold
        self.setMinimumHeight(48)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, ev):
        self._pressed = True
        self._held = False
        self._hold_timer.start()
        super().mousePressEvent(ev)

    def mouseReleaseEvent(self, ev):
        was_pressed = self._pressed
        self._pressed = False
        self._hold_timer.stop()
        if was_pressed:
            if self._held:
                # Continuous jog ends on release.
                if self.hold_callback:
                    self.hold_callback(False)
            else:
                # Short tap = single step.
                if self.step_callback:
                    self.step_callback()
        super().mouseReleaseEvent(ev)

    def mouseMoveEvent(self, ev):
        # Cancel if the finger/pointer drags off the button before release.
        if self._pressed and not self.rect().contains(ev.pos()):
            self._pressed = False
            self._hold_timer.stop()
            if self._held and self.hold_callback:
                self.hold_callback(False)
            self._held = False
        super().mouseMoveEvent(ev)

    def _on_hold(self):
        if self._pressed:
            self._held = True
            if self.hold_callback:
                self.hold_callback(True)


class StepChip(QPushButton):
    """Toggle-able step-size chip with an active highlighted state."""

    def __init__(self, label, value, unit, parent=None):
        super().__init__(label, parent)
        self.value = value
        self.unit = unit
        self.setCheckable(True)
        self.setMinimumHeight(40)
        self.setCursor(Qt.PointingHandCursor)


def chip_style():
    """QSS for a step-size chip (highlighted when checked)."""
    return (
        f"QPushButton {{ background: {P.DARK_BUTTON}; color: {P.DARK_TEXT};"
        f" border: 1px solid {P.DARK_BORDER}; border-radius: 6px;"
        f" padding: 4px 10px; font-size: 13px; }}"
        f"QPushButton:hover {{ background: {P.DARK_BUTTON_HOVER}; }}"
        f"QPushButton:checked {{ background: {P.DARK_ACCENT}; color: #1a1a16;"
        f" border-color: {P.DARK_ACCENT}; font-weight: bold; }}"
    )


def jog_btn_style(bg=None, text=None):
    """Touch-friendly jog button style (large, high-contrast)."""
    bg = bg or P.DARK_BUTTON
    text = text or P.DARK_TEXT
    return (
        f"QPushButton {{ background: {bg}; color: {text};"
        f" border: 1px solid {P.DARK_BORDER}; border-radius: 8px;"
        f" font-size: 15px; font-weight: bold; }}"
        f"QPushButton:hover {{ background: {P.DARK_BUTTON_HOVER}; }}"
        f"QPushButton:pressed {{ background: {P.DARK_ACCENT}; color: #1a1a16; }}"
    )


class JointRow(QWidget):
    """Single joint row: name, sim slider + spinbox, 5-button jog pad,
    and a direction-inversion toggle (sends #CFG di bit to the robot)."""

    # Emits (joint_index, delta_deg) when a jog pad button is pressed.
    jog_pad_clicked = pyqtSignal(int, float)   # joint_idx, delta_deg
    invert_toggled = pyqtSignal(int, bool)     # joint_idx, invert

    def __init__(self, joint_name, joint_index, limits=(-180, 180), parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.joint_index = joint_index
        self._limits = limits
        self._value = 0.0
        self._encoder_value = None
        self._step_size = 1.0

        self.setStyleSheet(
            f"QWidget {{ border: 1px solid {P.DARK_BORDER}; background: {P.DARK_PANEL}; border-radius: 4px; margin: 1px; }}"
        )
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 6)
        layout.setSpacing(4)

        # Row 1: name | position | spinbox | invert toggle
        top = QHBoxLayout()
        top.setSpacing(6)

        short = self.joint_name.replace("joint", "J").replace("_", "").upper()[:3]
        self.name_lbl = QLabel(short)
        self.name_lbl.setFixedWidth(26)
        self.name_lbl.setAlignment(Qt.AlignCenter)
        self.name_lbl.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        self.name_lbl.setToolTip(self.joint_name)
        top.addWidget(self.name_lbl)

        self.pos_readout = QLabel("0.0°")
        self.pos_readout.setFixedWidth(52)
        self.pos_readout.setAlignment(Qt.AlignCenter)
        self.pos_readout.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
        self.pos_readout.setToolTip("Current simulation position")
        top.addWidget(self.pos_readout)

        top.addStretch()

        self.spinbox = QDoubleSpinBox()
        lo, hi = self._limits
        self.spinbox.setRange(lo, hi)
        self.spinbox.setDecimals(2)
        self.spinbox.setFixedWidth(84)
        self.spinbox.setStyleSheet(P.input_style(font_size=12))
        self.spinbox.setToolTip("Simulation joint angle (does NOT move the robot)")
        top.addWidget(self.spinbox)
        deg_lbl = QLabel("°")
        deg_lbl.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        top.addWidget(deg_lbl)

        self.invert_check = QCheckBox("Invert")
        self.invert_check.setStyleSheet(
            f"color: {P.DARK_TEXT_DIM}; font-size: 11px; font-weight: bold; spacing: 4px; background: transparent; border: none;")
        self.invert_check.setToolTip("Direction inversion bit (sent to the robot)")
        self.invert_check.toggled.connect(
            lambda checked: self.invert_toggled.emit(self.joint_index, checked))
        top.addWidget(self.invert_check)

        layout.addLayout(top)

        # Row 2: simulation-only slider
        self.slider = QSlider(Qt.Horizontal)
        lo, hi = self._limits
        self.slider.setRange(int(lo * 10), int(hi * 10))
        self.slider.setValue(0)
        self.slider.setStyleSheet(P.slider_style())
        self.slider.setToolTip("Simulation only — does NOT move the robot")
        layout.addWidget(self.slider)

        # Row 3: − / + jog pad (uses the global step size; hold = continuous)
        pad = QHBoxLayout()
        pad.setSpacing(6)
        self.pad_btns = []
        for label, d in (("-", -1), ("+", 1)):
            btn = JogButton(label)
            btn.setStyleSheet(jog_btn_style())
            btn.setMinimumHeight(52)
            btn.setMinimumWidth(64)
            btn.setToolTip(f"Jog {d:+d} step (hold = continuous)")
            btn.step_callback = lambda dd=d: self.jog_pad_clicked.emit(self.joint_index, dd * self._step_size)
            btn.hold_callback = lambda start, dd=d: self.hold_triggered(self.joint_index, dd * self._step_size, start)
            pad.addWidget(btn)
            self.pad_btns.append(btn)
        # Show the current step for this row next to the pad.
        self._pad_step_lbl = QLabel(f"{self._step_size:g}°")
        self._pad_step_lbl.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        pad.addWidget(self._pad_step_lbl)
        pad.addStretch()
        layout.addLayout(pad)

    # Overridden by the panel: (joint_idx, delta_deg, started) on a hold.
    def hold_triggered(self, joint_idx, delta, started):
        pass

    def set_limits(self, lo, hi):
        self._limits = (lo, hi)
        self.limits = (lo, hi)  # public alias for soft-limit clamping
        self.slider.blockSignals(True)
        self.slider.setRange(int(lo * 10), int(hi * 10))
        self.slider.blockSignals(False)
        self.spinbox.blockSignals(True)
        self.spinbox.setRange(lo, hi)
        self.spinbox.blockSignals(False)

    def set_value(self, deg):
        self._value = deg
        self.pos_readout.setText(f"{deg:.1f}°")
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(deg)
        self.spinbox.blockSignals(False)
        self.slider.blockSignals(True)
        self.slider.setValue(int(deg * 10))
        self.slider.blockSignals(False)

    def set_step_size(self, step):
        self._step_size = step
        if hasattr(self, "_pad_step_lbl"):
            self._pad_step_lbl.setText(f"{step:g}°")

    def set_invert(self, on):
        self.invert_check.blockSignals(True)
        self.invert_check.setChecked(on)
        self.invert_check.blockSignals(False)

    def set_encoder_feedback(self, deg):
        """Show encoder feedback next to position readout."""
        self._encoder_value = deg
        diff = deg - self._value
        if abs(diff) < 0.3:
            color = P.DARK_SUCCESS
        elif abs(diff) < 1.0:
            color = P.DARK_WARNING
        else:
            color = P.DARK_ERROR
        self.pos_readout.setStyleSheet(
            f"color: {color}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;"
        )

    @property
    def value(self):
        return self._value


class JointControlPanel(QWidget):
    """Professional joint control panel with dynamic robot model integration."""

    joint_moved = pyqtSignal(list)
    gripper_changed = pyqtSignal(float)
    move_requested = pyqtSignal(list)
    home_requested = pyqtSignal()
    zero_requested = pyqtSignal()
    world_jog_requested = pyqtSignal(float, float, float)  # dx, dy, dz (mm)

    def __init__(self, num_joints=6, parent=None):
        super().__init__(parent)
        self.num_joints = num_joints
        self._rows = []
        self._default_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        self._default_limits = [(-180, 180), (-90, 90), (-135, 135),
                                (-180, 180), (-120, 120), (-180, 180)]
        self.values = [0.0] * num_joints
        self.gripper_value = 100.0
        self.sim = None
        self.connection_manager = None
        self._dh_arm = None  # for world-frame (Cartesian) jog IK
        self._active_hold = None
        self._step_size_index = 1  # default 0.5°
        self._setup_ui()

    def set_simulation(self, sim):
        self.sim = sim

    def set_connection_manager(self, cm):
        self.connection_manager = cm

    def configure_from_dh_arm(self, dh_arm):
        """Dynamically reconfigure joint names and limits from a DHArm."""
        if dh_arm is None:
            return
        self._dh_arm = dh_arm
        n = min(dh_arm.num_joints, 6)
        names = dh_arm.joint_names[:n] if dh_arm.joint_names else self._default_names[:n]

        limits = []
        for name in names:
            if name in dh_arm.joint_limits:
                lo, hi = dh_arm.joint_limits[name]
                # joint_limits are in RADIANS — convert to degrees for the UI.
                limits.append((round(math.degrees(lo), 1),
                               round(math.degrees(hi), 1)))
            else:
                limits.append((-180.0, 180.0))

        self._update_joint_config(names, limits)
        self._world_frame_group.setEnabled(dh_arm is not None)

    def _update_joint_config(self, names, limits):
        """Update the UI for new joint names and limits."""
        # Update existing rows
        for i in range(min(len(names), len(self._rows))):
            row = self._rows[i]
            short = names[i].replace("joint", "J").replace("_", "").upper()[:3]
            row.name_lbl.setText(short)
            row.name_lbl.setToolTip(names[i])
            row.set_limits(limits[i][0], limits[i][1])
            row.joint_name = names[i]

        # Hide extra rows
        for i in range(len(names), len(self._rows)):
            self._rows[i].setVisible(False)

    def _setup_ui(self):
        # Wrap everything in a scroll area so the panel never overlaps when
        # the window is too short (6 joint rows + world jog + gripper).
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._scroll.setStyleSheet(
            f"QScrollArea {{ background: transparent; border: none; }}"
            f"QScrollArea > QWidget > QWidget {{ background: {P.DARK_BG}; }}")
        outer.addWidget(self._scroll)

        content = QWidget()
        self._scroll.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # ── Header (always visible: readout + Home + Stop + speed) ──
        header = QHBoxLayout()
        title = QLabel("Joint Control")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)

        # Live position readout (updated by set_joints / world jog).
        self._pos_readout = QLabel("J: —")
        self._pos_readout.setStyleSheet(
            f"color: {P.DARK_ACCENT}; font-size: 13px; font-weight: bold; background: {P.DARK_INPUT};"
            f" border: 1px solid {P.DARK_BORDER}; border-radius: 4px; padding: 2px 8px;")
        self._pos_readout.setMinimumWidth(180)
        header.addWidget(self._pos_readout)
        header.addStretch()

        # Home (green) + Stop (red, emergency) — always visible.
        self._home_btn = QPushButton("HOME")
        self._home_btn.setFixedHeight(44)
        self._home_btn.setFixedWidth(84)
        self._home_btn.setStyleSheet(P.success_btn_style(font_size=14, padding="0px"))
        self._home_btn.clicked.connect(self._on_zero)
        header.addWidget(self._home_btn)

        self._stop_btn = QPushButton("STOP")
        self._stop_btn.setFixedHeight(44)
        self._stop_btn.setFixedWidth(84)
        self._stop_btn.setStyleSheet(P.btn_style(P.DARK_ERROR, text="#ffffff",
                                                 font_size=14, padding="0px"))
        self._stop_btn.clicked.connect(self._on_stop)
        header.addWidget(self._stop_btn)

        layout.addLayout(header)

        # Step size selector
        step_label = QLabel("Step:")
        step_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; border: none; background: transparent;")
        header.addWidget(step_label)
        self.step_combo = QComboBox()
        self.step_combo.addItems([f"{s}°" for s in [0.1, 0.5, 1.0, 5.0, 10.0]])
        self.step_combo.setCurrentIndex(1)  # default 0.5°
        self.step_combo.setFixedWidth(64)
        self.step_combo.setStyleSheet(P.input_style(font_size=11))
        self.step_combo.currentIndexChanged.connect(self._on_step_changed)
        header.addWidget(self.step_combo)

        layout.addLayout(header)

        # ── Sub-tabs: Joint Jog / World Jog ───────────────────────
        self._sub_tabs = QTabWidget()
        self._sub_tabs.setStyleSheet(
            f"QTabWidget::pane {{ border: 1px solid {P.DARK_BORDER}; "
            f"background: {P.DARK_PANEL}; }}"
            f"QTabBar::tab {{ background: {P.DARK_BUTTON}; color: {P.DARK_TEXT}; "
            f"padding: 4px 14px; border: 1px solid {P.DARK_BORDER}; "
            f"border-bottom: none; }}"
            f"QTabBar::tab:selected {{ background: {P.DARK_ACCENT}; color: {P.DARK_BG}; }}"
        )
        self._joint_tab = QWidget()
        self._world_tab = QWidget()
        self._sub_tabs.addTab(self._joint_tab, "Joint Jog")
        self._sub_tabs.addTab(self._world_tab, "World Jog")
        self._sub_tabs.currentChanged.connect(lambda _i: self._sync_ui())
        layout.addWidget(self._sub_tabs, 1)

        # --- Joint Jog tab content ---
        jl = QVBoxLayout(self._joint_tab)
        jl.setContentsMargins(6, 6, 6, 6)
        jl.setSpacing(6)

        # ── Config save / load buttons ────────────────────────────
        cfg_row = QHBoxLayout()
        cfg_row.setSpacing(6)
        self.save_cfg_btn = QPushButton("Save Config")
        self.save_cfg_btn.setFixedHeight(28)
        self.save_cfg_btn.setStyleSheet(P.success_btn_style(font_size=11, padding="2px 10px"))
        self.save_cfg_btn.setToolTip("Persist config to robot flash (M500)")
        self.save_cfg_btn.clicked.connect(self._save_config)
        cfg_row.addWidget(self.save_cfg_btn)

        self.load_cfg_btn = QPushButton("Load Config")
        self.load_cfg_btn.setFixedHeight(28)
        self.load_cfg_btn.setStyleSheet(P.warning_btn_style(font_size=11, padding="2px 10px"))
        self.load_cfg_btn.setToolTip("Reload config from robot flash (M501)")
        self.load_cfg_btn.clicked.connect(self._load_config)
        cfg_row.addWidget(self.load_cfg_btn)

        self.cfg_status = QLabel("")
        self.cfg_status.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; border: none; background: transparent;")
        cfg_row.addWidget(self.cfg_status, 1)
        jl.addLayout(cfg_row)

        # ── Joint rows ────────────────────────────────────────────
        for i in range(self.num_joints):
            row = JointRow(self._default_names[i], i, self._default_limits[i])
            row.slider.valueChanged.connect(lambda v, idx=i: self._on_slider(idx, v))
            row.spinbox.valueChanged.connect(lambda v, idx=i: self._on_spinbox(idx, v))
            row.jog_pad_clicked.connect(self._on_jog_pad)
            row.invert_toggled.connect(self._on_invert_toggled)
            row.hold_triggered = lambda idx, d, started: self._on_joint_hold(idx, d, started)
            self._rows.append(row)
            jl.addWidget(row)

        # ── Joint step-size chips ─────────────────────────────────
        jl.addWidget(QLabel("Step size:"))
        self._joint_chips_row = QHBoxLayout()
        jl.addLayout(self._joint_chips_row)
        self._joint_chips = []
        for s in JOG_STEP_SIZES:
            chip = StepChip(f"{s}°", s, "deg")
            chip.setStyleSheet(chip_style())
            chip.clicked.connect(lambda checked, c=chip: self._select_joint_step(c))
            self._joint_chips.append(chip)
            self._joint_chips_row.addWidget(chip)
        self._joint_chips[1].setChecked(True)  # default 0.5°
        self._joint_custom_step = QDoubleSpinBox()
        self._joint_custom_step.setRange(0.01, 360.0)
        self._joint_custom_step.setDecimals(2)
        self._joint_custom_step.setValue(1.0)
        self._joint_custom_step.setSuffix("°")
        self._joint_custom_step.setFixedWidth(90)
        self._joint_custom_step.setStyleSheet(P.input_style(font_size=11))
        self._joint_custom_step.valueChanged.connect(self._select_joint_custom)
        self._joint_chips_row.addWidget(self._joint_custom_step)
        self._joint_chips_row.addStretch()

        # ── Gripper Section ───────────────────────────────────────
        gripper_frame = QFrame()
        gripper_frame.setStyleSheet(f"QFrame {{ border: 1px solid {P.DARK_BORDER}; border-radius: 4px; background: {P.DARK_PANEL}; margin-top: 2px; }}")
        grip_layout = QVBoxLayout(gripper_frame)
        grip_layout.setContentsMargins(8, 6, 8, 6)
        grip_layout.setSpacing(6)

        grip_header = QHBoxLayout()
        grip_label = QLabel("Gripper")
        grip_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
        grip_header.addWidget(grip_label)
        grip_header.addStretch()
        grip_layout.addLayout(grip_header)

        gripper_row = QHBoxLayout()
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 100)
        self.gripper_slider.setValue(100)
        self.gripper_slider.setStyleSheet(P.slider_style())
        self.gripper_slider.valueChanged.connect(self._on_gripper_changed)
        gripper_row.addWidget(self.gripper_slider, 1)

        self.gripper_value_label = QLabel("100%")
        self.gripper_value_label.setFixedWidth(46)
        self.gripper_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.gripper_value_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 13px; font-weight: bold; border: none; background: transparent;")
        gripper_row.addWidget(self.gripper_value_label)
        grip_layout.addLayout(gripper_row)

        gripper_btns = QHBoxLayout()
        for text, callback in [("Open", self._open_gripper), ("Close", self._close_gripper)]:
            btn = QPushButton(text)
            btn.setFixedHeight(28)
            btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11))
            btn.clicked.connect(callback)
            gripper_btns.addWidget(btn)
        grip_layout.addLayout(gripper_btns)
        jl.addWidget(gripper_frame)

        # ── Jog Speed & Feed Rate ─────────────────────────────────
        params_frame = QFrame()
        params_frame.setStyleSheet(f"QFrame {{ border: 1px solid {P.DARK_BORDER}; border-radius: 4px; background: {P.DARK_PANEL}; margin-top: 2px; }}")
        params_layout = QHBoxLayout(params_frame)
        params_layout.setContentsMargins(8, 6, 8, 6)
        params_layout.setSpacing(6)

        # Jog speed slider
        speed_label = QLabel("Speed:")
        speed_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        params_layout.addWidget(speed_label)

        self.jog_speed_slider = QSlider(Qt.Horizontal)
        self.jog_speed_slider.setRange(1, 100)
        self.jog_speed_slider.setValue(50)
        self.jog_speed_slider.setFixedWidth(80)
        self.jog_speed_slider.setStyleSheet(P.slider_style(height=6, handle=14))
        params_layout.addWidget(self.jog_speed_slider)

        self.jog_speed_label = QLabel("50%")
        self.jog_speed_label.setFixedWidth(32)
        self.jog_speed_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        self.jog_speed_slider.valueChanged.connect(
            lambda v: self.jog_speed_label.setText(f"{v}%"))
        params_layout.addWidget(self.jog_speed_label)

        params_layout.addSpacing(12)

        # Feed rate
        fr_label = QLabel("Feed Rate:")
        fr_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        params_layout.addWidget(fr_label)
        self.feed_rate_input = QSpinBox()
        self.feed_rate_input.setRange(1, 50000)
        self.feed_rate_input.setValue(600)
        self.feed_rate_input.setFixedWidth(108)
        self.feed_rate_input.setStyleSheet(P.input_style(font_size=11))
        params_layout.addWidget(self.feed_rate_input)
        fr_unit = QLabel("mm/min")
        fr_unit.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        params_layout.addWidget(fr_unit)
        params_layout.addStretch()
        jl.addWidget(params_frame)

        # ── Direct Control Toggle ─────────────────────────────────
        self.direct_control_check = QCheckBox("Direct Control (real robot)")
        self.direct_control_check.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; spacing: 6px; background: transparent; border: none;")
        jl.addWidget(self.direct_control_check)

        # ── Soft Limits Toggle ────────────────────────────────────
        self.soft_limits_check = QCheckBox("Soft Limits (clamp to joint range)")
        self.soft_limits_check.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 11px; spacing: 6px; background: transparent; border: none;")
        self.soft_limits_check.setChecked(True)
        jl.addWidget(self.soft_limits_check)

        # ── Action Buttons ────────────────────────────────────────
        action_row = QHBoxLayout()
        action_row.setSpacing(6)

        btn_configs = [
            ("Move Joints", self._on_move, P.DARK_ACCENT),
            ("Reset Zero", self._on_reset_zero, P.DARK_BUTTON),
            ("Go to Zero", self._on_zero, P.DARK_SUCCESS),
            ("Stop", self._on_stop, P.DARK_ERROR),
        ]
        for text, callback, bg in btn_configs:
            btn = QPushButton(text)
            btn.setFixedHeight(30)
            btn.setStyleSheet(P.btn_style(bg, text=P.DARK_BG if bg in (P.DARK_ACCENT, P.DARK_SUCCESS) else P.DARK_TEXT, font_size=11))
            btn.clicked.connect(callback)
            action_row.addWidget(btn)
        jl.addLayout(action_row)
        jl.addStretch()

        # --- World Jog tab content ---
        wl = QVBoxLayout(self._world_tab)
        wl.setContentsMargins(6, 6, 6, 6)
        wl.setSpacing(6)

        # ── World speed override (separate from step size) ────────
        wspeed_row = QHBoxLayout()
        wspeed_lbl = QLabel("Speed:")
        wspeed_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        wspeed_row.addWidget(wspeed_lbl)
        self._world_speed_slider = QSlider(Qt.Horizontal)
        self._world_speed_slider.setRange(1, 100)
        self._world_speed_slider.setValue(50)
        self._world_speed_slider.setStyleSheet(P.slider_style(height=8, handle=18))
        wspeed_row.addWidget(self._world_speed_slider, 1)
        self._world_speed_label = QLabel("50%")
        self._world_speed_label.setFixedWidth(40)
        self._world_speed_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        self._world_speed_slider.valueChanged.connect(
            lambda v: self._world_speed_label.setText(f"{v}%"))
        wspeed_row.addWidget(self._world_speed_label)
        wl.addLayout(wspeed_row)

        # ── Linear step-size chips (XYZ mm) ────────────────────────
        lin_lbl = QLabel("Linear step (X/Y/Z):")
        lin_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; border: none; background: transparent;")
        wl.addWidget(lin_lbl)
        self._world_chips_row = QHBoxLayout()
        wl.addLayout(self._world_chips_row)
        self._world_lin_chips = []
        lin_sizes = [1, 10, 50, 100]
        for s in lin_sizes:
            chip = StepChip(f"{s} mm", s, "mm")
            chip.setStyleSheet(chip_style())
            chip.clicked.connect(lambda checked, c=chip: self._select_world_step(c, "lin"))
            self._world_lin_chips.append(chip)
            self._world_chips_row.addWidget(chip)
        self._world_chips_row.addStretch()
        self._world_lin_chips[1].setChecked(True)  # default 10 mm
        self._world_lin_step = 10.0

        # ── Rotational step-size chips (ABC deg) ───────────────────
        rot_lbl = QLabel("Rotational step (A/B/C):")
        rot_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; border: none; background: transparent;")
        wl.addWidget(rot_lbl)
        self._world_rot_row = QHBoxLayout()
        wl.addLayout(self._world_rot_row)
        self._world_rot_chips = []
        rot_sizes = [1, 5, 10, 45]
        for s in rot_sizes:
            chip = StepChip(f"{s}°", s, "deg")
            chip.setStyleSheet(chip_style())
            chip.clicked.connect(lambda checked, c=chip: self._select_world_step(c, "rot"))
            self._world_rot_chips.append(chip)
            self._world_rot_row.addWidget(chip)
        self._world_rot_row.addStretch()
        self._world_rot_chips[0].setChecked(True)  # default 1°
        self._world_rot_step = 1.0

        self._world_frame_group = QGroupBox("World Frame Jog (IK)")
        wf = QVBoxLayout(self._world_frame_group)
        wf.setSpacing(6)
        wf_note = QLabel("Tap = one step · hold = continuous. Tool frame.")
        wf_note.setWordWrap(True)
        wf_note.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 10px; background: transparent; border: none;")
        wf.addWidget(wf_note)

        # X/Y/Z (mm) and A/B/C (deg) button rows (JogButton: tap/hold)
        self._wf_rows = {}
        wf_axes = [
            ("X", "mm", "lin"),
            ("Y", "mm", "lin"),
            ("Z", "mm", "lin"),
            ("A", "deg", "rot"),
            ("B", "deg", "rot"),
            ("C", "deg", "rot"),
        ]
        for letter, unit, kind in wf_axes:
            row_l = QHBoxLayout()
            lbl = QLabel(f"{letter} ({unit})")
            lbl.setStyleSheet(f"color: {P.DARK_ACCENT2}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
            lbl.setFixedWidth(64)
            row_l.addWidget(lbl)
            self._wf_rows[letter] = {}
            for d in (-1, 1):
                btn = JogButton("-" if d < 0 else "+")
                btn.setStyleSheet(jog_btn_style())
                btn.setMinimumHeight(56)
                btn.setMinimumWidth(72)
                btn.step_callback = lambda L=letter, dd=d: self._on_world_jog(L, self._world_step(kind, dd))
                btn.hold_callback = lambda start, L=letter, dd=d: self._on_world_hold(L, dd, start)
                row_l.addWidget(btn)
                self._wf_rows[letter][d] = btn
            row_l.addStretch()
            wf.addLayout(row_l)

        wl.addWidget(self._world_frame_group)
        wl.addStretch()
        self._world_frame_group.setEnabled(False)  # enabled once a DH arm is set

        # ── Sync timer for position readout ───────────────────────
        self._sync_timer = QTimer(self)
        self._sync_timer.timeout.connect(self._sync_from_simulation)
        self._sync_timer.start(100)  # 10 Hz

    def _on_step_changed(self, idx):
        """Update step size for all joint rows."""
        self._step_size_index = idx
        sizes = JOG_STEP_SIZES
        step = sizes[idx] if idx < len(sizes) else 1.0
        for row in self._rows:
            row.set_step_size(step)

    def _on_slider(self, idx, value):
        deg = value / 10.0
        self.values[idx] = deg
        self._rows[idx].spinbox.blockSignals(True)
        self._rows[idx].spinbox.setValue(deg)
        self._rows[idx].spinbox.blockSignals(False)
        self._apply_joints()

    def _on_spinbox(self, idx, value):
        self.values[idx] = value
        self._rows[idx].slider.blockSignals(True)
        self._rows[idx].slider.setValue(int(value * 10))
        self._rows[idx].slider.blockSignals(False)
        self._apply_joints()

    def _on_jog_pad(self, idx, delta_deg):
        """Jog pad button: relative step move via #MV.

        The studio computes the step count (gear ratio) and clamps the speed
        to the motion-panel max_speed; the robot just executes the raw move
        with its accel/decel profile. Using #MV directly (always relative)
        avoids the fragile G91/G0 mode dance that made some jogs silently do
        nothing. Only sends to the robot when Direct Control is ON. Also
        updates the simulation model so the view stays in sync.
        """
        if idx < 0 or idx >= 6:
            return
        # Update sim — clamp to soft limits when enabled
        new_val = self.values[idx] + delta_deg
        if self.soft_limits_check.isChecked():
            lo, hi = self._rows[idx].limits if hasattr(self._rows[idx], "limits") else (-180, 180)
            new_val = max(lo, min(hi, new_val))
            if abs(new_val - (self.values[idx] + delta_deg)) > 1e-9:
                # clamped — don't send the move
                delta_deg = new_val - self.values[idx]
                if delta_deg == 0:
                    return
        self.values[idx] = new_val
        self._rows[idx].set_value(self.values[idx])
        robot_name = self._get_active_robot()
        if self.sim and robot_name:
            # Sim joints are RADIANS; the jog values are degrees.
            self.sim.set_joint_positions(robot_name, [math.radians(v) for v in self.values])

        if not (self.direct_control_check.isChecked() and self.connection_manager):
            return
        if delta_deg == 0:
            # HOME button: G28 stub
            self.connection_manager.command("G28")
            return
        # Speed slider = % of the joint's max_speed (studio-enforced cap).
        pct = self.jog_speed_slider.value()
        self.connection_manager.jog_joint(idx + 1, delta_deg, speed_pct=pct)

    # ── World Frame (Cartesian) Jog ──────────────────────────────

    def _on_world_jog(self, letter, delta):
        """World-frame jog: move tool by delta in X/Y/Z (mm).

        Routes through the Three.js viewport's validated PoE IK instead
        of the Python DHArm IK (which doesn't match the real robot).
        """
        dx, dy, dz = 0.0, 0.0, 0.0
        if letter == "X": dx = delta
        elif letter == "Y": dy = delta
        elif letter == "Z": dz = delta
        else:
            return  # rotation deltas handled by gizmo drag only

        # Find the viewport widget from the main window.
        viewport = None
        if self.connection_manager:
            mw = self.connection_manager.parent()
            if mw and hasattr(mw, 'viewport'):
                viewport = mw.viewport

        if viewport and hasattr(viewport, 'jog_world'):
            viewport.jog_world(dx, dy, dz)
            # Poll for the JS IK result after a short delay.
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(80, lambda: self._readback_js_joints())
        else:
            # Fallback: Python IK.
            if not self._dh_arm:
                return
            arm = self._dh_arm
            n = arm.num_joints
            joints_rad = [math.radians(v) for v in self.values[:n]]
            T_cur = arm.forward(joints_rad)
            p_cur = T_cur[:3, 3]
            d_m = delta / 1000.0
            vec = {"X": [d_m, 0, 0], "Y": [0, d_m, 0], "Z": [0, 0, d_m]}[letter]
            p_new = p_cur + T_cur[:3, :3] @ np.array(vec, dtype=float)
            sol = arm.compute_ik(p_new, T_cur[:3, :3], joint_angles=list(joints_rad), retries=15)
            if sol is None:
                self._pos_readout.setText("T: IK FAILED")
                return
            new_deg = [math.degrees(a) for a in sol]
            self.set_joints(new_deg)
            self._apply_joints()

    def _readback_js_joints(self):
        """Read joint angles back from the JS viewport after PoE IK solve."""
        viewport = None
        if self.connection_manager:
            mw = self.connection_manager.parent()
            if mw and hasattr(mw, 'viewport'):
                viewport = mw.viewport
        if not viewport or not hasattr(viewport, '_web'):
            return
        def _cb(joints):
            if joints and isinstance(joints, list) and len(joints) >= 6:
                old = list(self.values[:6])
                new = list(joints[:6])
                # Update panel state + 3D model.
                self.set_joints(new)
                self._apply_joints()
                # Send coordinated move to the real robot.
                if self.connection_manager and self.connection_manager.is_connected:
                    deltas = [new[i] - old[i] for i in range(6)]
                    self.connection_manager.coordinated_move(deltas)
        viewport._web.page().runJavaScript(
            'window.astra.getJoints ? window.astra.getJoints() : null', _cb)

    def _tool_pos(self):
        if not self._dh_arm:
            return None
        n = self._dh_arm.num_joints
        T = self._dh_arm.forward([math.radians(v) for v in self.values[:n]])
        return T[:3, 3] * 1000.0  # mm

    def _tool_pos_str(self):
        p = self._tool_pos()
        if p is None:
            return "—"
        return "[%.0f, %.0f, %.0f] mm" % (p[0], p[1], p[2])

    # ── Step-size chips ──────────────────────────────────────────

    def _select_joint_step(self, chip):
        for c in self._joint_chips:
            c.setChecked(c is chip)
        self._step_size_index = JOG_STEP_SIZES.index(chip.value) if chip.value in JOG_STEP_SIZES else self._step_size_index
        for row in self._rows:
            row.set_step_size(chip.value)

    def _select_joint_custom(self, value):
        for c in self._joint_chips:
            c.setChecked(False)
        for row in self._rows:
            row.set_step_size(value)

    def _select_world_step(self, chip, kind):
        chips = self._world_lin_chips if kind == "lin" else self._world_rot_chips
        for c in chips:
            c.setChecked(c is chip)
        if kind == "lin":
            self._world_lin_step = float(chip.value)
        else:
            self._world_rot_step = float(chip.value)

    def _world_step(self, kind, direction):
        return (self._world_lin_step if kind == "lin" else self._world_rot_step) * direction

    # ── Hold-to-jog ──────────────────────────────────────────────

    def _on_joint_hold(self, idx, delta_deg, started):
        """Hold a joint jog button: continuous #JC while held, #H on release."""
        if not self.connection_manager:
            return
        if started:
            self._active_hold = ("joint", idx)
            self.connection_manager.jog_start(idx + 1, 1 if delta_deg > 0 else -1,
                                              self.jog_speed_slider.value())
        else:
            self._active_hold = None
            self.connection_manager.jog_stop()

    def _on_world_hold(self, letter, direction, started):
        """Hold a world jog button: repeat the IK step on a timer while held."""
        if not self._dh_arm:
            return
        if started:
            self._active_hold = ("world", letter, direction)
            if not hasattr(self, "_world_hold_timer"):
                self._world_hold_timer = QTimer(self)
                self._world_hold_timer.setInterval(80)
                self._world_hold_timer.timeout.connect(self._world_hold_tick)
            self._world_hold_timer.start()
            # First step immediately.
            self._world_hold_tick()
        else:
            self._active_hold = None
            if hasattr(self, "_world_hold_timer"):
                self._world_hold_timer.stop()
            if self.connection_manager:
                self.connection_manager.jog_stop()

    def _world_hold_tick(self):
        if not self._active_hold or self._active_hold[0] != "world":
            return
        _, letter, direction = self._active_hold
        kind = "lin" if letter in ("X", "Y", "Z") else "rot"
        step = self._world_step(kind, direction)
        self._on_world_jog(letter, step)

    def _on_invert_toggled(self, idx, invert):
        """Direction inversion toggle -> #CFG di bit to the robot."""
        if not self.connection_manager:
            return
        joint = idx + 1
        ok = self.connection_manager.cfg_write(joint, di=1 if invert else 0)
        if not ok:
            self.cfg_status.setText(f"J{joint} invert write failed")
        else:
            self.cfg_status.setText(f"J{joint} invert={'ON' if invert else 'OFF'}")

    def _save_config(self):
        if not self.connection_manager:
            self.cfg_status.setText("Not connected")
            return
        ok = self.connection_manager.save_robot_config()
        self.cfg_status.setText("Config saved" if ok else "Save failed")

    def _load_config(self):
        if not self.connection_manager:
            self.cfg_status.setText("Not connected")
            return
        ok = self.connection_manager.load_robot_config()
        self.cfg_status.setText("Config loaded" if ok else "Load failed")
        # Refresh invert toggles from the robot
        for i in range(self.num_joints):
            cfg = self.connection_manager.cfg_read(i + 1)
            if cfg:
                self._rows[i].set_invert(bool(cfg.get("dir_inverted", 0)))

    def _on_gripper_changed(self, value):
        self.gripper_value = value
        self.gripper_value_label.setText(f"{value}%")
        self.gripper_changed.emit(value / 100.0)
        if self.direct_control_check.isChecked() and self.connection_manager:
            self.connection_manager.set_gripper(value / 100.0)

    def _open_gripper(self):
        self.gripper_slider.setValue(0)
        if self.connection_manager:
            self.connection_manager.set_gripper(0.0)

    def _close_gripper(self):
        self.gripper_slider.setValue(100)
        if self.connection_manager:
            self.connection_manager.set_gripper(1.0)

    def _get_active_robot(self):
        """Get the name of the first available robot in the simulation."""
        if self.sim and self.sim.robots:
            for name in self.sim.robots:
                return name
        return None

    def _apply_joints(self):
        """Sliders/spinboxes are SIMULATION-ONLY — never send to the robot.

        Real-robot motion happens only via the jog pad buttons (G-code).
        """
        robot_name = self._get_active_robot()
        if self.sim and robot_name:
            # Sim joints are RADIANS; the jog values are degrees.
            self.sim.set_joint_positions(robot_name, [math.radians(v) for v in self.values])
        self.joint_moved.emit(self.values)

    def _on_move(self):
        self.move_requested.emit(self.values)

    def _on_reset_zero(self):
        self.set_joints([0.0] * self.num_joints)

    def _on_zero(self):
        self.set_joints([0.0] * self.num_joints)
        self.home_requested.emit()

    def _on_stop(self):
        """Emergency stop jogging."""
        self._active_hold = None
        if hasattr(self, "_world_hold_timer"):
            self._world_hold_timer.stop()
        if self.connection_manager and self.connection_manager.is_connected:
            self.connection_manager.jog_stop()
            self.connection_manager.stop()
        if self.sim:
            self.sim.running = False

    def _sync_from_simulation(self):
        """Periodically sync position readouts from simulation."""
        if not self.sim:
            return
        for robot_name in self.sim.robots:
            try:
                joints = self.sim.get_joint_positions(robot_name)
                rev = self.sim.get_revolute_joints(robot_name)
                for i in range(min(len(rev), len(self._rows))):
                    j_idx = rev[i]["index"]
                    pos_deg = joints[j_idx]  # already in degrees? No, radians
                    # Check if positions are in radians (value around 0-6) vs degrees
                    max_val = max(abs(j) for j in joints[:6]) if joints else 0
                    if max_val > 6.28:  # degrees
                        pos_deg = joints[j_idx]
                    else:  # radians
                        pos_deg = math.degrees(joints[j_idx])
                    self._rows[i].pos_readout.setText(f"{pos_deg:.1f}°")
                    # Color code: close to target = green, off = warning
                    diff = abs(pos_deg - self.values[i])
                    if diff < 0.3:
                        color = P.DARK_SUCCESS
                    elif diff < 1.0:
                        color = P.DARK_WARNING
                    else:
                        color = P.DARK_TEXT_DIM
                    self._rows[i].pos_readout.setStyleSheet(
                        f"color: {color}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
            except Exception:
                pass
            # Refresh the header live readout — Cartesian tool pose on the
            # World tab, joint angles on the Joint tab.
            try:
                degs = []
                for i in range(min(6, len(self._rows))):
                    degs.append(math.degrees(joints[rev[i]["index"]]))
                on_world = (hasattr(self, "_sub_tabs")
                            and self._sub_tabs.currentWidget() is getattr(self, "_world_tab", None))
                if on_world:
                    # Use the sim joints to keep the model in sync, then show
                    # the tool position in the Cartesian frame being jogged.
                    if len(degs) >= 6:
                        self.values = list(degs[:6])
                    self._pos_readout.setText(f"T: {self._tool_pos_str()}")
                else:
                    self._pos_readout.setText(
                        f"J: {', '.join(f'{v:.1f}' for v in degs[:6])}")
            except Exception:
                pass
            break

    def set_joints(self, positions):
        """Set all joint positions from external source (e.g., FK apply)."""
        if len(positions) != self.num_joints:
            return
        self.values = [float(p) for p in positions[:self.num_joints]]
        self._sync_ui()

    def get_joints(self):
        return list(self.values)

    def _sync_ui(self):
        """Sync UI widgets with current values."""
        for i in range(self.num_joints):
            self._rows[i].set_value(self.values[i])
        # Refresh the live position readout (joints or tool, per active tab).
        try:
            if hasattr(self, "_sub_tabs") and self._sub_tabs.currentWidget() is getattr(self, "_world_tab", None):
                self._pos_readout.setText(f"T: {self._tool_pos_str()}")
            else:
                self._pos_readout.setText("J: " + ", ".join(
                    f"{v:.1f}" for v in self.values[:6]))
        except Exception:
            pass

    def refresh_poses(self, poses):
        """Update joint positions from encoder feedback."""
        if poses and len(poses) == self.num_joints:
            for i in range(min(len(poses), len(self._rows))):
                self._rows[i].set_encoder_feedback(poses[i])
