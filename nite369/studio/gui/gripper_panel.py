"""Gripper control panel for Nite 369.

Provides interface for controlling the gripper on Slave 2 (axis 3).

All gripper tuning is done HERE in software — no firmware changes needed:
  - Open / Close limits set as PWM pulse widths (microseconds)
  - PWM inversion: if the servo moves the wrong way, swap the limits
  - Test buttons: Full Open / Full Close / Center
  - Live position slider mapped to the configured pulse range

The firmware maps #G<steps> (0..5000) to a servo pulse:
  angle_10 = steps * 1800 / 5000          (0..1800 deci-degrees)
  us       = 500 + angle_10 * 2000 / 1800  (500..2500 us)
This panel inverts that math to convert a desired pulse width (us) back
into the #G steps the firmware expects.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSlider, QSpinBox, QFrame, QGroupBox, QCheckBox, QGridLayout,
)
from PyQt5.QtCore import Qt, pyqtSignal

from . import palette as P


def _us_to_g_steps(us: float) -> int:
    """Convert a servo pulse width (us) to the firmware #G step value.

    Firmware: steps(0..5000) -> angle_10(0..1800) -> us(500..2500).
    Invert: us -> angle_10 = (us-500) * 1800 / 2000, then steps = angle_10 * 5000 / 1800.
    Clamped to [0, 5000] and rounded to an int.
    """
    us = max(500.0, min(2500.0, float(us)))
    angle_10 = (us - 500.0) * 1800.0 / 2000.0
    return int(round(angle_10 * 5000.0 / 1800.0))


def _g_steps_to_us(steps: int) -> float:
    """Convert a firmware #G step value (0..5000) to a pulse width (us)."""
    steps = max(0, min(5000, int(steps)))
    angle_10 = steps * 1800.0 / 5000.0
    return 500.0 + angle_10 * 2000.0 / 1800.0


class GripperControlPanel(QWidget):
    """Gripper control panel for Nite 369 robot."""

    gripper_command = pyqtSignal(str)  # Send command to robot
    # 0.0 (open) .. 1.0 (closed) — emitted when the user changes the gripper.
    gripper_position_changed = pyqtSignal(float)

    # Default pulse range (microseconds). Firmware: 500us = closed, 2500us = open.
    DEFAULT_CLOSE_US = 500
    DEFAULT_OPEN_US = 2500

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._current_steps = 0
        self._target_steps = 0
        self._setup_ui()
        self._refresh_limits()

    def set_connection_manager(self, cm):
        """Set the connection manager."""
        self._connection_manager = cm
        if cm:
            cm.encoderUpdate.connect(self._on_encoder_data)

    # ── Limit computation ─────────────────────────────────────────

    @property
    def close_us(self) -> int:
        """Pulse width (us) that CLOSES the gripper."""
        return self._close_us

    @property
    def open_us(self) -> int:
        """Pulse width (us) that OPENS the gripper."""
        return self._open_us

    def _refresh_limits(self):
        """Recompute open/close pulse limits from the spinboxes + inversion."""
        close = self.close_us_spin.value()
        opn = self.open_us_spin.value()
        if self.invert_check.isChecked():
            # Inverted: swap so the effective open/close pulse is reversed.
            self._open_us, self._close_us = close, opn
        else:
            self._close_us, self._open_us = close, opn
        # Keep the slider in the configured range.
        self.pos_slider.setRange(
            min(self._close_us, self._open_us),
            max(self._close_us, self._open_us),
        )
        lo = min(self._close_us, self._open_us)
        hi = max(self._close_us, self._open_us)
        self.slider_min_label.setText(str(lo))
        self.slider_max_label.setText(str(hi))
        self._status(
            f"Open = {self._open_us}us · Close = {self._close_us}us"
            + (" · INVERTED" if self.invert_check.isChecked() else ""),
            "info",
        )

    # ── Command helpers ───────────────────────────────────────────

    def _send(self, steps: int):
        """Send a gripper position (steps) to the robot."""
        steps = max(0, min(5000, int(steps)))
        self.gripper_command.emit(f"G{steps}")
        self._target_steps = steps
        self.pos_label.setText(str(steps))
        self._status(f"Sent G{steps} ({_g_steps_to_us(steps):.0f} us)")

    def _send_us(self, us: float):
        """Send a gripper position as a pulse width (us)."""
        self._send(_us_to_g_steps(us))

    def _move_open(self):
        """Open the gripper (to the configured open pulse)."""
        self._send_us(self._open_us)
        self._status(f"OPEN -> {self._open_us} us")

    def _move_close(self):
        """Close the gripper (to the configured close pulse)."""
        self._send_us(self._close_us)
        self._status(f"CLOSE -> {self._close_us} us")

    def _move_center(self):
        """Center the gripper (midpoint of open/close)."""
        mid = (self._open_us + self._close_us) // 2
        self._send_us(mid)
        self._status(f"CENTER -> {mid} us")

    def _on_slider_changed(self, value):
        """Slider moved — just update the label, do NOT send yet.

        Sending on every tick floods the connection with #G commands while
        dragging (the gripper stutters or the link wedges). The command is
        sent once, when the slider is released (_on_slider_released).
        """
        self.pos_label.setText(str(value))
        self._target_steps = _us_to_g_steps(value)

    def _on_slider_released(self):
        """Slider released — send the final position once."""
        self._send_us(self.pos_slider.value())

    def _on_encoder_data(self, encoder_values: list):
        """Update position display from encoder data (if a gripper encoder exists)."""
        if len(encoder_values) > 5:
            self._current_steps = int(encoder_values[5])
            self.pos_label.setText(str(self._current_steps))

    def _status(self, msg, level="info"):
        colors = {"info": P.DARK_TEXT_DIM, "success": P.DARK_SUCCESS,
                  "error": P.DARK_ERROR, "warning": P.DARK_WARNING}
        self.status_text.setStyleSheet(
            f"color: {colors.get(level, P.DARK_TEXT_DIM)}; font-size: 12px; border: none; background: transparent;"
        )
        self.status_text.setText(msg)

    def set_connected(self, connected: bool):
        """Update UI for connection state."""
        if connected:
            self.status_led.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
            self.status_text.setText("Connected")
        else:
            self.status_led.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
            self.status_text.setText("Disconnected")

    # ── UI ────────────────────────────────────────────────────────

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # ── Title ──────────────────────────────────────────────
        title = QLabel("Gripper Control")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 15px; font-weight: bold; background: transparent; border: none;")
        layout.addWidget(title)

        # ── Position Display ───────────────────────────────────
        pos_group = QGroupBox("Position")
        pos_group.setStyleSheet(P.groupbox_style())
        pos_layout = QHBoxLayout(pos_group)

        self.pos_label = QLabel("0")
        self.pos_label.setStyleSheet(
            f"color: {P.DARK_ACCENT}; font-size: 22px; font-weight: bold; "
            "font-family: 'Consolas', monospace; background: transparent; border: none;"
        )
        self.pos_label.setAlignment(Qt.AlignCenter)
        pos_layout.addWidget(self.pos_label)

        pos_unit = QLabel("steps")
        pos_unit.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 13px; background: transparent; border: none;")
        pos_layout.addWidget(pos_unit)
        layout.addWidget(pos_group)

        # ── Open / Close Limits ────────────────────────────────
        lim_group = QGroupBox("Open / Close Limits (pulse width)")
        lim_group.setStyleSheet(P.groupbox_style())
        lim_layout = QGridLayout(lim_group)
        lim_layout.setSpacing(6)

        lim_layout.addWidget(QLabel("Open pulse (us):"), 0, 0)
        self.open_us_spin = QSpinBox()
        self.open_us_spin.setRange(500, 2500)
        self.open_us_spin.setValue(self.DEFAULT_OPEN_US)
        self.open_us_spin.setFixedWidth(100)
        self.open_us_spin.setStyleSheet(P.input_style(font_size=11))
        self.open_us_spin.valueChanged.connect(self._refresh_limits)
        lim_layout.addWidget(self.open_us_spin, 0, 1)

        lim_layout.addWidget(QLabel("Close pulse (us):"), 1, 0)
        self.close_us_spin = QSpinBox()
        self.close_us_spin.setRange(500, 2500)
        self.close_us_spin.setValue(self.DEFAULT_CLOSE_US)
        self.close_us_spin.setFixedWidth(100)
        self.close_us_spin.setStyleSheet(P.input_style(font_size=11))
        self.close_us_spin.valueChanged.connect(self._refresh_limits)
        lim_layout.addWidget(self.close_us_spin, 1, 1)

        self.invert_check = QCheckBox("Invert (swap open/close)")
        self.invert_check.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 11px; background: transparent; border: none;")
        self.invert_check.toggled.connect(self._refresh_limits)
        lim_layout.addWidget(self.invert_check, 2, 0, 1, 2)

        note = QLabel("The firmware maps 500us = closed, 2500us = open. If your "
                      "servo moves the wrong way, tick Invert instead of rewiring.")
        note.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 10px; background: transparent; border: none;")
        note.setWordWrap(True)
        lim_layout.addWidget(note, 3, 0, 1, 2)

        layout.addWidget(lim_group)

        # ── Manual Control ─────────────────────────────────────
        manual_group = QGroupBox("Manual Control")
        manual_group.setStyleSheet(P.groupbox_style())
        manual_layout = QVBoxLayout(manual_group)

        btn_layout = QHBoxLayout()

        self.close_btn = QPushButton("CLOSE")
        self.close_btn.setFixedHeight(38)
        self.close_btn.setStyleSheet(P.danger_btn_style(font_size=13, padding="8px 20px"))
        self.close_btn.clicked.connect(lambda: self._move_close())
        btn_layout.addWidget(self.close_btn)

        self.open_btn = QPushButton("OPEN")
        self.open_btn.setFixedHeight(38)
        self.open_btn.setStyleSheet(P.success_btn_style(font_size=13, padding="8px 20px"))
        self.open_btn.clicked.connect(lambda: self._move_open())
        btn_layout.addWidget(self.open_btn)

        self.center_btn = QPushButton("CENTER")
        self.center_btn.setFixedHeight(38)
        self.center_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=12, padding="8px 14px"))
        self.center_btn.clicked.connect(lambda: self._move_center())
        btn_layout.addWidget(self.center_btn)

        manual_layout.addLayout(btn_layout)
        layout.addWidget(manual_group)

        # ── Position Slider (pulse width) ──────────────────────
        slider_group = QGroupBox("Pulse Width Slider (us)")
        slider_group.setStyleSheet(P.groupbox_style())
        slider_layout = QVBoxLayout(slider_group)

        self.pos_slider = QSlider(Qt.Horizontal)
        self.pos_slider.setRange(500, 2500)
        self.pos_slider.setValue(1500)
        self.pos_slider.setTickPosition(QSlider.TicksBelow)
        self.pos_slider.setTickInterval(250)
        self.pos_slider.setStyleSheet(P.slider_style())
        self.pos_slider.valueChanged.connect(self._on_slider_changed)
        # Send only when the user releases the slider, not on every tick —
        # dragging otherwise floods the link with #G commands.
        self.pos_slider.sliderReleased.connect(self._on_slider_released)
        slider_layout.addWidget(self.pos_slider)

        slider_labels = QHBoxLayout()
        self.slider_min_label = QLabel("500")
        self.slider_min_label.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 10px; font-family: 'Consolas', monospace; background: transparent; border: none;")
        slider_labels.addWidget(self.slider_min_label)
        slider_labels.addStretch()
        self.slider_max_label = QLabel("2500")
        self.slider_max_label.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 10px; font-family: 'Consolas', monospace; background: transparent; border: none;")
        slider_labels.addWidget(self.slider_max_label)
        slider_layout.addLayout(slider_labels)
        layout.addWidget(slider_group)

        # ── Quick Presets ──────────────────────────────────────
        preset_group = QGroupBox("Quick Presets")
        preset_group.setStyleSheet(P.groupbox_style())
        preset_layout = QHBoxLayout(preset_group)

        presets = [
            ("Full Open", self.DEFAULT_OPEN_US),
            ("Half Open", 1500),
            ("Closed", self.DEFAULT_CLOSE_US),
        ]
        for name, us in presets:
            btn = QPushButton(name)
            btn.setFixedHeight(34)
            btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11))
            btn.clicked.connect(lambda checked, p=us: self._send_us(p))
            preset_layout.addWidget(btn)

        layout.addWidget(preset_group)

        # ── Status Bar ─────────────────────────────────────────
        status_frame = QFrame()
        status_frame.setStyleSheet(f"QFrame {{ background: {P.DARK_INPUT}; border: 1px solid {P.DARK_BORDER}; border-radius: 4px; }}")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(10, 5, 10, 5)

        self.status_led = QLabel("●")
        self.status_led.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        status_layout.addWidget(self.status_led)

        self.status_text = QLabel("Gripper is ready")
        self.status_text.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; border: none; background: transparent;")
        status_layout.addWidget(self.status_text)
        status_layout.addStretch()

        layout.addWidget(status_frame)
        layout.addStretch()
