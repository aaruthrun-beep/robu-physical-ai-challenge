"""Professional encoder monitoring panel for Nite 369.

Displays real-time encoder positions for all 6 joints with:
- Animated progress bars per joint
- Numerical degree readout with precision
- Color-coded status (normal/warning/error)
- Zero-offset calibration per encoder
- Configurable polling rate
- Historical min/max tracking
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSlider, QSpinBox, QFrame, QGridLayout, QGroupBox,
    QProgressBar,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QPropertyAnimation, QEasingCurve
from PyQt5.QtGui import QColor, QFont, QPainter, QLinearGradient, QBrush

from . import palette as P


class EncoderBar(QWidget):
    """Single encoder channel with progress bar, label, and value."""

    def __init__(self, joint_name: str, joint_index: int, parent=None):
        super().__init__(parent)
        self.joint_name = joint_name
        self.joint_index = joint_index
        self._value = 0.0
        self._min_val = 0.0
        self._max_val = 0.0
        self._range = (-180, 180)
        self._calibrated = False
        self._status = "normal"

        self.setMinimumHeight(56)
        self.setStyleSheet(
            f"EncoderBar {{ background: {P.DARK_PANEL}; border: 1px solid {P.DARK_BORDER}; border-radius: 4px; }}"
        )
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(8)

        # Joint name label
        self.name_label = QLabel(self.joint_name)
        self.name_label.setMinimumWidth(110)
        self.name_label.setStyleSheet(f"""
            QLabel {{
                color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold;
                border: none; background: transparent;
            }}
        """)
        layout.addWidget(self.name_label)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 3600)  # -180.0 to 180.0 in tenths
        self.progress.setValue(1800)  # Center = 0 degrees
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(20)
        self.progress.setStyleSheet(f"""
            QProgressBar {{
                background: {P.DARK_INPUT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
                text-align: center;
            }}
            QProgressBar::chunk {{
                background: {P.DARK_ACCENT}; border-radius: 3px;
            }}
        """)
        layout.addWidget(self.progress, 1)

        # Value label
        self.value_label = QLabel("0.00°")
        self.value_label.setMinimumWidth(90)
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_label.setStyleSheet(f"""
            QLabel {{
                color: {P.DARK_SUCCESS}; font-size: 14px; font-weight: bold;
                font-family: 'Consolas', monospace;
                border: none; background: transparent;
            }}
        """)
        layout.addWidget(self.value_label)

        # Raw count
        self.raw_label = QLabel("0")
        self.raw_label.setMinimumWidth(60)
        self.raw_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.raw_label.setStyleSheet(f"""
            QLabel {{
                color: {P.DARK_TEXT_DIM}; font-size: 12px;
                font-family: 'Consolas', monospace;
                border: none; background: transparent;
            }}
        """)
        layout.addWidget(self.raw_label)

        # Resolution info
        self.res_label = QLabel("12-bit")
        self.res_label.setMinimumWidth(40)
        self.res_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.res_label.setStyleSheet(f"""
            QLabel {{
                color: {P.DARK_TEXT_MUTED}; font-size: 10px;
                font-family: 'Consolas', monospace;
                border: none; background: transparent;
            }}
        """)
        self.res_label.setToolTip("AS5600: 12-bit = 0.0879°/LSB")
        layout.addWidget(self.res_label)

        # Calibration indicator
        self.cal_indicator = QLabel("●")
        self.cal_indicator.setFixedWidth(20)
        self.cal_indicator.setAlignment(Qt.AlignCenter)
        self.cal_indicator.setStyleSheet(f"""
            QLabel {{
                color: {P.DARK_TEXT_MUTED}; font-size: 14px; font-weight: bold;
                border: none; background: transparent;
            }}
        """)
        self.cal_indicator.setToolTip("Not calibrated. Right-click to set zero.")
        layout.addWidget(self.cal_indicator)
    
    def set_value(self, degrees: float, raw: int = 0):
        """Update the displayed encoder value."""
        self._value = degrees
        
        # Update progress bar
        # Map -180..180 to 0..3600
        pct = int((degrees + 180) * 10)
        pct = max(0, min(3600, pct))
        self.progress.setValue(pct)
        
        # Update labels
        self.value_label.setText(f"{degrees:.2f}°")
        self.raw_label.setText(str(raw))
        
        # Color code based on value range and status
        abs_val = abs(degrees)
        if abs_val > 170:
            self._set_bar_color(P.DARK_ERROR)  # Near limit - error
            self.value_label.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 14px; font-weight: bold; font-family: 'Consolas', monospace; border: none; background: transparent;")
        elif abs_val > 135:
            self._set_bar_color(P.DARK_WARNING)  # Warning
            self.value_label.setStyleSheet(f"color: {P.DARK_WARNING}; font-size: 14px; font-weight: bold; font-family: 'Consolas', monospace; border: none; background: transparent;")
        else:
            self._set_bar_color(P.DARK_ACCENT)  # Normal
            self.value_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 14px; font-weight: bold; font-family: 'Consolas', monospace; border: none; background: transparent;")

    def set_calibrated(self, calibrated: bool):
        """Mark this encoder as calibrated."""
        self._calibrated = calibrated
        if calibrated:
            self.cal_indicator.setText("●")
            self.cal_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 14px; font-weight: bold; border: none; background: transparent;")
            self.cal_indicator.setToolTip("Zero offset set")
        else:
            self.cal_indicator.setText("○")
            self.cal_indicator.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 14px; font-weight: bold; border: none; background: transparent;")
            self.cal_indicator.setToolTip("Not calibrated")

    def _set_bar_color(self, color_hex: str):
        """Set progress bar color based on status."""
        self.progress.setStyleSheet(f"""
            QProgressBar {{
                background: {P.DARK_INPUT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background: {color_hex}; border-radius: 3px;
            }}
        """)
    
    @property
    def value(self) -> float:
        return self._value
    
    def reset_min_max(self):
        self._min_val = self._value
        self._max_val = self._value


class EncoderMonitorPanel(QWidget):
    """Live encoder monitoring panel for all 6 robot joints."""

    zero_requested = pyqtSignal(int)   # joint_index
    reset_zeros_requested = pyqtSignal()
    poll_rate_changed = pyqtSignal(int)  # ms

    JOINT_LABELS = ["J1 — Base Rotation", "J2 — Shoulder", "J3 — Elbow",
                    "J4 — Forearm Roll", "J5 — Wrist Pitch", "J6 — Wrist Roll"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._bars = []
        self._polling_active = False
        self._setup_ui()
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._request_poll)
        self._poll_interval = 100  # ms

    def set_connection_manager(self, cm):
        """Set the connection manager for encoder data."""
        self._connection_manager = cm
        if cm:
            # Connect to Nite protocol encoder updates
            cm.encoderUpdate.connect(self._on_encoder_data)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # ── Header ──────────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel("Encoder Monitor")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 15px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)

        self.poll_indicator = QLabel("●")
        self.poll_indicator.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        self.poll_indicator.setToolTip("Polling status")
        header.addWidget(self.poll_indicator)

        header.addStretch()

        poll_label = QLabel("Poll:")
        poll_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        header.addWidget(poll_label)

        self.poll_rate_spin = QSpinBox()
        self.poll_rate_spin.setRange(20, 1000)
        self.poll_rate_spin.setValue(100)
        self.poll_rate_spin.setFixedWidth(100)
        self.poll_rate_spin.setStyleSheet(P.input_style(font_size=12))
        self.poll_rate_spin.valueChanged.connect(self._on_poll_rate_changed)
        header.addWidget(self.poll_rate_spin)
        poll_unit = QLabel("ms")
        poll_unit.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        header.addWidget(poll_unit)

        self.toggle_poll_btn = QPushButton("Pause")
        self.toggle_poll_btn.setFixedHeight(30)
        self.toggle_poll_btn.setFixedWidth(84)
        self.toggle_poll_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=12))
        self.toggle_poll_btn.clicked.connect(self._toggle_polling)
        header.addWidget(self.toggle_poll_btn)
        layout.addLayout(header)

        # ── Encoder Bars ────────────────────────────────────────
        for i, label in enumerate(self.JOINT_LABELS):
            bar = EncoderBar(label, i)
            self._bars.append(bar)
            layout.addWidget(bar)

        # ── Control Bar ─────────────────────────────────────────
        control_frame = QFrame()
        control_frame.setStyleSheet(f"QFrame {{ border: 1px solid {P.DARK_BORDER}; border-radius: 4px; background: {P.DARK_PANEL}; }}")
        control_layout = QHBoxLayout(control_frame)
        control_layout.setContentsMargins(8, 6, 8, 6)
        control_layout.setSpacing(6)

        self.calibrate_btn = QPushButton("Calibrate Zero")
        self.calibrate_btn.setFixedHeight(32)
        self.calibrate_btn.setStyleSheet(P.accent_btn_style(font_size=12, padding="6px 14px"))
        self.calibrate_btn.clicked.connect(self._calibrate_selected)
        control_layout.addWidget(self.calibrate_btn)

        self.reset_zero_btn = QPushButton("Reset Zeros")
        self.reset_zero_btn.setFixedHeight(32)
        self.reset_zero_btn.setStyleSheet(P.warning_btn_style(font_size=12, padding="6px 14px"))
        self.reset_zero_btn.clicked.connect(self._reset_zeros)
        control_layout.addWidget(self.reset_zero_btn)

        status_frame = QFrame()
        status_frame.setStyleSheet(f"QFrame {{ background: {P.DARK_INPUT}; border: 1px solid {P.DARK_BORDER}; border-radius: 4px; }}")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(10, 4, 10, 4)

        self.status_led = QLabel("●")
        self.status_led.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        status_layout.addWidget(self.status_led)

        self.status_text = QLabel("Waiting for the robot to connect…")
        self.status_text.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; border: none; background: transparent;")
        status_layout.addWidget(self.status_text)
        status_layout.addStretch()

        self.latency_label = QLabel("-- ms")
        self.latency_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
        status_layout.addWidget(self.latency_label)
        control_layout.addWidget(status_frame, 1)

        layout.addWidget(control_frame)
        layout.addStretch()

    def _toggle_polling(self):
        """Start/stop automatic encoder polling."""
        self._polling_active = not self._polling_active
        if self._polling_active and self._connection_manager and self._connection_manager.is_connected:
            self._poll_timer.start(self._poll_interval)
            self.toggle_poll_btn.setText("Pause")
            self.poll_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
        else:
            self._poll_timer.stop()
            self.toggle_poll_btn.setText("Resume")
            self.poll_indicator.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; background: transparent; border: none;")

    def _request_poll(self):
        """Timer callback — request encoder data."""
        self._polling_active = True
        if self._connection_manager and self._connection_manager.is_connected:
            self._connection_manager.poll_encoders()

    def _on_encoder_data(self, encoder_values: list):
        """Handle incoming encoder data from protocol."""
        for i, bar in enumerate(self._bars):
            if i < len(encoder_values):
                bar.set_value(encoder_values[i], int(encoder_values[i] * 100))

        self.status_led.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
        self.status_text.setText("Receiving encoder data")
        self.latency_label.setText("ok")

        if not self._polling_active:
            self._polling_active = True
            self._poll_timer.start(self._poll_interval)
            self.toggle_poll_btn.setText("Pause")
            self.poll_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")

    def _on_poll_rate_changed(self, ms: int):
        """Update polling interval."""
        self._poll_interval = ms
        if self._poll_timer.isActive():
            self._poll_timer.setInterval(ms)
        self.poll_rate_changed.emit(ms)
        if self._connection_manager:
            self._connection_manager.set_poll_interval(ms)

    def _calibrate_selected(self):
        """Set current encoder values as zero offsets."""
        if self._connection_manager:
            nite = self._connection_manager.nite_protocol
            if nite:
                for i in range(6):
                    nite.calibrate_zero(i)
                    self._bars[i].set_calibrated(True)
                self.status_text.setText("Zero calibration set for all joints")

    def _reset_zeros(self):
        """Reset all zero offsets."""
        if self._connection_manager:
            nite = self._connection_manager.nite_protocol
            if nite:
                nite.reset_zero_offsets()
                for bar in self._bars:
                    bar.set_calibrated(False)
                self.status_text.setText("All zero offsets have been cleared")

    def set_connected(self, connected: bool):
        """Update UI for connection state."""
        if connected:
            self.status_led.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
            self.status_text.setText("Connected — press Start Poll to read encoders")
            # Do NOT auto-start polling at 100ms: that floods the master with
            # #P and starves moves. The connection manager polls status at 1s;
            # the user starts encoder polling explicitly with the button.
            self._polling_active = False
            self._poll_timer.stop()
            self.toggle_poll_btn.setText("Start Poll")
            self.poll_indicator.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; background: transparent; border: none;")
        else:
            self.status_led.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
            self.status_text.setText("Disconnected")
            self._poll_timer.stop()
            self._polling_active = False
            self.toggle_poll_btn.setText("Start Poll")
            self.poll_indicator.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; background: transparent; border: none;")
            for bar in self._bars:
                bar.set_value(0.0)

    def update_status(self, message: str, level: str = "info"):
        """Update the status text."""
        colors = {"info": P.DARK_TEXT_DIM, "success": P.DARK_SUCCESS, "error": P.DARK_ERROR, "warning": P.DARK_WARNING}
        self.status_text.setStyleSheet(f"color: {colors.get(level, P.DARK_TEXT_DIM)}; font-size: 12px; border: none; background: transparent;")
        self.status_text.setText(message)
