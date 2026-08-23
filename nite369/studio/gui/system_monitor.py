"""Professional system monitoring dashboard for Nite 369.

Displays:
- Overall robot state with color-coded status
- Joint position comparison (encoder vs commanded)
- Communication health metrics
- Scrollable error log with acknowledge
"""

from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFrame, QGridLayout, QGroupBox, QTextEdit, QListWidget,
    QListWidgetItem, QScrollArea,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont, QTextCursor

from . import palette as P


class SystemMonitorPanel(QWidget):
    """System health and status dashboard for Nite 369."""

    ERROR_TYPES = {
        "info": P.DARK_TEXT_MUTED,
        "success": P.DARK_SUCCESS,
        "warning": P.DARK_WARNING,
        "error": P.DARK_ERROR,
        "alarm": P.DARK_ERROR,
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._event_history = []
        self._max_events = 100
        self._error_count = 0
        self._packet_count = 0
        self._setup_ui()
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._update_display)
        self._refresh_timer.start(1000)

    def set_connection_manager(self, cm):
        self._connection_manager = cm
        if cm:
            cm.connectionStateChanged.connect(self._on_connection_state)
            cm.errorOccurred.connect(lambda msg: self.add_event("error", msg))
            cm.messageReceived.connect(self._on_message)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # ── Header ──────────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel("System Monitor")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()

        self.conn_indicator = QLabel("● Disconnected")
        self.conn_indicator.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(self.conn_indicator)

        self.clear_events_btn = QPushButton("Clear Log")
        self.clear_events_btn.setFixedHeight(28)
        self.clear_events_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11))
        self.clear_events_btn.clicked.connect(self._clear_events)
        header.addWidget(self.clear_events_btn)
        layout.addLayout(header)

        # ── Status Grid ─────────────────────────────────────────
        status_grid = QGridLayout()
        status_grid.setSpacing(6)

        def make_frame():
            frame = QFrame()
            frame.setStyleSheet(f"QFrame {{ border: 1px solid {P.DARK_BORDER}; border-radius: 4px; background: {P.DARK_PANEL}; }}")
            f_layout = QVBoxLayout(frame)
            f_layout.setContentsMargins(10, 6, 10, 6)
            f_layout.setSpacing(2)
            return frame, f_layout

        # Robot state
        state_frame, state_layout = make_frame()
        state_layout.addWidget(QLabel("Robot State"))
        self.state_label = QLabel("● Idle")
        self.state_label.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 15px; font-weight: bold; border: none; background: transparent;")
        state_layout.addWidget(self.state_label)
        self.state_sub = QLabel("Ready")
        self.state_sub.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; border: none; background: transparent;")
        state_layout.addWidget(self.state_sub)
        status_grid.addWidget(state_frame, 0, 0)

        # Communication
        comm_frame, comm_layout = make_frame()
        comm_layout.addWidget(QLabel("Communication"))
        self.packets_label = QLabel("0 packets/s")
        self.packets_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 15px; font-weight: bold; border: none; background: transparent;")
        comm_layout.addWidget(self.packets_label)
        self.latency_label = QLabel("Latency: -- ms")
        self.latency_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; border: none; background: transparent;")
        comm_layout.addWidget(self.latency_label)
        status_grid.addWidget(comm_frame, 0, 1)

        # Errors
        error_frame, error_layout = make_frame()
        error_layout.addWidget(QLabel("Errors"))
        self.error_count_label = QLabel("0")
        self.error_count_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 15px; font-weight: bold; border: none; background: transparent;")
        error_layout.addWidget(self.error_count_label)
        self.last_error_label = QLabel("No errors")
        self.last_error_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; border: none; background: transparent;")
        self.last_error_label.setWordWrap(True)
        error_layout.addWidget(self.last_error_label)
        status_grid.addWidget(error_frame, 0, 2)

        # Equalize column widths
        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(1, 1)
        status_grid.setColumnStretch(2, 1)

        layout.addLayout(status_grid)

        # ── Joint Position Error (Encoder vs Commanded) ─────────
        position_group = QGroupBox("Joint Position (Commanded → Encoder)")
        position_group.setStyleSheet(P.groupbox_style(font_size=11))
        pos_layout = QGridLayout(position_group)
        pos_layout.setSpacing(4)

        headers = ["Joint", "Commanded", "Encoder", "Error", "Status"]
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; font-weight: bold; border: none; background: transparent;")
            pos_layout.addWidget(lbl, 0, col)

        self._pos_labels = []
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        for i, name in enumerate(joint_names):
            row = i + 1
            name_lbl = QLabel(name)
            name_lbl.setFixedWidth(28)
            name_lbl.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
            pos_layout.addWidget(name_lbl, row, 0)

            cmd_lbl = QLabel("--°")
            cmd_lbl.setFixedWidth(64)
            cmd_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
            pos_layout.addWidget(cmd_lbl, row, 1)

            enc_lbl = QLabel("--°")
            enc_lbl.setFixedWidth(64)
            enc_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
            pos_layout.addWidget(enc_lbl, row, 2)

            err_lbl = QLabel("--°")
            err_lbl.setFixedWidth(64)
            err_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
            pos_layout.addWidget(err_lbl, row, 3)

            status_lbl = QLabel("—")
            status_lbl.setFixedWidth(48)
            status_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; font-weight: bold; border: none; background: transparent;")
            pos_layout.addWidget(status_lbl, row, 4)

            self._pos_labels.append((cmd_lbl, enc_lbl, err_lbl, status_lbl))

        layout.addWidget(position_group)

        # ── Event Log ───────────────────────────────────────────
        event_group = QGroupBox("Event Log")
        event_group.setStyleSheet(P.groupbox_style(font_size=11))
        event_layout = QVBoxLayout(event_group)
        event_layout.setContentsMargins(6, 6, 6, 6)

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumHeight(130)
        self.event_log.setStyleSheet(f"""
            QTextEdit {{
                background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
                font-family: 'Consolas', monospace; font-size: 11px;
                padding: 4px;
            }}
        """)
        event_layout.addWidget(self.event_log)
        layout.addWidget(event_group)

    def _on_connection_state(self, state: str):
        """Handle connection state changes."""
        if state == "connected":
            self.conn_indicator.setText("● Connected")
            self.conn_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
            self.add_event("success", "Connected to robot")
        elif state == "disconnected":
            self.conn_indicator.setText("● Disconnected")
            self.conn_indicator.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
            self.add_event("warning", "Disconnected from robot")
            self._error_count = 0
            self.error_count_label.setText("0")

    def _on_message(self, msg: str):
        """Process incoming messages."""
        self._packet_count += 1
        if msg.startswith(">ER"):
            self.add_event("error", msg[4:].strip())
        elif msg.startswith(">OK"):
            pass

    def _update_display(self):
        """Periodic display update — 1 Hz."""
        if self._connection_manager and self._connection_manager.is_connected:
            nite = self._connection_manager.nite_protocol
            if nite:
                # Update position comparison
                for i, (cmd_lbl, enc_lbl, err_lbl, status_lbl) in enumerate(self._pos_labels):
                    cmd = nite.commanded_positions[i] if i < len(nite.commanded_positions) else 0
                    enc = nite.encoder_values[i] if i < len(nite.encoder_values) else 0
                    error = cmd - enc

                    cmd_lbl.setText(f"{cmd:.1f}°")
                    enc_lbl.setText(f"{enc:.1f}°")
                    err_lbl.setText(f"{error:+.1f}°")

                    # Color-code error
                    abs_err = abs(error)
                    if abs_err < 0.5:
                        err_color = P.DARK_SUCCESS
                        status_lbl.setText("OK")
                        status_color = P.DARK_SUCCESS
                    elif abs_err < 2.0:
                        err_color = P.DARK_WARNING
                        status_lbl.setText("Warning")
                        status_color = P.DARK_WARNING
                    else:
                        err_color = P.DARK_ERROR
                        status_lbl.setText("Error")
                        status_color = P.DARK_ERROR
                    err_lbl.setStyleSheet(
                        f"color: {err_color}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
                    status_lbl.setStyleSheet(
                        f"color: {status_color}; font-size: 11px; font-weight: bold; border: none; background: transparent;")

                # Update state
                state_str = nite.state.value
                colors = {"idle": P.DARK_SUCCESS, "moving": P.DARK_ACCENT,
                          "homing": P.DARK_WARNING, "error": P.DARK_ERROR, "alarm": P.DARK_ERROR}
                color = colors.get(state_str, P.DARK_TEXT_MUTED)
                self.state_label.setStyleSheet(f"color: {color}; font-size: 14px; font-weight: bold; border: none; background: transparent;")
                self.state_label.setText(f"● {state_str.title()}")
                mask = nite.enabled_mask
                enabled_joints = [f"J{i+1}" for i in range(6) if mask & (1 << i)]
                self.state_sub.setText("Drivers enabled: " + (", ".join(enabled_joints) if enabled_joints else "none"))

                # Update packet count
                self.packets_label.setText(f"{self._packet_count} packets (total)")
                port = getattr(self._connection_manager, '_connected_port', '?')
                self.latency_label.setText(f"Connected to {port}")
        else:
            self.state_label.setText("● Disconnected")
            self.state_label.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 14px; font-weight: bold; border: none; background: transparent;")
            self.state_sub.setText("No connection")

    def add_event(self, level: str, message: str):
        """Add an event to the log."""
        ts = datetime.now().strftime("%H:%M:%S")
        color = self.ERROR_TYPES.get(level, P.DARK_TEXT_MUTED)

        cursor = self.event_log.textCursor()
        cursor.movePosition(QTextCursor.End)

        # Format timestamp
        fmt = cursor.charFormat()
        fmt.setForeground(QColor(P.DARK_TEXT_MUTED))
        cursor.insertText(f"[{ts}] ", fmt)

        # Format message
        fmt.setForeground(QColor(color))
        cursor.insertText(f"{message}\n", fmt)

        self.event_log.setTextCursor(cursor)
        self.event_log.ensureCursorVisible()

        # Count errors
        if level == "error":
            self._error_count += 1
            self.error_count_label.setText(str(self._error_count))
            self.error_count_label.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 14px; font-weight: bold; border: none; background: transparent;")
            short = message if len(message) <= 60 else message[:60] + "…"
            self.last_error_label.setText(short)
            self.last_error_label.setToolTip(message)

        # Keep in history
        self._event_history.append((ts, level, message))
        if len(self._event_history) > self._max_events:
            self._event_history.pop(0)

    def _clear_events(self):
        """Clear the event log."""
        self.event_log.clear()
        self._event_history.clear()
        self._error_count = 0
        self.error_count_label.setText("0")
        self.error_count_label.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 15px; font-weight: bold; border: none; background: transparent;")
        self.last_error_label.setText("No errors")
