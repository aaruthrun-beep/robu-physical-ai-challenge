"""Raw Nite369 command console panel for Astra Studio."""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTextEdit,
    QLineEdit, QPushButton, QLabel, QComboBox
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor

from . import palette as P


class ConsolePanel(QWidget):
    """Send raw Nite369 commands and view responses."""

    command_sent = pyqtSignal(str)

    PRESETS = [
        ("--- Link / Status ---", None),
        ("Ping slaves (PING)", "PING"),
        ("Firmware version (V)", "V"),
        ("Position report (M114)", "M114"),
        ("Steps/deg report (M503)", "M503"),
        ("Read limit switches (L)", "L"),
        ("", None),
        ("--- Motion ---", None),
        ("G-code jog J1 +10° (G1 X10)", "G1 X10 F1000"),
        ("G-code jog J2 +10° (G1 Y10)", "G1 Y10 F1000"),
        ("G-code jog J3 +10° (G1 Z10)", "G1 Z10 F1000"),
        ("Relative mode (G91)", "G91"),
        ("Absolute mode (G90)", "G90"),
        ("Disable motors (M18)", "M18"),
        ("Halt all (#H)", "H"),
        ("", None),
        ("--- Config ---", None),
        ("Read J1 config (CFG1)", "CFG1"),
        ("Read J2 config (CFG2)", "CFG2"),
        ("Read J3 config (CFG3)", "CFG3"),
        ("Read J4 config (CFG4)", "CFG4"),
        ("Read J5 config (CFG5)", "CFG5"),
        ("Read J6 config (CFG6)", "CFG6"),
        ("Read J1 profile (CR1)", "CR1"),
        ("Write J1 profile (CF1)", "CF1,2000,500,500"),
        ("Save config to flash (CS)", "CS"),
        ("Reload config (M501)", "M501"),
        ("Reset to defaults (CFGRESET)", "CFGRESET"),
        ("", None),
        ("--- Homing ---", None),
        ("Home all (HM0)", "HM0"),
        ("Read homing cfg J1 (HG1)", "HG1"),
        ("", None),
        ("--- Gripper ---", None),
        ("Gripper open (G200)", "G200"),
        ("Gripper close (G-200)", "G-200"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._setup_ui()

    def set_connection_manager(self, cm):
        self._connection_manager = cm

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        title = QLabel("Console")
        title.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        layout.addWidget(title)

        # Preset commands
        preset_row = QHBoxLayout()
        self._preset_combo = QComboBox()
        self._preset_combo.setFixedHeight(26)
        self._preset_combo.setStyleSheet(P.input_style(font_size=10))
        for label, cmd in self.PRESETS:
            if cmd is None and label:
                self._preset_combo.insertSeparator(self._preset_combo.count())
            elif label:
                self._preset_combo.addItem(label, cmd)
        self._preset_combo.currentIndexChanged.connect(self._on_preset_selected)
        preset_row.addWidget(self._preset_combo, 1)

        self._send_preset_btn = QPushButton("Send")
        self._send_preset_btn.setFixedHeight(26)
        self._send_preset_btn.setFixedWidth(60)
        self._send_preset_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=10))
        self._send_preset_btn.clicked.connect(self._send_preset)
        preset_row.addWidget(self._send_preset_btn)
        layout.addLayout(preset_row)

        # Response log
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFont(QFont("Consolas", 9))
        self._log.setStyleSheet(f"background: {P.DARK_INPUT}; color: {P.DARK_TEXT}; border: 1px solid {P.DARK_BORDER}; border-radius: 3px;")
        self._log.setMaximumHeight(240)
        layout.addWidget(self._log)

        # Manual command input
        cmd_row = QHBoxLayout()
        self._cmd_input = QLineEdit()
        self._cmd_input.setPlaceholderText("Type command: #MV1,1000,1600")
        self._cmd_input.setFont(QFont("Consolas", 10))
        self._cmd_input.setStyleSheet(P.input_style(font_size=10))
        self._cmd_input.returnPressed.connect(self._send_manual)
        cmd_row.addWidget(self._cmd_input, 1)

        self._send_btn = QPushButton("Send Command")
        self._send_btn.setFixedHeight(28)
        self._send_btn.setFixedWidth(110)
        self._send_btn.setStyleSheet(P.btn_style(P.DARK_ACCENT, text=P.DARK_BG, font_size=10))
        self._send_btn.clicked.connect(self._send_manual)
        cmd_row.addWidget(self._send_btn)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.setFixedHeight(28)
        self._clear_btn.setFixedWidth(60)
        self._clear_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=10))
        self._clear_btn.clicked.connect(self._log.clear)
        cmd_row.addWidget(self._clear_btn)
        layout.addLayout(cmd_row)

    def _on_preset_selected(self, idx):
        cmd = self._preset_combo.currentData()
        if cmd:
            self._cmd_input.setText(cmd)

    def _send_preset(self):
        cmd = self._preset_combo.currentData()
        if cmd:
            self._send(cmd)

    def _send_manual(self):
        cmd = self._cmd_input.text().strip()
        if cmd:
            self._send(cmd)

    def _send(self, cmd):
        # Ensure # prefix
        if not cmd.startswith('#'):
            cmd = '#' + cmd

        self._log.append(f"> {cmd}")
        self.command_sent.emit(cmd)

        if self._connection_manager and self._connection_manager.is_connected:
            try:
                resp = self._connection_manager.send_command(cmd)
                if resp:
                    self._log.append(f"< {resp}")
            except Exception as e:
                self._log.append(f"< Error: {e}")
        else:
            self._log.append("< — not connected —")

        # Auto-scroll
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())
