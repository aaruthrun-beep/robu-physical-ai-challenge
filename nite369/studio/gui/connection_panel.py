"""Connection panel for Nite 369 robot communication controls.

Provides Nite 369 Ethernet and Serial connection modes,
port selection with auto-detection, and Connect/Disconnect buttons.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QComboBox, QTabWidget, QLineEdit, QSpinBox, QGroupBox,
    QFormLayout, QFrame,
)
from PyQt5.QtCore import Qt, pyqtSignal

from . import palette as P


class ConnectionPanel(QWidget):
    """Connection controls for Nite 369 Ethernet and Serial protocols."""

    connect_requested = pyqtSignal(str, dict)
    disconnect_requested = pyqtSignal()

    def __init__(self, connection_manager=None, parent=None):
        super().__init__(parent)
        self.connection_manager = connection_manager
        self._setup_ui()

    def set_connection_manager(self, cm):
        self.connection_manager = cm
        if cm:
            cm.connectionStateChanged.connect(self._on_state_changed)
            cm.errorOccurred.connect(self._on_error)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # Status indicator row (compact)
        status_row = QHBoxLayout()
        status_row.setContentsMargins(0, 0, 0, 0)
        status_row.addStretch()
        self.status_indicator = QLabel("Disconnected")
        self.status_indicator.setStyleSheet(
            f"color: {P.DARK_ERROR}; font-size: 11px; font-weight: bold; "
            f"border: 1px solid {P.DARK_BORDER}; padding: 3px 10px; border-radius: 3px; background: {P.DARK_INPUT};"
        )
        status_row.addWidget(self.status_indicator)
        layout.addLayout(status_row)

        self.mode_tabs = QTabWidget()

        # ── Tab 1: Nite 369 (Ethernet) ─────────────────────────
        nite_eth_tab = QWidget()
        nite_eth_layout = QFormLayout(nite_eth_tab)
        nite_eth_layout.setContentsMargins(8, 8, 8, 8)
        nite_eth_layout.setSpacing(6)

        self.nite_host_input = QLineEdit("192.168.1.50")
        self.nite_host_input.setStyleSheet(P.input_style())
        nite_eth_layout.addRow("Host:", self.nite_host_input)

        self.nite_port_input = QSpinBox()
        self.nite_port_input.setRange(1, 65535)
        self.nite_port_input.setValue(23)
        self.nite_port_input.setStyleSheet(P.input_style())
        nite_eth_layout.addRow("Port:", self.nite_port_input)

        self.nite_timeout_input = QSpinBox()
        self.nite_timeout_input.setRange(1, 30)
        self.nite_timeout_input.setValue(5)
        self.nite_timeout_input.setStyleSheet(P.input_style())
        nite_eth_layout.addRow("Timeout (s):", self.nite_timeout_input)
        self.mode_tabs.addTab(nite_eth_tab, "Nite 369 (Ethernet)")

        # ── Tab 2: Nite 369 (Serial) ───────────────────────────
        nite_ser_tab = QWidget()
        nite_ser_layout = QFormLayout(nite_ser_tab)
        nite_ser_layout.setContentsMargins(8, 8, 8, 8)
        nite_ser_layout.setSpacing(6)

        port_row = QHBoxLayout()
        self.nite_port_combo = QComboBox()
        self.nite_port_combo.setMinimumWidth(120)
        self.nite_port_combo.setStyleSheet(P.input_style())
        port_row.addWidget(self.nite_port_combo, 1)
        self.refresh_btn = QPushButton("↻")
        self.refresh_btn.setFixedSize(28, 28)
        self.refresh_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=12, padding="0px"))
        self.refresh_btn.setToolTip("Refresh available ports")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        port_row.addWidget(self.refresh_btn)
        nite_ser_layout.addRow("Port:", port_row)

        self.nite_baud_combo = QComboBox()
        self.nite_baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "250000", "500000", "1000000"])
        self.nite_baud_combo.setCurrentText("115200")
        self.nite_baud_combo.setStyleSheet(P.input_style())
        nite_ser_layout.addRow("Baud:", self.nite_baud_combo)
        self.mode_tabs.addTab(nite_ser_tab, "Nite 369 (Serial)")

        layout.addWidget(self.mode_tabs)

        conn_row = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet(P.success_btn_style(font_size=12, padding="7px 18px"))
        self.connect_btn.clicked.connect(self._on_connect)
        conn_row.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setStyleSheet(P.danger_btn_style(font_size=12, padding="7px 18px"))
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        self.disconnect_btn.setEnabled(False)
        conn_row.addWidget(self.disconnect_btn)
        layout.addLayout(conn_row)

        self._refresh_ports()

    def _refresh_ports(self):
        self.nite_port_combo.clear()
        try:
            if self.connection_manager:
                ports = self.connection_manager.get_available_ports()
            else:
                from ..control.transports import SerialTransport
                ports = SerialTransport.available_ports()
        except Exception as e:
            self.status_indicator.setText("Port scan failed")
            self.status_indicator.setToolTip(f"Couldn't list serial ports: {e}")
            return
        for p in ports:
            display = f"{p['port']} — {p.get('description', '')}"
            self.nite_port_combo.addItem(display, p['port'])

    def _on_connect(self):
        mode = self.mode_tabs.currentIndex()
        if mode == 0:  # Nite 369 Ethernet
            host = self.nite_host_input.text()
            port = self.nite_port_input.value()
            timeout = self.nite_timeout_input.value()
            self.connect_requested.emit("nite_ethernet", {"host": host, "port": port, "timeout": timeout})
        else:  # Nite 369 Serial
            port = self.nite_port_combo.currentData()
            baud = int(self.nite_baud_combo.currentText())
            self.connect_requested.emit("nite_serial", {"port": port, "baud_rate": baud})

    def _on_disconnect(self):
        self.disconnect_requested.emit()

    def _on_state_changed(self, state):
        base = f"font-size: 11px; font-weight: bold; border: 1px solid; padding: 3px 10px; border-radius: 3px; background: {P.DARK_INPUT};"
        if state == "connected":
            self.status_indicator.setText("Connected")
            self.status_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; border-color: {P.DARK_SUCCESS}; {base}")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
        elif state == "disconnected":
            self.status_indicator.setText("Disconnected")
            self.status_indicator.setStyleSheet(f"color: {P.DARK_ERROR}; border-color: {P.DARK_ERROR}; {base}")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
        elif state == "error":
            self.status_indicator.setText("Connection Error")
            self.status_indicator.setStyleSheet(f"color: {P.DARK_WARNING}; border-color: {P.DARK_WARNING}; {base}")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)

    def _on_error(self, msg):
        self.status_indicator.setText(f"Error: {msg[:60]}{'…' if len(msg) > 60 else ''}")
        self.status_indicator.setToolTip(msg)
        self.status_indicator.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 11px; font-weight: bold;")
