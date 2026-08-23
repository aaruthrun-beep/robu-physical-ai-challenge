"""Professional TMC2209 stepper driver configuration panel.

Provides full register-level control of TMC2209 drivers on Slave 2 (wrist):
- J4 (Forearm Roll)  — addr 0x00
- J5 (Wrist Pitch)   — addr 0x01
- J6 (Wrist Roll)    — addr 0x02
- Gripper            — addr 0x03

Features:
- Tabbed driver selection with visual indicators
- IRUN/IHOLD current sliders with real-time readback
- Microstepping resolution selector (1 → 256)
- SpreadCycle / StealthChop mode toggle
- DRV_STATUS diagnostic panel with parsed error flags
- Enable/Disable per driver
- Configuration presets (save/load profiles)
- Raw register hex explorer
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QSlider, QSpinBox, QDoubleSpinBox, QFrame, QGridLayout,
    QGroupBox, QTabWidget, QComboBox, QCheckBox, QTextEdit,
    QLineEdit, QScrollArea, QMessageBox, QFileDialog,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QFont

from . import palette as P


# TMC2209 Register Addresses
TMC_REGISTERS = {
    0x00: "GCONF",
    0x01: "GSTAT",
    0x02: "IFCNT",
    0x03: "SLAVECONF",
    0x10: "IHOLD_IRUN",
    0x11: "TPOWERDOWN",
    0x13: "TPWMTHRS",
    0x22: "VACTUAL",
    0x6C: "CHOPCONF",
    0x6F: "DRV_STATUS",
}

# Microstep resolution lookup
MRES_TABLE = [
    ("Full Step", 0, 1),
    ("1/2", 1, 2),
    ("1/4", 2, 4),
    ("1/8", 3, 8),
    ("1/16", 4, 16),
    ("1/32", 5, 32),
    ("1/64", 6, 64),
    ("1/128", 7, 128),
    ("1/256", 8, 256),
]

# Configuration presets
TMC_PRESETS = {
    "1.0A Low Noise": {
        "gconf": 0x00C0,
        "chopconf": 0x00010053,
        "ihold_irun": 0x00040F10,
        "desc": "IRUN=16 (1.0A), IHOLD=4 (25%), 1/8 µsteps, SpreadCycle",
    },
    "0.75A Balanced": {
        "gconf": 0x00C0,
        "chopconf": 0x00010054,
        "ihold_irun": 0x00040B0C,
        "desc": "IRUN=12 (0.75A), IHOLD=4 (25%), 1/16 µsteps, SpreadCycle",
    },
    "0.5A Economy": {
        "gconf": 0x00C0,
        "chopconf": 0x00010055,
        "ihold_irun": 0x00030708,
        "desc": "IRUN=8 (0.5A), IHOLD=3 (20%), 1/32 µsteps, SpreadCycle",
    },
    "StealthChop Silent": {
        "gconf": 0x00C0,
        "chopconf": 0x00010053,
        "ihold_irun": 0x00040F10,
        "desc": "IRUN=16 (1.0A), IHOLD=4 (25%), 1/8 µsteps, StealthChop",
    },
    "High Torque": {
        "gconf": 0x00C0,
        "chopconf": 0x00010052,
        "ihold_irun": 0x00041014,
        "desc": "IRUN=20 (1.25A), IHOLD=5 (25%), 1/4 µsteps, SpreadCycle",
    },
}

# DRV_STATUS bit descriptions
DRV_STATUS_BITS = [
    (0, "Overtemp Shutdown", "red"),
    (1, "Overtemp Warning", "orange"),
    (2, "Short to GND", "red"),
    (3, "Short to VS (A)", "red"),
    (4, "Short to VS (B)", "red"),
    (5, "Open Load (A)", "orange"),
    (6, "Open Load (B)", "orange"),
    (7, "Die Temp > 150°C", "orange"),
    (31, "Standstill", "green"),
]


class TMCConfigPanel(QWidget):
    """Full TMC2209 stepper driver configuration interface."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._current_driver = 0  # 0-3
        self._selected_preset = ""
        self._drv_status = {}
        self._setup_ui()

    def set_connection_manager(self, cm):
        self._connection_manager = cm
        if cm:
            cm.tmcStatusUpdate.connect(self._on_tmc_status)
            cm.tmcRegisterRead.connect(self._on_tmc_reg_read)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # Root background — the global dark theme doesn't style plain QWidget,
        # so this panel was showing white. Scope to this panel only so child
        # widgets keep their own styles.
        self.setObjectName("tmcConfigPanel")
        self.setStyleSheet(
            f"QWidget#tmcConfigPanel {{ background: {P.DARK_BG}; }}")

        # ── Header ──────────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel("TMC2209 Driver Configuration")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()

        self.conn_status = QLabel("●")
        self.conn_status.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; background: transparent; border: none;")
        header.addWidget(self.conn_status)
        layout.addLayout(header)

        # ── Driver Tabs ─────────────────────────────────────────
        self.driver_tabs = QTabWidget()
        self.driver_tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {P.DARK_BORDER}; background: {P.DARK_PANEL};
            }}
            QTabBar::tab {{
                background: {P.DARK_BUTTON}; color: {P.DARK_TEXT_DIM};
                border: 1px solid {P.DARK_BORDER};
                padding: 6px 16px; font-size: 12px; font-weight: bold;
            }}
            QTabBar::tab:selected {{
                background: {P.DARK_PANEL}; color: {P.DARK_ACCENT};
                border-bottom: 2px solid {P.DARK_ACCENT};
            }}
            QTabBar::tab:hover:!selected {{
                background: {P.DARK_BUTTON_HOVER}; color: {P.DARK_TEXT};
            }}
        """)

        driver_names = ["J4 — Forearm Roll", "J5 — Wrist Pitch",
                        "J6 — Wrist Roll", "Gripper"]
        for i, name in enumerate(driver_names):
            tab = self._create_driver_tab(i)
            self.driver_tabs.addTab(tab, name)

        self.driver_tabs.currentChanged.connect(self._on_driver_changed)
        layout.addWidget(self.driver_tabs, 1)

        # ── Bottom Controls (two rows so nothing overlaps on narrow widths) ──
        bottom_frame = QFrame()
        bottom_frame.setStyleSheet(f"QFrame {{ border: 1px solid {P.DARK_BORDER}; border-radius: 4px; background: {P.DARK_PANEL}; }}")
        bottom_outer = QVBoxLayout(bottom_frame)
        bottom_outer.setContentsMargins(8, 6, 8, 6)
        bottom_outer.setSpacing(4)

        # Row 1: Apply to All + Read All
        row1 = QHBoxLayout()
        row1.setSpacing(6)
        self.apply_all_btn = QPushButton("Apply to All")
        self.apply_all_btn.setFixedHeight(32)
        self.apply_all_btn.setStyleSheet(P.accent_btn_style(font_size=12, padding="6px 14px"))
        self.apply_all_btn.clicked.connect(self._apply_to_all)
        row1.addWidget(self.apply_all_btn)

        self.refresh_all_btn = QPushButton("⟳ Read All")
        self.refresh_all_btn.setFixedHeight(32)
        self.refresh_all_btn.setStyleSheet(P.warning_btn_style(font_size=12, padding="6px 14px"))
        self.refresh_all_btn.clicked.connect(self._refresh_all)
        row1.addWidget(self.refresh_all_btn)
        row1.addStretch()
        bottom_outer.addLayout(row1)

        # Row 2: Preset selector + Apply Preset
        row2 = QHBoxLayout()
        row2.setSpacing(6)
        row2.addWidget(QLabel("Preset:"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItems([""] + list(TMC_PRESETS.keys()))
        self.preset_combo.setFixedWidth(170)
        self.preset_combo.setStyleSheet(P.input_style(font_size=12))
        self.preset_combo.currentTextChanged.connect(self._on_preset_selected)
        row2.addWidget(self.preset_combo)

        self.apply_preset_btn = QPushButton("Apply Preset")
        self.apply_preset_btn.setFixedHeight(32)
        self.apply_preset_btn.setStyleSheet(P.success_btn_style(font_size=12, padding="6px 12px"))
        self.apply_preset_btn.clicked.connect(self._apply_preset)
        row2.addWidget(self.apply_preset_btn)
        row2.addStretch()
        bottom_outer.addLayout(row2)

        layout.addWidget(bottom_frame)

        # ── TMC Activity Log (only TMC-related messages) ──────────
        log_frame = QFrame()
        log_frame.setStyleSheet(f"QFrame {{ border: 1px solid {P.DARK_BORDER}; border-radius: 4px; background: {P.DARK_PANEL}; }}")
        log_outer = QVBoxLayout(log_frame)
        log_outer.setContentsMargins(8, 4, 8, 6)
        log_outer.setSpacing(4)

        log_header = QHBoxLayout()
        log_title = QLabel("TMC Activity Log")
        log_title.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; background: transparent; border: none;")
        log_header.addWidget(log_title)
        log_header.addStretch()
        self.tmc_log_clear_btn = QPushButton("Clear")
        self.tmc_log_clear_btn.setFixedHeight(22)
        self.tmc_log_clear_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=9, padding="1px 8px"))
        self.tmc_log_clear_btn.clicked.connect(self._clear_tmc_log)
        log_header.addWidget(self.tmc_log_clear_btn)
        log_outer.addLayout(log_header)

        self.tmc_log = QTextEdit()
        self.tmc_log.setReadOnly(True)
        self.tmc_log.setMaximumHeight(140)
        self.tmc_log.setStyleSheet(f"""
            QTextEdit {{
                background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
                font-family: 'Consolas', 'Courier New', monospace; font-size: 11px;
            }}
        """)
        log_outer.addWidget(self.tmc_log)
        layout.addWidget(log_frame)

    def _create_driver_tab(self, driver_index: int) -> QWidget:
        """Create the configuration tab for a single TMC driver."""
        tab = QWidget()
        outer_layout = QVBoxLayout(tab)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        content = QWidget()
        # Match the dark tab pane — a plain QWidget here was white.
        content.setStyleSheet(f"QWidget {{ background: {P.DARK_PANEL}; }}")
        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        addr = driver_index
        tab.driver_addr = addr

        # ── Status Row ──────────────────────────────────────────
        status_row = QHBoxLayout()

        # Store enable buttons per-driver (fix: was shared `self.enable_btn`)
        enable_btn = QPushButton("ENABLE")
        enable_btn.setFixedHeight(30)
        enable_btn.setFixedWidth(96)
        enable_btn.setCheckable(True)
        enable_btn.setChecked(True)
        enable_btn.setStyleSheet(f"""
            QPushButton {{
                background: {P.DARK_SUCCESS}; color: #1a1a16; border: 1px solid {P.lighten(P.DARK_SUCCESS, 20)};
                border-radius: 4px; padding: 4px 12px; font-size: 12px; font-weight: bold;
            }}
            QPushButton:checked {{
                background: {P.DARK_ERROR}; color: white; border: 1px solid {P.lighten(P.DARK_ERROR, 20)};
            }}
            QPushButton:hover {{ background: {P.lighten(P.DARK_SUCCESS, 15)}; }}
            QPushButton:checked:hover {{ background: {P.lighten(P.DARK_ERROR, 15)}; }}
            QPushButton:pressed {{ background: {P.DARK_SUCCESS_DIM}; }}
            QPushButton:checked:pressed {{ background: {P.DARK_ERROR_DIM}; }}
        """)
        # Initialize per-addr enable button storage
        if not hasattr(self, '_enable_btns'):
            self._enable_btns = [None] * 4
        self._enable_btns[addr] = enable_btn
        enable_btn.toggled.connect(lambda checked, a=addr: self._toggle_enable(a, checked))
        status_row.addWidget(enable_btn)

        # Store status labels per-driver
        if not hasattr(self, '_drv_status_labels'):
            self._drv_status_labels = [None] * 4
        drv_label = QLabel("STATUS: —")
        drv_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold; border: none; background: transparent;")
        self._drv_status_labels[addr] = drv_label
        status_row.addWidget(drv_label)
        status_row.addStretch()

        if not hasattr(self, '_temp_labels'):
            self._temp_labels = [None] * 4
        temp_lbl = QLabel("Temp: --°C")
        temp_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; border: none; background: transparent;")
        self._temp_labels[addr] = temp_lbl
        status_row.addWidget(temp_lbl)

        if not hasattr(self, '_sg_labels'):
            self._sg_labels = [None] * 4
        sg_lbl = QLabel("StallGuard: —")
        sg_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
        sg_lbl.setToolTip("StallGuard load result — higher values mean more load")
        self._sg_labels[addr] = sg_lbl
        status_row.addWidget(sg_lbl)
        layout.addLayout(status_row)

        # ── Current Control ─────────────────────────────────────
        current_group = QGroupBox("Current Control")
        current_group.setStyleSheet(P.groupbox_style())
        current_layout = QGridLayout(current_group)
        current_layout.setSpacing(6)

        # IRUN
        current_layout.addWidget(QLabel("IRUN (Run Current):"), 0, 0)
        irun_slider = QSlider(Qt.Horizontal)
        irun_slider.setRange(0, 31)
        irun_slider.setValue(16)
        irun_slider.setStyleSheet(P.slider_style())
        self._irun_sliders = getattr(self, '_irun_sliders', [None]*4)
        self._irun_sliders[addr] = irun_slider
        current_layout.addWidget(irun_slider, 0, 1)
        current_layout.setColumnStretch(1, 1)

        irun_val = QLabel("16 (1.00A)")
        irun_val.setFixedWidth(90)
        irun_val.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
        current_layout.addWidget(irun_val, 0, 2)
        irun_slider.valueChanged.connect(lambda v, l=irun_val: l.setText(f"{v} ({v*0.0625:.2f}A)"))

        # IHOLD
        current_layout.addWidget(QLabel("IHOLD (Hold Current):"), 1, 0)
        ihold_slider = QSlider(Qt.Horizontal)
        ihold_slider.setRange(0, 31)
        ihold_slider.setValue(8)
        ihold_slider.setStyleSheet(P.slider_style())
        self._ihold_sliders = getattr(self, '_ihold_sliders', [None]*4)
        self._ihold_sliders[addr] = ihold_slider
        current_layout.addWidget(ihold_slider, 1, 1)

        ihold_val = QLabel("8 (0.50A)")
        ihold_val.setFixedWidth(90)
        ihold_val.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-family: 'Consolas', monospace; border: none; background: transparent;")
        current_layout.addWidget(ihold_val, 1, 2)
        ihold_slider.valueChanged.connect(lambda v, l=ihold_val: l.setText(f"{v} ({v*0.0625:.2f}A)"))

        apply_current_btn = QPushButton("Apply Current")
        apply_current_btn.setFixedHeight(30)
        apply_current_btn.setStyleSheet(P.accent_btn_style(font_size=11, padding="4px 12px"))
        apply_current_btn.clicked.connect(lambda: self._apply_current(addr))
        current_layout.addWidget(apply_current_btn, 2, 0, 1, 3)

        # Store refs for this tab
        self._irun_val_labels = getattr(self, '_irun_val_labels', [])
        while len(self._irun_val_labels) <= addr:
            self._irun_val_labels.append(None)
        self._irun_val_labels[addr] = irun_val
        self._ihold_val_labels = getattr(self, '_ihold_val_labels', [])
        while len(self._ihold_val_labels) <= addr:
            self._ihold_val_labels.append(None)
        self._ihold_val_labels[addr] = ihold_val

        layout.addWidget(current_group)

        # ── Microstepping & Mode ────────────────────────────────
        config_group = QGroupBox("Step Configuration")
        config_group.setStyleSheet(P.groupbox_style())
        config_layout = QGridLayout(config_group)
        config_layout.setSpacing(6)

        config_layout.addWidget(QLabel("Microsteps:"), 0, 0)
        mres_combo = QComboBox()
        for name, mres, steps in MRES_TABLE:
            mres_combo.addItem(f"1/{steps}", mres)
        mres_combo.setCurrentIndex(3)  # Default: 1/8
        mres_combo.setStyleSheet(P.input_style(font_size=12))
        self._mres_combos = getattr(self, '_mres_combos', [None]*4)
        self._mres_combos[addr] = mres_combo
        config_layout.addWidget(mres_combo, 0, 1)

        if not hasattr(self, '_interp_checks'):
            self._interp_checks = [None] * 4
        interp_check = QCheckBox("256x Interpolation")
        interp_check.setChecked(True)
        interp_check.setToolTip("Enables 256x microstep interpolation for smoother motion")
        interp_check.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 12px; spacing: 6px; background: transparent; border: none;")
        self._interp_checks[addr] = interp_check
        config_layout.addWidget(interp_check, 0, 2)

        config_layout.addWidget(QLabel("Mode:"), 1, 0)
        mode_combo = QComboBox()
        mode_combo.addItems(["SpreadCycle", "StealthChop"])
        mode_combo.setCurrentIndex(0)
        mode_combo.setStyleSheet(P.input_style(font_size=12))
        self._mode_combos = getattr(self, '_mode_combos', [None]*4)
        self._mode_combos[addr] = mode_combo
        config_layout.addWidget(mode_combo, 1, 1)

        if not hasattr(self, '_toff_spins'):
            self._toff_spins = [None] * 4
        toff_spin = QSpinBox()
        toff_spin.setRange(1, 15)
        toff_spin.setValue(3)
        toff_spin.setStyleSheet(P.input_style(font_size=12))
        toff_spin.setToolTip("Off-time setting (t_off) for the chopper — higher values run quieter")
        self._toff_spins[addr] = toff_spin
        config_layout.addWidget(QLabel("Off Time:"), 1, 2)
        config_layout.addWidget(toff_spin, 1, 3)

        apply_step_btn = QPushButton("Apply Step Config")
        apply_step_btn.setFixedHeight(30)
        apply_step_btn.setStyleSheet(P.accent_btn_style(font_size=11, padding="4px 12px"))
        apply_step_btn.clicked.connect(lambda: self._apply_step_config(addr))
        config_layout.addWidget(apply_step_btn, 2, 0, 1, 4)

        layout.addWidget(config_group)

        # ── Diagnostics ─────────────────────────────────────────
        diag_group = QGroupBox("DRV_STATUS Diagnostics")
        diag_group.setStyleSheet(P.groupbox_style())
        diag_layout = QVBoxLayout(diag_group)
        diag_layout.setSpacing(4)

        if not hasattr(self, '_diag_labels'):
            self._diag_labels = [None] * 4
        diag_text = QTextEdit()
        diag_text.setReadOnly(True)
        diag_text.setMaximumHeight(90)
        diag_text.setStyleSheet(f"""
            QTextEdit {{
                background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 3px;
                font-family: 'Consolas', monospace; font-size: 12px;
                padding: 6px;
            }}
        """)
        diag_text.setText("DRV_STATUS: --\nNo data yet — press Read DRV_STATUS")
        self._diag_labels[addr] = diag_text
        diag_layout.addWidget(diag_text)

        read_status_btn = QPushButton("Read DRV_STATUS")
        read_status_btn.setFixedHeight(30)
        read_status_btn.setStyleSheet(P.warning_btn_style(font_size=12, padding="4px 12px"))
        read_status_btn.clicked.connect(lambda: self._read_status(addr))
        diag_layout.addWidget(read_status_btn)

        layout.addWidget(diag_group)

        # ── Register Explorer ───────────────────────────────────
        reg_group = QGroupBox("Register Explorer (Hex)")
        reg_group.setStyleSheet(P.groupbox_style())
        reg_outer = QVBoxLayout(reg_group)
        reg_outer.setSpacing(6)

        # Row 1: register selector + value
        reg_row1 = QHBoxLayout()
        reg_row1.setSpacing(6)
        reg_row1.addWidget(QLabel("Register:"))
        reg_combo = QComboBox()
        for addr_val, name in sorted(TMC_REGISTERS.items()):
            reg_combo.addItem(f"0x{addr_val:02X} {name}", addr_val)
        reg_combo.setStyleSheet(P.input_style(font_size=12))
        self._reg_combos = getattr(self, '_reg_combos', [None]*4)
        self._reg_combos[addr] = reg_combo
        reg_row1.addWidget(reg_combo, 1)

        reg_row1.addWidget(QLabel("Value (hex):"))
        reg_val_input = QLineEdit("00000000")
        reg_val_input.setMaximumWidth(120)
        reg_val_input.setStyleSheet(P.input_style(font_size=12))
        self._reg_val_inputs = getattr(self, '_reg_val_inputs', [None]*4)
        self._reg_val_inputs[addr] = reg_val_input
        reg_row1.addWidget(reg_val_input)
        reg_outer.addLayout(reg_row1)

        # Row 2: Read / Write buttons
        reg_row2 = QHBoxLayout()
        reg_row2.setSpacing(6)
        read_reg_btn = QPushButton("Read")
        read_reg_btn.setFixedHeight(30)
        read_reg_btn.setStyleSheet(P.accent_btn_style(font_size=12, padding="4px 12px"))
        read_reg_btn.clicked.connect(lambda: self._read_register(addr))
        reg_row2.addWidget(read_reg_btn)

        write_reg_btn = QPushButton("Write")
        write_reg_btn.setFixedHeight(30)
        write_reg_btn.setStyleSheet(P.success_btn_style(font_size=12, padding="4px 12px"))
        write_reg_btn.clicked.connect(lambda: self._write_register(addr))
        reg_row2.addWidget(write_reg_btn)
        reg_row2.addStretch()
        reg_outer.addLayout(reg_row2)

        layout.addWidget(reg_group)
        layout.addStretch()

        scroll.setWidget(content)
        outer_layout.addWidget(scroll)

        return tab

    def _clear_tmc_log(self):
        if hasattr(self, "tmc_log"):
            self.tmc_log.clear()

    def _tmc_log_line(self, msg, level="info"):
        """Append a TMC-only log line with timestamp + color."""
        from datetime import datetime
        if not hasattr(self, "tmc_log"):
            return
        ts = datetime.now().strftime("%H:%M:%S")
        colors = {
            "info": P.DARK_TEXT_DIM,
            "ok": P.DARK_SUCCESS,
            "error": P.DARK_ERROR,
            "command": P.DARK_ACCENT,
        }
        color = colors.get(level, P.DARK_TEXT_DIM)
        self.tmc_log.append(f'<span style="color:#888;">[{ts}]</span> '
                            f'<span style="color:{color};">{msg}</span>')
        # keep log bounded
        doc = self.tmc_log.document()
        if doc.blockCount() > 300:
            cursor = self.tmc_log.textCursor()
            cursor.select(cursor.Document)
            cursor.setPosition(0)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, 200)
            cursor.removeSelectedText()

    def _on_driver_changed(self, index: int):
        """Handle driver tab change."""
        self._current_driver = index

    def _on_tmc_status(self, driver_addr: int, status: dict):
        """Handle DRV_STATUS data from protocol."""
        if driver_addr < len(self._diag_labels) and self._diag_labels[driver_addr]:
            diag = self._diag_labels[driver_addr]
            lines = [f"DRV_STATUS: 0x{status['raw']:08X}"]
            for bit, desc, color in DRV_STATUS_BITS:
                if status.get({
                    0: "overtemp_shutdown", 1: "overtemp_warning",
                    2: "short_to_gnd", 3: "short_to_vs_a",
                    4: "short_to_vs_b", 5: "open_load_a",
                    6: "open_load_b", 7: "temp_high",
                    31: "standstill",
                }.get(bit, ""), False):
                    prefix = "[!]" if color == "orange" else "[X]"
                    lines.append(f"  {prefix} {desc}")
            if status.get("standstill"):
                lines.append("  [OK] Standstill (motor stopped)")
            lines.append(f"  Actual current: {status['cs_actual']}  StallGuard: {status['stallguard_result']}")
            diag.setText("\n".join(lines))

    def _on_tmc_reg_read(self, driver_addr: int, reg: int, value: int):
        """Handle register read response."""
        if driver_addr < len(self._reg_val_inputs) and self._reg_val_inputs[driver_addr]:
            self._reg_val_inputs[driver_addr].setText(f"{value:08X}")

    def _toggle_enable(self, addr: int, enabled: bool):
        """Toggle driver enable state."""
        if self._connection_manager:
            self._connection_manager.tmc_set_enabled(addr, enabled)
            btn = self._enable_btns[addr]
            if enabled:
                btn.setText("ENABLED")
            else:
                btn.setText("DISABLED")

    def _apply_current(self, addr: int):
        """Apply IRUN/IHOLD current settings to a driver."""
        if not self._connection_manager or not self._connection_manager.is_connected:
            return
        irun = self._irun_sliders[addr].value()
        ihold = self._ihold_sliders[addr].value()
        self._connection_manager.tmc_set_current(addr, irun, ihold)
        # Update status via per-addr label
        if hasattr(self, '_drv_status_labels') and addr < len(self._drv_status_labels):
            lbl = self._drv_status_labels[addr]
            if lbl:
                lbl.setText(f"Current: run {irun*0.0625:.2f}A, hold {ihold*0.0625:.2f}A")

    def _apply_step_config(self, addr: int):
        """Apply microstepping and mode settings."""
        if not self._connection_manager or not self._connection_manager.is_connected:
            return
        mres = self._mres_combos[addr].currentData()
        interp = self._interp_checks[addr].isChecked() if hasattr(self, '_interp_checks') and self._interp_checks[addr] else True
        spreadcycle = self._mode_combos[addr].currentText() == "SpreadCycle"
        ok1 = self._connection_manager.tmc_set_microsteps(addr, mres)
        ok2 = self._connection_manager.tmc_set_mode(addr, spreadcycle)
        self._tmc_log_line(
            f"DRV{addr}: microsteps={mres} mode={'SpreadCycle' if spreadcycle else 'StealthChop'} "
            f"{'OK' if (ok1 and ok2) else 'FAIL'}", "ok" if (ok1 and ok2) else "error")

    def _read_status(self, addr: int):
        """Read DRV_STATUS from a driver."""
        if self._connection_manager:
            status = self._connection_manager.tmc_read_drv_status(addr)
            if status and hasattr(self, '_drv_status_labels') and addr < len(self._drv_status_labels):
                lbl = self._drv_status_labels[addr]
                if lbl:
                    lbl.setText("Status read successfully")
                self._tmc_log_line(f"DRV{addr}: DRV_STATUS read 0x{status['raw']:08X}", "ok")
            else:
                self._tmc_log_line(f"DRV{addr}: DRV_STATUS read FAILED", "error")

    def _read_register(self, addr: int):
        """Read a register from a driver."""
        if not self._connection_manager or not self._connection_manager.is_connected:
            return
        reg = self._reg_combos[addr].currentData()
        value = self._connection_manager.tmc_read_register(addr, reg)
        if value is not None:
            self._reg_val_inputs[addr].setText(f"{value:08X}")
            self._tmc_log_line(f"DRV{addr}: reg 0x{reg:02X} read = 0x{value:08X}", "ok")
        else:
            self._tmc_log_line(f"DRV{addr}: reg 0x{reg:02X} read FAILED", "error")

    def _write_register(self, addr: int):
        """Write a register value to a driver."""
        if not self._connection_manager or not self._connection_manager.is_connected:
            return
        reg = self._reg_combos[addr].currentData()
        try:
            val_str = self._reg_val_inputs[addr].text().strip()
            value = int(val_str, 16)
            ok = self._connection_manager.tmc_write_register(addr, reg, value)
            self._tmc_log_line(f"DRV{addr}: reg 0x{reg:02X} write 0x{value:08X} "
                               f"{'OK' if ok else 'FAIL'}", "ok" if ok else "error")
        except ValueError:
            self._tmc_log_line(f"DRV{addr}: invalid hex value '{val_str}'", "error")

    def _apply_to_all(self):
        """Apply current tab's settings to all 4 drivers."""
        if not self._connection_manager or not self._connection_manager.is_connected:
            return
        current_addr = self._current_driver
        irun = self._irun_sliders[current_addr].value()
        ihold = self._ihold_sliders[current_addr].value()
        mres = self._mres_combos[current_addr].currentData()
        spreadcycle = self._mode_combos[current_addr].currentText() == "SpreadCycle"

        ok = True
        for addr in range(4):
            ok &= self._connection_manager.tmc_set_current(addr, irun, ihold)
            ok &= self._connection_manager.tmc_set_microsteps(addr, mres)
            ok &= self._connection_manager.tmc_set_mode(addr, spreadcycle)
        self._tmc_log_line(f"Apply-to-all (IRUN={irun} IHOLD={ihold} MRES={mres}) "
                           f"{'OK' if ok else 'FAIL'}", "ok" if ok else "error")

    def _refresh_all(self):
        """Read DRV_STATUS from all drivers."""
        if self._connection_manager:
            for addr in range(4):
                status = self._connection_manager.tmc_read_drv_status(addr)
                if status:
                    self._tmc_log_line(f"DRV{addr}: status refreshed 0x{status['raw']:08X}", "ok")
                else:
                    self._tmc_log_line(f"DRV{addr}: status refresh FAILED", "error")

    def _on_preset_selected(self, preset_name: str):
        """Handle preset selection from dropdown."""
        self._selected_preset = preset_name

    def _apply_preset(self):
        """Apply the selected configuration preset to all drivers."""
        if not self._selected_preset or not self._connection_manager:
            return
        preset = TMC_PRESETS.get(self._selected_preset)
        if not preset:
            return
        ok = True
        for addr in range(4):
            ok &= self._connection_manager.tmc_configure(
                addr,
                gconf=preset["gconf"],
                chopconf=preset["chopconf"],
                ihold_irun=preset["ihold_irun"],
            )
        self._tmc_log_line(f"Preset '{self._selected_preset}' applied {'OK' if ok else 'FAIL'}", "ok" if ok else "error")
        # Update status on all driver tabs
        if hasattr(self, '_drv_status_labels'):
            for addr in range(4):
                lbl = self._drv_status_labels[addr]
                if lbl:
                    lbl.setText(f"Applied preset: {self._selected_preset}")

    def _apply_full_step_default(self):
        """Set all wrist TMC drivers (J4/J5/J6/gripper) to 1/8 microstepping.

        Called automatically on connect. 1/8 (MRES=3) matches the firmware
        boot config and the 1600 steps/rev config — full step stalls the
        motors at high speed. This is software-side only; the drivers are
        re-configured over the UART every connection.
        """
        if not self._connection_manager or not self._connection_manager.is_connected:
            return
        ok = True
        for addr in range(4):
            ok &= self._connection_manager.tmc_set_microsteps(addr, 3)  # MRES=3 = 1/8
        self._tmc_log_line(f"Default applied: all drivers 1/8 microstep (MRES=3) "
                           f"{'OK' if ok else 'FAIL'}", "ok" if ok else "error")

    def set_connected(self, connected: bool):
        """Update connection state."""
        if connected:
            self.conn_status.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
            # Force full stepping as the default on every connect.
            self._apply_full_step_default()
        else:
            self.conn_status.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
            for addr in range(4):
                if self._diag_labels[addr]:
                    self._diag_labels[addr].setText("DRV_STATUS: --\nDisconnected")
