"""Homing & Limits panel for Nite369.

Provides:
- Live limit-switch preview (all 6, from #L)
- Per-joint homing (Home button + status from #HQ)
- Home All + sequence homing (configurable order)
- Homing config per joint (search speed, creep speed, backoff, offset,
  invert limit/dir) via #HC
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QFrame, QGridLayout, QGroupBox, QSpinBox, QDoubleSpinBox,
    QCheckBox, QComboBox, QScrollArea, QMessageBox,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from . import palette as P

# Firmware homing states (homing.h)
HOME_STATE_NAMES = {
    0: "Idle",
    1: "Searching",
    2: "Backoff",
    3: "Creeping",
    4: "Homed",
    5: "Error",
}


class HomingPanel(QWidget):
    """Limit-switch preview + homing control for the 6-DOF arm."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._limit_states = [False] * 6
        self._home_status = [{} for _ in range(6)]
        self._sequence = list(range(1, 7))  # default 1..6
        self._sequence_running = False
        self._seq_index = 0
        self._home_buttons = []
        self._setup_ui()

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._poll_limits)
        # Limit polling is gated on panel visibility (see showEvent/hideEvent):
        # it must NOT run every 500ms in the background — each #L is 2 SPI
        # round-trips on the master and floods the worker queue, delaying
        # jog commands. Poll only while the panel is actually on screen.
        self._refresh_timer.setInterval(500)
        self._refresh_timer.setSingleShot(False)

        self._home_poll_timer = QTimer(self)
        self._home_poll_timer.timeout.connect(self._poll_home_status)

    def showEvent(self, event):
        super().showEvent(event)
        if self._connection_manager and self._connection_manager.is_connected:
            self._refresh_timer.start()
            self._poll_limits()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._refresh_timer.stop()

    def set_connection_manager(self, cm):
        self._connection_manager = cm
        if cm:
            cm.connectionStateChanged.connect(self._on_connection_state)
            self._on_connection_state(cm.is_connected)

    def _setup_ui(self):
        # Root background (plain QWidget defaults to white otherwise)
        self.setObjectName("homingPanel")
        self.setStyleSheet(f"QWidget#homingPanel {{ background: {P.DARK_BG}; }}")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(8)

        # ── Header ──────────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel("Homing & Limits")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()
        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
        header.addWidget(self.conn_status)
        outer.addLayout(header)

        # ── Limit switch preview ────────────────────────────────
        limits_group = QGroupBox("Limit Switch Preview")
        limits_group.setStyleSheet(P.groupbox_style(font_size=12))
        lim_layout = QGridLayout(limits_group)
        lim_layout.setSpacing(6)
        self._limit_indicators = []
        for i in range(6):
            name_lbl = QLabel(f"J{i+1}:")
            name_lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; background: transparent; border: none;")
            ind = QLabel("● Open")
            ind.setStyleSheet(self._indicator_style(False))
            ind.setAlignment(Qt.AlignCenter)
            self._limit_indicators.append(ind)
            lim_layout.addWidget(name_lbl, i // 3, (i % 3) * 2)
            lim_layout.addWidget(ind, i // 3, (i % 3) * 2 + 1)
        outer.addWidget(limits_group)

        # ── Homing buttons ──────────────────────────────────────
        btn_row = QHBoxLayout()
        self.home_all_btn = QPushButton("Home All (J1..J6)")
        self.home_all_btn.setStyleSheet(P.accent_btn_style(font_size=12, padding="6px 14px"))
        self.home_all_btn.clicked.connect(self._home_all)
        btn_row.addWidget(self.home_all_btn)

        self.run_seq_btn = QPushButton("Run Sequence")
        self.run_seq_btn.setStyleSheet(P.success_btn_style(font_size=12, padding="6px 14px"))
        self.run_seq_btn.clicked.connect(self._toggle_sequence)
        btn_row.addWidget(self.run_seq_btn)

        self.stop_btn = QPushButton("■ Stop")
        self.stop_btn.setStyleSheet(P.warning_btn_style(font_size=12, padding="6px 14px"))
        self.stop_btn.clicked.connect(self._stop_all)
        btn_row.addWidget(self.stop_btn)

        self.apply_cfg_btn = QPushButton("Apply Homing Config")
        self.apply_cfg_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11, padding="6px 12px"))
        self.apply_cfg_btn.clicked.connect(self._apply_homing_config)
        btn_row.addWidget(self.apply_cfg_btn)
        btn_row.addStretch()
        outer.addLayout(btn_row)

        # ── Sequence config ─────────────────────────────────────
        seq_row = QHBoxLayout()
        seq_row.addWidget(QLabel("Homing sequence:"))
        self.seq_combo = QComboBox()
        self.seq_combo.setStyleSheet(P.input_style(font_size=11))
        self.seq_combo.setMinimumWidth(200)
        self._update_seq_combo()
        seq_row.addWidget(self.seq_combo)
        seq_row.addStretch()
        outer.addLayout(seq_row)

        # ── Per-joint homing config + control ───────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        content = QWidget()
        content.setStyleSheet(f"QWidget {{ background: {P.DARK_BG}; }}")
        grid = QGridLayout(content)
        grid.setSpacing(6)

        headers = ["Joint", "Status", "Homed", "Search spd", "Creep spd",
                   "Backoff", "Offset", "Inv Lim", "Inv Dir", ""]
        for c, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 11px; font-weight: bold; background: transparent; border: none;")
            grid.addWidget(lbl, 0, c)

        self._joint_rows = []
        for i in range(6):
            row = self._build_joint_row(i)
            self._joint_rows.append(row)
            for c, w in enumerate(row):
                grid.addWidget(w, i + 1, c)

        scroll.setWidget(content)
        outer.addWidget(scroll, 1)

    def _indicator_style(self, triggered):
        color = P.DARK_ERROR if triggered else P.DARK_TEXT_MUTED
        return (f"color: {color}; font-size: 13px; font-weight: bold; "
                f"background: {P.DARK_INPUT}; border: 1px solid {P.DARK_BORDER}; "
                f"border-radius: 4px; padding: 4px;")

    def _build_joint_row(self, idx):
        """Build one row of widgets for joint idx (0-5). Returns list."""
        name = QLabel(f"J{idx+1}")
        name.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 12px; font-weight: bold; background: transparent; border: none;")

        status = QLabel("Idle")
        status.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; background: transparent; border: none;")

        homed = QLabel("No")
        homed.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")

        search = QSpinBox()
        search.setRange(50, 50000)
        search.setValue(1000)
        search.setSingleStep(100)
        search.setStyleSheet(P.input_style(font_size=11))
        search.setFixedWidth(80)

        creep = QSpinBox()
        creep.setRange(10, 10000)
        creep.setValue(100)
        creep.setSingleStep(10)
        creep.setStyleSheet(P.input_style(font_size=11))
        creep.setFixedWidth(80)

        backoff = QSpinBox()
        backoff.setRange(0, 50000)
        backoff.setValue(200)
        backoff.setStyleSheet(P.input_style(font_size=11))
        backoff.setFixedWidth(80)

        offset = QDoubleSpinBox()
        offset.setRange(-360.0, 360.0)
        offset.setDecimals(2)
        offset.setValue(0.0)
        offset.setStyleSheet(P.input_style(font_size=11))
        offset.setFixedWidth(90)

        inv_lim = QCheckBox()
        inv_lim.setToolTip("Invert limit switch polarity (NC switch = checked)")
        inv_lim.setChecked(True)

        inv_dir = QCheckBox()
        inv_dir.setToolTip("Invert homing search direction")

        home_btn = QPushButton("Home")
        home_btn.setStyleSheet(P.accent_btn_style(font_size=10, padding="2px 10px"))
        home_btn.clicked.connect(lambda checked, i=idx: self._home_joint(i))

        widgets = [name, status, homed, search, creep, backoff, offset,
                   inv_lim, inv_dir, home_btn]
        self._home_buttons.append(home_btn)
        return widgets

    def _update_seq_combo(self):
        self.seq_combo.clear()
        self.seq_combo.addItem("Default (J1→J2→J3→J4→J5→J6)")
        self.seq_combo.addItem("J2→J1→J3→J5→J4→J6")
        self.seq_combo.addItem("J3→J2→J1→J6→J5→J4")
        self.seq_combo.addItem("Reverse (J6→J1)")

    # ── Connection ──────────────────────────────────────────────

    def _on_connection_state(self, connected):
        if connected:
            self.conn_status.setText("Connected")
            self.conn_status.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 12px; background: transparent; border: none;")
            # Start polling only if the panel is actually visible.
            if self.isVisible():
                self._refresh_timer.start()
                self._poll_limits()
        else:
            self.conn_status.setText("Disconnected")
            self.conn_status.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
            self._refresh_timer.stop()

    # ── Limit preview ───────────────────────────────────────────

    def _poll_limits(self):
        cm = self._connection_manager
        if not cm or not cm.is_connected:
            return
        states = cm.read_limits()
        if states is None:
            return
        self._limit_states = states
        for i, ind in enumerate(self._limit_indicators):
            trig = states[i] if i < len(states) else False
            ind.setText("● Triggered" if trig else "● Open")
            ind.setStyleSheet(self._indicator_style(trig))

    # ── Homing actions ──────────────────────────────────────────

    def _home_joint(self, idx):
        cm = self._connection_manager
        if not cm or not cm.is_connected:
            return
        ok = cm.home_joint(idx + 1)
        if ok:
            self._joint_rows[idx][1].setText("Searching...")
            if not self._home_poll_timer.isActive():
                self._home_poll_timer.start(250)
        else:
            self._joint_rows[idx][1].setText("FAILED")
            self._joint_rows[idx][1].setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 11px; background: transparent; border: none;")

    def _home_all(self):
        cm = self._connection_manager
        if not cm or not cm.is_connected:
            return
        if cm.home_all():
            for row in self._joint_rows:
                row[1].setText("Searching...")
            if not self._home_poll_timer.isActive():
                self._home_poll_timer.start(250)

    def _stop_all(self):
        cm = self._connection_manager
        if cm:
            cm.stop()
        self._sequence_running = False
        self._seq_index = 0
        self.run_seq_btn.setText("Run Sequence")
        if self._home_poll_timer.isActive():
            self._home_poll_timer.stop()

    def _toggle_sequence(self):
        if self._sequence_running:
            self._sequence_running = False
            self._seq_index = 0
            self.run_seq_btn.setText("Run Sequence")
            return
        sel = self.seq_combo.currentIndex()
        if sel == 0:
            self._sequence = [1, 2, 3, 4, 5, 6]
        elif sel == 1:
            self._sequence = [2, 1, 3, 5, 4, 6]
        elif sel == 2:
            self._sequence = [3, 2, 1, 6, 5, 4]
        else:
            self._sequence = [6, 5, 4, 3, 2, 1]
        self._sequence_running = True
        self._seq_index = 0
        self.run_seq_btn.setText("Stop Sequence")
        self._step_sequence()

    def _step_sequence(self):
        """Home the next joint in the sequence; poll until done."""
        if not self._sequence_running:
            return
        if self._seq_index >= len(self._sequence):
            self._sequence_running = False
            self._seq_index = 0
            self.run_seq_btn.setText("Run Sequence")
            return
        joint = self._sequence[self._seq_index]
        cm = self._connection_manager
        if not cm or not cm.is_connected:
            self._sequence_running = False
            return
        cm.home_joint(joint)
        self._joint_rows[joint - 1][1].setText("Searching...")
        # poll homing status until found, then advance
        self._home_poll_timer.start(200)

    def _poll_home_status(self):
        """Poll #HQ for all joints. Drives the sequence state machine."""
        cm = self._connection_manager
        if not cm or not cm.is_connected:
            return
        for i in range(6):
            st = cm.home_status(i + 1)
            if st is None:
                continue
            self._home_status[i] = st
            status_lbl = self._joint_rows[i][1]
            name = HOME_STATE_NAMES.get(st.get("state", 0), "?")
            homed = st.get("homed", False)
            status_lbl.setText(name)
            color = P.DARK_SUCCESS if homed else (
                P.DARK_WARNING if name in ("Searching", "Backoff", "Creeping") else P.DARK_TEXT_DIM)
            status_lbl.setStyleSheet(f"color: {color}; font-size: 11px; background: transparent; border: none;")
            self._joint_rows[i][2].setText("Yes" if homed else "No")
            self._joint_rows[i][2].setStyleSheet(
                f"color: {P.DARK_SUCCESS if homed else P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")

        # sequence progression
        if self._sequence_running and self._seq_index < len(self._sequence):
            joint = self._sequence[self._seq_index]
            st = self._home_status[joint - 1]
            if st.get("homed", False) or st.get("state", 0) == 4:
                self._seq_index += 1
                QTimer.singleShot(200, self._step_sequence)

        # stop polling when nothing is homing
        if not self._sequence_running:
            any_active = any(
                self._home_status[i].get("state", 0) in (1, 2, 3) for i in range(6))
            if not any_active:
                self._home_poll_timer.stop()

    # ── Homing config ───────────────────────────────────────────

    def _apply_homing_config(self):
        """Send the current homing config for all joints to the firmware."""
        cm = self._connection_manager
        if not cm or not cm.is_connected:
            return
        ok = True
        for i, row in enumerate(self._joint_rows):
            search = row[3].value()
            creep = row[4].value()
            backoff = row[5].value()
            offset_deg = row[6].value()
            inv_lim = 1 if row[7].isChecked() else 0
            inv_dir = 1 if row[8].isChecked() else 0
            # offset is in degrees -> steps (approx via config steps/deg)
            offset_steps = int(offset_deg * 100)  # rough; refined after #CFG read
            ok &= cm.home_set_config(
                i + 1, search_speed=search, creep_speed=creep,
                backoff_steps=backoff, home_offset=offset_steps,
                invert_limit=inv_lim, invert_dir=inv_dir)
        return ok
