"""Macro / Waypoint / Trajectory / RT-Streaming panel.

Exposes every new Nite369 v2 command that isn't already in an existing panel:
  - Waypoints:   #WPS / #WPM / #WPL / #WPD
  - Macros:      #MACR / #MACS / #MACP / #MACL / #MACD
  - Trajectory:  #QA  / #QE / #QS / #QC / #QH
  - RT streaming: #RT<Hz>
  - G-code:      G28 / G92 / M114 / M503
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QLineEdit, QSpinBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QMessageBox, QSplitter,
    QFrame, QComboBox,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor

from . import palette as P


_STYLESHEET = f"""
QGroupBox {{
    color: {P.DARK_TEXT}; font-size: 12px; font-weight: bold;
    border: 1px solid {P.DARK_BORDER}; border-radius: 4px;
    margin-top: 10px; padding: 14px 10px 10px 10px;
    background: {P.DARK_PANEL};
}}
QGroupBox::title {{
    subcontrol-origin: margin; left: 10px; padding: 0 6px;
    color: {P.DARK_ACCENT};
}}
QTableWidget {{
    background: {P.DARK_BG}; color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER}; border-radius: 4px;
    gridline-color: {P.DARK_BORDER_SOFT};
    selection-background-color: {P.DARK_ACCENT2};
    font-size: 11px;
}}
QTableWidget::item {{ padding: 2px 4px; }}
QHeaderView::section {{
    background: {P.DARK_PANEL}; color: {P.DARK_TEXT_DIM};
    border: 1px solid {P.DARK_BORDER}; padding: 3px 6px;
    font-size: 11px; font-weight: bold;
}}
QLineEdit {{
    background: {P.DARK_BG}; color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER}; border-radius: 4px;
    padding: 4px 8px; font-size: 12px;
}}
QLineEdit:focus {{ border: 1px solid {P.DARK_ACCENT}; }}
QLabel {{ color: {P.DARK_TEXT}; font-size: 12px; background: transparent; border: none; }}
"""


def _btn(text, tooltip, cb, accent=False):
    btn = QPushButton(text)
    btn.setToolTip(tooltip)
    btn.setFixedHeight(28)
    if accent:
        btn.setStyleSheet(
            P.btn_style(P.DARK_ACCENT, text="#1a1a16",
                        hover=P.DARK_ACCENT_HOVER, font_size=11, padding="2px 10px"))
    else:
        btn.setStyleSheet(
            P.btn_style(P.DARK_BUTTON, font_size=11, padding="2px 10px"))
    btn.clicked.connect(cb)
    return btn


class MacroWaypointPanel(QWidget):
    """Consolidated panel for Waypoints, Macros, Trajectory, RT streaming."""

    # Emitted when a command changes robot state so other panels can refresh.
    robot_state_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.connection_manager = None
        self.setStyleSheet(_STYLESHEET)
        self._setup_ui()
        self._recording = False

    # ── Public API ──────────────────────────────────────────────────

    def set_connection_manager(self, cm):
        self.connection_manager = cm

    # ── UI ──────────────────────────────────────────────────────────

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── Waypoints ───────────────────────────────────────────────
        wp_group = QGroupBox("Waypoints")
        wp_lay = QVBoxLayout(wp_group)
        wp_lay.setSpacing(4)

        row = QHBoxLayout()
        self._wp_name = QLineEdit()
        self._wp_name.setPlaceholderText("waypoint name")
        self._wp_name.setFixedWidth(120)
        row.addWidget(self._wp_name)
        row.addWidget(_btn("Save", "Save current position as waypoint", self._wp_save))
        row.addWidget(_btn("Move", "Move to this waypoint", self._wp_move))
        row.addWidget(_btn("Delete", "Delete waypoint by name", self._wp_delete))
        row.addStretch()
        wp_lay.addLayout(row)

        self._wp_table = QTableWidget(0, 7)
        self._wp_table.setHorizontalHeaderLabels(
            ["Name", "J1", "J2", "J3", "J4", "J5", "J6"])
        self._wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._wp_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._wp_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._wp_table.setMaximumHeight(140)
        wp_lay.addWidget(self._wp_table)

        wp_row2 = QHBoxLayout()
        wp_row2.addWidget(_btn("Refresh List", "Read waypoints from firmware", self._wp_refresh))
        wp_row2.addStretch()
        wp_lay.addLayout(wp_row2)

        root.addWidget(wp_group)

        # ── Macros ──────────────────────────────────────────────────
        mac_group = QGroupBox("Macro (Teach & Repeat)")
        mac_lay = QVBoxLayout(mac_group)
        mac_lay.setSpacing(4)

        mac_row = QHBoxLayout()
        self._mac_rec_btn = _btn("Record", "Start recording commands", self._mac_toggle_rec, accent=True)
        mac_row.addWidget(self._mac_rec_btn)
        mac_row.addWidget(_btn("Play", "Replay recorded macro", self._mac_play))
        mac_row.addWidget(_btn("Clear", "Delete recorded macro", self._mac_clear))
        mac_row.addStretch()
        mac_lay.addLayout(mac_row)

        self._mac_steps_label = QLabel("No macro recorded")
        self._mac_steps_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px;")
        mac_lay.addWidget(self._mac_steps_label)

        mac_row2 = QHBoxLayout()
        mac_row2.addWidget(_btn("List Steps", "Refresh macro step list", self._mac_list))
        mac_row2.addStretch()
        mac_lay.addLayout(mac_row2)

        root.addWidget(mac_group)

        # ── Trajectory Buffer ───────────────────────────────────────
        traj_group = QGroupBox("Trajectory Buffer")
        traj_lay = QVBoxLayout(traj_group)
        traj_lay.setSpacing(4)

        traj_row = QHBoxLayout()
        traj_row.addWidget(QLabel("Target:"))
        self._traj_joints = [None] * 6
        labels = ["J1", "J2", "J3", "J4", "J5", "J6"]
        for i in range(6):
            spin = QSpinBox()
            spin.setRange(-9999, 9999)
            spin.setValue(0)
            spin.setPrefix(f"{labels[i]} ")
            spin.setFixedWidth(80)
            spin.setStyleSheet(P.input_style(font_size=10, padding="2px 4px"))
            self._traj_joints[i] = spin
            traj_row.addWidget(spin)
        traj_lay.addLayout(traj_row)

        traj_row2 = QHBoxLayout()
        traj_row2.addWidget(QLabel("Feed:"))
        self._traj_feed = QSpinBox()
        self._traj_feed.setRange(1, 100000)
        self._traj_feed.setValue(2000)
        self._traj_feed.setFixedWidth(80)
        self._traj_feed.setStyleSheet(P.input_style(font_size=10, padding="2px 4px"))
        traj_row2.addWidget(self._traj_feed)
        traj_row2.addWidget(_btn("Add (#QA)", "Enqueue move to trajectory buffer", self._traj_add))
        traj_row2.addWidget(_btn("Execute (#QE)", "Run all queued moves", self._traj_exec, accent=True))
        traj_row2.addWidget(_btn("Halt (#QH)", "Stop trajectory execution", self._traj_halt))
        traj_row2.addWidget(_btn("Clear (#QC)", "Clear the trajectory buffer", self._traj_clear))
        traj_row2.addStretch()
        traj_lay.addLayout(traj_row2)

        self._traj_status = QLabel("Queue: 0 items")
        self._traj_status.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px;")
        traj_lay.addWidget(self._traj_status)

        traj_row3 = QHBoxLayout()
        traj_row3.addWidget(_btn("Status (#QS)", "Query queue status", self._traj_status_query))
        traj_row3.addStretch()
        traj_lay.addLayout(traj_row3)

        root.addWidget(traj_group)

        # ── RT Streaming + G-code Quick Actions ─────────────────────
        misc_group = QGroupBox("RT Streaming & G-code")
        misc_lay = QVBoxLayout(misc_group)
        misc_lay.setSpacing(4)

        rt_row = QHBoxLayout()
        rt_row.addWidget(QLabel("RT Hz:"))
        self._rt_hz = QSpinBox()
        self._rt_hz.setRange(0, 200)
        self._rt_hz.setValue(10)
        self._rt_hz.setFixedWidth(60)
        self._rt_hz.setStyleSheet(P.input_style(font_size=10, padding="2px 4px"))
        rt_row.addWidget(self._rt_hz)
        rt_row.addWidget(_btn("Start", "Start real-time streaming", self._rt_start))
        rt_row.addWidget(_btn("Stop", "Stop real-time streaming", self._rt_stop))
        rt_row.addSpacing(20)
        rt_row.addWidget(_btn("G28 Home", "Real homing via G28", self._g28_home))
        rt_row.addWidget(_btn("M114 Encoders", "Read encoder positions via M114", self._m114_read))
        rt_row.addWidget(_btn("M503 Config", "Report config via M503", self._m503_report))
        rt_row.addStretch()
        misc_lay.addLayout(rt_row)

        root.addWidget(misc_group)

        root.addStretch()

    # ── Helpers ─────────────────────────────────────────────────────

    def _cm(self):
        return self.connection_manager

    def _ok(self, result, context=""):
        if result is None:
            self._log(f"{context}: no response")
            return False
        if isinstance(result, str):
            ok = result.startswith(">OK") or "DONE" in result
            if not ok:
                self._log(f"{context}: {result[:80]}")
            return ok
        return bool(result)

    def _log(self, msg):
        if self.connection_manager:
            self.connection_manager.messageReceived.emit(msg)

    # ── Waypoint handlers ───────────────────────────────────────────

    def _wp_save(self):
        name = self._wp_name.text().strip()
        if not name:
            self._log("Waypoint name is empty")
            return
        cm = self._cm()
        if cm:
            ok = cm.waypoint_save(name)
            self._log(f"Waypoint '{name}' {'saved' if ok else 'save FAILED'}")
            if ok:
                self._wp_refresh()

    def _wp_move(self):
        name = self._wp_name.text().strip()
        if not name:
            self._log("Waypoint name is empty")
            return
        cm = self._cm()
        if cm:
            ok = cm.waypoint_move(name)
            self._log(f"Move to '{name}' {'OK' if ok else 'FAILED'}")

    def _wp_delete(self):
        name = self._wp_name.text().strip()
        if not name:
            self._log("Waypoint name is empty")
            return
        cm = self._cm()
        if cm:
            ok = cm.waypoint_delete(name)
            self._log(f"Delete '{name}' {'OK' if ok else 'FAILED'}")
            if ok:
                self._wp_refresh()

    def _wp_refresh(self):
        cm = self._cm()
        if not cm:
            return
        wps = cm.waypoint_list()
        self._wp_table.setRowCount(0)
        if wps is None:
            self._log("No waypoints or firmware doesn't support #WPL")
            return
        for wp in wps:
            row = self._wp_table.rowCount()
            self._wp_table.insertRow(row)
            self._wp_table.setItem(row, 0, QTableWidgetItem(wp.get("name", "?")))
            for j in range(6):
                val = wp.get(f"j{j+1}", 0.0)
                self._wp_table.setItem(row, j + 1, QTableWidgetItem(f"{val:.1f}"))
        self._log(f"Loaded {len(wps)} waypoint(s)")

    # ── Macro handlers ──────────────────────────────────────────────

    def _mac_toggle_rec(self):
        cm = self._cm()
        if not cm:
            return
        if not self._recording:
            ok = cm.macro_record_start()
            if ok:
                self._recording = True
                self._mac_rec_btn.setText("Stop Rec")
                self._log("Macro recording STARTED")
            else:
                self._log("Macro record start FAILED")
        else:
            ok = cm.macro_record_stop()
            if ok:
                self._recording = False
                self._mac_rec_btn.setText("Record")
                self._log("Macro recording STOPPED")
                self._mac_list()
            else:
                self._log("Macro record stop FAILED")

    def _mac_play(self):
        cm = self._cm()
        if cm:
            self._log("Macro replay starting...")
            ok = cm.macro_play()
            self._log(f"Macro replay {'DONE' if ok else 'FAILED'}")

    def _mac_clear(self):
        cm = self._cm()
        if cm:
            ok = cm.macro_clear()
            self._log(f"Macro cleared {'OK' if ok else 'FAILED'}")
            self._mac_steps_label.setText("No macro recorded")

    def _mac_list(self):
        cm = self._cm()
        if not cm:
            return
        steps = cm.macro_list()
        if steps is None:
            self._mac_steps_label.setText("No macro or firmware doesn't support #MACL")
            return
        if not steps:
            self._mac_steps_label.setText("No macro recorded")
        else:
            self._mac_steps_label.setText(f"{len(steps)} step(s): " + " -> ".join(steps[:5])
                                            + ("..." if len(steps) > 5 else ""))
        self._log(f"Macro: {len(steps)} step(s)")

    # ── Trajectory handlers ─────────────────────────────────────────

    def _traj_add(self):
        cm = self._cm()
        if not cm:
            return
        targets = {}
        for i in range(6):
            val = self._traj_joints[i].value()
            if val != 0:
                targets[i + 1] = float(val)
        if not targets:
            self._log("Trajectory add: all joints are zero — nothing to enqueue")
            return
        feed = self._traj_feed.value()
        ok = cm.queue_add(targets, feed)
        self._log(f"Trajectory add: {'OK' if ok else 'FAILED'} {targets} F{feed}")
        self._traj_status_query()

    def _traj_exec(self):
        cm = self._cm()
        if cm:
            self._log("Trajectory execute starting...")
            ok = cm.queue_execute()
            self._log(f"Trajectory execute {'DONE' if ok else 'FAILED'}")
            self._traj_status_query()

    def _traj_halt(self):
        cm = self._cm()
        if cm:
            ok = cm.queue_halt()
            self._log(f"Trajectory halt {'OK' if ok else 'FAILED'}")

    def _traj_clear(self):
        cm = self._cm()
        if cm:
            ok = cm.queue_clear()
            self._log(f"Trajectory clear {'OK' if ok else 'FAILED'}")
            self._traj_status_query()

    def _traj_status_query(self):
        cm = self._cm()
        if not cm:
            return
        st = cm.queue_status()
        if st is None:
            self._traj_status.setText("Queue: unknown")
            return
        status_parts = [f"Count: {st['count']}"]
        if st['executing']:
            status_parts.append("EXECUTING")
        if st['tail'] != 0:
            status_parts.append(f"Tail: {st['tail']}")
        self._traj_status.setText("Queue: " + ", ".join(status_parts))

    # ── RT Streaming handlers ───────────────────────────────────────

    def _rt_start(self):
        cm = self._cm()
        if cm:
            hz = self._rt_hz.value()
            result = cm.rt_streaming(hz)
            self._log(f"RT streaming: {result[:60] if result else 'no response'}")

    def _rt_stop(self):
        cm = self._cm()
        if cm:
            result = cm.rt_streaming(0)
            self._log(f"RT streaming off: {result[:60] if result else 'no response'}")

    # ── G-code quick actions ────────────────────────────────────────

    def _g28_home(self):
        cm = self._cm()
        if cm:
            self._log("G28 homing started...")
            ok = cm.gcode_home()
            self._log(f"G28 {'DONE' if ok else 'FAILED'}")
            self.robot_state_changed.emit("idle")

    def _m114_read(self):
        cm = self._cm()
        if cm:
            enc = cm.gcode_read_encoders()
            if enc:
                self._log(f"M114 encoders: {', '.join(f'{v:.1f}' for v in enc)}")
            else:
                self._log("M114: no response or firmware doesn't support it")

    def _m503_report(self):
        cm = self._cm()
        if cm:
            result = cm.gcode_report_config()
            self._log(f"M503: {result[:200] if result else 'no response'}")
