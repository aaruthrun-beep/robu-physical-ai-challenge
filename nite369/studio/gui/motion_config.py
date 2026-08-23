"""Motion Configuration panel for the Astra robot.

Lets the operator tune the robot's motion profile — per-joint max speed,
acceleration, deceleration, steps-per-degree calibration, direction, and
soft limits — then push it to the firmware using the master's #CFG/#CF/#CS
protocol and Marlin-style M-codes.

Firmware axis mapping:
  - Slave 1  = J1, J2, J3 (axes 0-2)
  - Slave 2  = J4, J5, J6 (axes 0-2)

Protocol notes (master spi_cmd_master_eth.c):
  - #CFG<j>           read  {steps_per_rev, gear_ratio, dir_inverted}
  - #CFG<j>,s,gr,di   write per-joint calibration (persisted on the slave)
  - #CR<j>            read  {max_speed, accel, decel}
  - #CF<j>,m,a,d      write per-joint motion profile (persisted on the slave)
  - #CS               save both slaves' config to flash
  - M500/M501         save/load the master's local copy (kept in sync)
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QTabWidget, QDoubleSpinBox, QSpinBox, QSlider,
    QComboBox, QFrame, QGridLayout, QButtonGroup, QRadioButton,
    QMessageBox,
)
from PyQt5.QtCore import Qt

from . import palette as P


# Per-joint soft limits (Astra Studio side, degrees) — defaults match the DH arm
DEFAULT_SOFT_LIMITS = [
    (-170.0, 170.0),   # J1
    (-85.0, 85.0),     # J2
    (-130.0, 130.0),   # J3
    (-170.0, 170.0),   # J4
    (-115.0, 115.0),   # J5
    (-170.0, 170.0),   # J6
]

MOTION_PRESETS = {
    "Balanced (default)": {
        "max_speed": 1500.0,   # steps/s
        "accel": 500.0,        # steps/s² (finite move)
        "decel": 500.0,        # steps/s² (finite move)
        "jog_accel": 1000.0,   # steps/s² (continuous jog ramp-up)
        "jog_decel": 2000.0,   # steps/s² (continuous jog stop)
    },
    "Fast": {
        "max_speed": 8000.0,
        "accel": 4000.0,
        "decel": 4000.0,
        "jog_accel": 6000.0,
        "jog_decel": 16000.0,
    },
    "Very Fast": {
        "max_speed": 8000.0,
        "accel": 6000.0,
        "decel": 6000.0,
        "jog_accel": 8000.0,
        "jog_decel": 24000.0,
    },
    "Safe / Gentle": {
        "max_speed": 800.0,
        "accel": 200.0,
        "decel": 300.0,
        "jog_accel": 400.0,
        "jog_decel": 800.0,
    },
}

JOINT_LABELS = [
    "J1 — Base Rotation",
    "J2 — Shoulder",
    "J3 — Elbow",
    "J4 — Forearm Roll",
    "J5 — Wrist Pitch",
    "J6 — Wrist Roll",
]


class JointMotionTab(QWidget):
    """Editable motion profile for a single joint."""

    def __init__(self, joint_index, parent=None):
        super().__init__(parent)
        self.joint_index = joint_index
        self.slave = "slave1" if joint_index < 3 else "slave2"
        self.axis = joint_index % 3 if joint_index < 3 else joint_index - 3
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # ── Joint header ──────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel(JOINT_LABELS[self.joint_index])
        title.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()
        slave_tag = QLabel(f"Controller: {'Slave 1 (base)' if self.slave == 'slave1' else 'Slave 2 (wrist)'}")
        slave_tag.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        header.addWidget(slave_tag)
        layout.addLayout(header)

        # ── Motion profile sliders ────────────────────────────────
        motion_group = QGroupBox("Motion Profile (steps/s)")
        motion_group.setStyleSheet(P.groupbox_style())
        motion_layout = QGridLayout(motion_group)
        motion_layout.setSpacing(8)

        def add_slider_row(row, label, attr, lo, hi, value, unit):
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 12px; background: transparent; border: none;")
            motion_layout.addWidget(lbl, row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(lo, hi)
            slider.setValue(int(value))
            slider.setStyleSheet(P.slider_style())
            setattr(self, f"{attr}_slider", slider)
            motion_layout.addWidget(slider, row, 1)

            spin = QDoubleSpinBox()
            spin.setRange(float(lo), float(hi))
            spin.setDecimals(0)
            spin.setValue(float(value))
            spin.setFixedWidth(110)
            spin.setStyleSheet(P.input_style(font_size=11))
            setattr(self, f"{attr}_spin", spin)
            motion_layout.addWidget(spin, row, 2)

            # Unit label OUTSIDE the spin box (a suffix inside gets selected
            # when editing the number).
            unit_lbl = QLabel(unit)
            unit_lbl.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
            motion_layout.addWidget(unit_lbl, row, 3)

            # Two-way sync
            slider.valueChanged.connect(lambda v, s=spin: s.setValue(float(v)))
            spin.valueChanged.connect(lambda v, s=slider: s.setValue(int(v)))

            return lbl, slider, spin

        add_slider_row(0, "Max Speed", "speed", 100, 8000, 1500, "steps/s")
        add_slider_row(1, "Acceleration", "accel", 50, 10000, 500, "steps/s²")
        add_slider_row(2, "Deceleration", "decel", 50, 10000, 500, "steps/s²")
        # Continuous jog (hold-to-run): separate ramp-up / stop profiles.
        add_slider_row(3, "Jog Accel", "jog_accel", 50, 10000, 1000, "steps/s²")
        # Rapid stop for continuous jog (hold-to-run release). Higher = snappier.
        add_slider_row(4, "Jog Stop Decel", "jog_decel", 50, 40000, 2000, "steps/s²")

        note = QLabel("Applied per joint on its controller (steps/s). "
                      "Jog Accel is the ramp-up when you start a continuous jog "
                      "(#JC); Jog Stop Decel is used when you release it — it "
                      "decelerates fast instead of stopping instantly.")
        note.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 10px; background: transparent; border: none;")
        note.setWordWrap(True)
        motion_layout.addWidget(note, 5, 0, 1, 4)

        layout.addWidget(motion_group)

        # ── Calibration & direction ───────────────────────────────
        cal_group = QGroupBox("Calibration")
        cal_group.setStyleSheet(P.groupbox_style())
        cal_layout = QGridLayout(cal_group)
        cal_layout.setSpacing(8)

        cal_layout.addWidget(QLabel("Steps per degree:"), 0, 0)
        self.steps_spin = QDoubleSpinBox()
        self.steps_spin.setRange(0.01, 10000.0)
        self.steps_spin.setDecimals(3)
        self.steps_spin.setValue(200.0)
        self.steps_spin.setFixedWidth(120)
        self.steps_spin.setStyleSheet(P.input_style(font_size=11))
        self.steps_spin.setToolTip("Motor steps to rotate this joint 1 degree (calibration)")
        cal_layout.addWidget(self.steps_spin, 0, 1)

        cal_layout.addWidget(QLabel("Gear ratio (x100):"), 1, 0)
        self.gear_spin = QSpinBox()
        self.gear_spin.setRange(1, 100000)
        self.gear_spin.setValue(100)
        self.gear_spin.setFixedWidth(120)
        self.gear_spin.setStyleSheet(P.input_style(font_size=11))
        self.gear_spin.setToolTip("Gearbox ratio stored ×100 (e.g. 2280 = 22.8:1). Changing this keeps steps/deg fixed by recomputing steps_per_rev.")
        cal_layout.addWidget(self.gear_spin, 1, 1)

        # Steps/deg and gear ratio are coupled via steps_per_rev:
        #   steps_per_deg = steps_per_rev * gear_ratio / 100 / 360
        self.steps_spin.valueChanged.connect(self._sync_from_steps_per_deg)
        self.gear_spin.valueChanged.connect(self._sync_from_gear_ratio)
        self._spr = self._compute_spr(self.steps_spin.value(), self.gear_spin.value())

        cal_layout.addWidget(QLabel("Direction:"), 2, 0)
        dir_box = QWidget()
        dir_layout = QHBoxLayout(dir_box)
        dir_layout.setContentsMargins(0, 0, 0, 0)
        self.dir_group = QButtonGroup(self)
        self.dir_normal = QRadioButton("Normal")
        self.dir_inverted = QRadioButton("Inverted")
        for rb in (self.dir_normal, self.dir_inverted):
            rb.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 11px; background: transparent; border: none;")
            self.dir_group.addButton(rb)
            dir_layout.addWidget(rb)
        self.dir_normal.setChecked(True)
        cal_layout.addWidget(dir_box, 2, 1)

        layout.addWidget(cal_group)

        # ── Soft limits ───────────────────────────────────────────
        lim_group = QGroupBox("Soft Limits (Astra Studio)")
        lim_group.setStyleSheet(P.groupbox_style())
        lim_layout = QHBoxLayout(lim_group)

        lim_layout.addWidget(QLabel("Min:"))
        self.lim_min = QDoubleSpinBox()
        self.lim_min.setRange(-360.0, 360.0)
        self.lim_min.setDecimals(1)
        self.lim_min.setFixedWidth(90)
        self.lim_min.setStyleSheet(P.input_style(font_size=11))
        lim_layout.addWidget(self.lim_min)
        lim_deg_lbl = QLabel("°")
        lim_deg_lbl.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        lim_layout.addWidget(lim_deg_lbl)

        lim_layout.addWidget(QLabel("Max:"))
        self.lim_max = QDoubleSpinBox()
        self.lim_max.setRange(-360.0, 360.0)
        self.lim_max.setDecimals(1)
        self.lim_max.setFixedWidth(90)
        self.lim_max.setStyleSheet(P.input_style(font_size=11))
        lim_layout.addWidget(self.lim_max)
        lim_layout.addWidget(QLabel("°"))

        lim_layout.addStretch()
        layout.addWidget(lim_group)

        # Default soft limits
        lo, hi = DEFAULT_SOFT_LIMITS[self.joint_index]
        self.lim_min.setValue(lo)
        self.lim_max.setValue(hi)

        layout.addStretch()

    # ── Coupled calibration fields ────────────────────────────────

    def _sync_from_steps_per_deg(self, spd):
        """User edited steps/deg — keep gear ratio, recompute spr."""
        self._spr = self._compute_spr(spd, self.gear_spin.value())

    def _sync_from_gear_ratio(self, gr):
        """User edited gear ratio — keep steps/deg, recompute spr."""
        self._spr = self._compute_spr(self.steps_spin.value(), gr)

    @staticmethod
    def _compute_spr(spd, gr):
        if gr <= 0:
            return 3200
        return int(round(spd * 360.0 * 100.0 / gr))

    def steps_per_deg(self) -> float:
        return self.steps_spin.value()

    def gear_ratio(self) -> int:
        return self.gear_spin.value()

    def steps_per_rev(self) -> int:
        return self._compute_spr(self.steps_spin.value(), self.gear_spin.value())

    # ── Accessors ─────────────────────────────────────────────────

    def get_values(self):
        """Return the tab's editable values as a dict."""
        return {
            "max_speed": self.speed_spin.value(),
            "accel": self.accel_spin.value(),
            "decel": self.decel_spin.value(),
            "jog_accel": self.jog_accel_spin.value(),
            "jog_decel": self.jog_decel_spin.value(),
            "steps_per_deg": self.steps_spin.value(),
            "gear_ratio": self.gear_spin.value(),
            "dir_invert": 1 if self.dir_inverted.isChecked() else 0,
            "lim_min": self.lim_min.value(),
            "lim_max": self.lim_max.value(),
        }

    def set_values(self, values: dict):
        """Populate the tab from parsed firmware config.

        Expects keys: max_speed, accel, decel (steps/s),
        steps_per_deg (float), gear_ratio (int, ×100), dir_invert (0/1).
        """
        self.speed_spin.setValue(values.get("max_speed", 1500.0))
        self.accel_spin.setValue(values.get("accel", 500.0))
        self.decel_spin.setValue(values.get("decel", 500.0))
        self.jog_accel_spin.setValue(values.get("jog_accel", 1000.0))
        self.jog_decel_spin.setValue(values.get("jog_decel", 2000.0))

        spd = float(values.get("steps_per_deg", 200.0))
        gr = int(values.get("gear_ratio", 100))
        # Block signals so loading doesn't fight the coupling handlers
        self.steps_spin.blockSignals(True)
        self.gear_spin.blockSignals(True)
        self.steps_spin.setValue(spd)
        self.gear_spin.setValue(gr)
        self.steps_spin.blockSignals(False)
        self.gear_spin.blockSignals(False)
        self._spr = self._compute_spr(spd, gr)

        self.dir_inverted.setChecked(bool(values.get("dir_invert", 0)))
        self.dir_normal.setChecked(not bool(values.get("dir_invert", 0)))
        if "lim_min" in values:
            self.lim_min.setValue(values["lim_min"])
        if "lim_max" in values:
            self.lim_max.setValue(values["lim_max"])


class MotionConfigPanel(QWidget):
    """Robot motion profile configuration with firmware sync."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connection_manager = None
        self._connected = False
        self._loaded = None  # last parsed firmware config
        self._setup_ui()

    def set_connection_manager(self, cm):
        self._connection_manager = cm

    def set_connected(self, connected: bool):
        """Enable/disable controls based on connection state.

        Read/Apply/Save need the real robot (firmware read/write), so they
        stay gated on connection. Apply Preset only fills the panel's values
        and works in simulation too.
        """
        self._connected = connected
        self.conn_dot.setStyleSheet(
            f"color: {P.DARK_SUCCESS if connected else P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;"
        )
        self.conn_dot.setToolTip("Connected" if connected else "Not connected")
        for btn in (self.read_btn, self.apply_btn, self.save_btn, self.reload_btn):
            btn.setEnabled(connected)
        # Preset applies to the UI only — always usable (sim or real).
        self.preset_apply_btn.setEnabled(True)
        self._set_status(
            "Ready — connect to the robot to read its configuration"
            if not connected else
            "Connected — press Read from Robot to load the current configuration"
        )

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # ── Header ──────────────────────────────────────────────
        header = QHBoxLayout()
        title = QLabel("Motion Configuration")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 15px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()
        self.conn_dot = QLabel("●")
        self.conn_dot.setStyleSheet(f"color: {P.DARK_ERROR}; font-size: 12px; background: transparent; border: none;")
        self.conn_dot.setToolTip("Not connected")
        header.addWidget(self.conn_dot)
        layout.addLayout(header)

        # ── Action bar ──────────────────────────────────────────
        bar = QHBoxLayout()
        bar.setSpacing(6)

        self.read_btn = QPushButton("Read from Robot")
        self.read_btn.setStyleSheet(P.accent_btn_style(font_size=11))
        self.read_btn.clicked.connect(self._on_read)
        bar.addWidget(self.read_btn)

        self.apply_btn = QPushButton("Apply & Save")
        self.apply_btn.setStyleSheet(P.success_btn_style(font_size=11))
        self.apply_btn.clicked.connect(self._on_apply_save)
        bar.addWidget(self.apply_btn)

        self.save_btn = QPushButton("Save to Flash")
        self.save_btn.setStyleSheet(P.warning_btn_style(font_size=11))
        self.save_btn.clicked.connect(self._on_save)
        bar.addWidget(self.save_btn)

        self.reload_btn = QPushButton("Reload from Flash")
        self.reload_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11))
        self.reload_btn.clicked.connect(self._on_reload)
        bar.addWidget(self.reload_btn)

        layout.addLayout(bar)

        # ── Presets ─────────────────────────────────────────────
        preset_row = QHBoxLayout()
        preset_row.setSpacing(6)
        preset_row.addWidget(QLabel("Preset:"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(list(MOTION_PRESETS.keys()))
        self.preset_combo.setFixedWidth(180)
        self.preset_combo.setStyleSheet(P.input_style(font_size=11))
        preset_row.addWidget(self.preset_combo)
        self.preset_apply_btn = QPushButton("Apply Preset")
        self.preset_apply_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11))
        self.preset_apply_btn.clicked.connect(self._on_apply_preset)
        preset_row.addWidget(self.preset_apply_btn)
        preset_row.addStretch()
        layout.addLayout(preset_row)

        # ── Joint tabs ──────────────────────────────────────────
        self.tabs = QTabWidget()
        self._tabs = []
        for i in range(6):
            tab = JointMotionTab(i)
            self._tabs.append(tab)
            self.tabs.addTab(tab, f"J{i+1}")
        layout.addWidget(self.tabs, 1)

        # ── Status ──────────────────────────────────────────────
        status_frame = QFrame()
        status_frame.setStyleSheet(f"QFrame {{ background: {P.DARK_INPUT}; border: 1px solid {P.DARK_BORDER}; border-radius: 4px; }}")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(10, 5, 10, 5)
        self.status_label = QLabel("Ready — connect to the robot to read its configuration")
        self.status_label.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 11px; border: none; background: transparent;")
        self.status_label.setWordWrap(True)
        status_layout.addWidget(self.status_label)
        layout.addWidget(status_frame)

        self.set_connected(False)

    def _set_status(self, message, level="info"):
        colors = {"info": P.DARK_TEXT_DIM, "success": P.DARK_SUCCESS,
                  "error": P.DARK_ERROR, "warning": P.DARK_WARNING}
        self.status_label.setStyleSheet(
            f"color: {colors.get(level, P.DARK_TEXT_DIM)}; font-size: 11px; border: none; background: transparent;"
        )
        self.status_label.setText(message)

    # ── Read ────────────────────────────────────────────────────

    # Known mechanical gear ratios (×100) for the base joints. The firmware
    # defaults to 100 (1:1) until calibrated; when the robot reports that
    # unset value we use the real mechanical ratio so the panel doesn't
    # clobber it. J4-J6 ratios are still unknown (placeholders).
    KNOWN_GEAR_RATIOS = {1: 2280, 2: 4596, 3: 4078}

    def _on_read(self):
        if not self._connection_manager:
            self._set_status("No connection manager available", "error")
            return
        cfg = {"joints": {}, "loaded": False}
        ok = True
        for i in range(1, 7):
            c = self._connection_manager.cfg_read(i)
            m = self._connection_manager.cf_read(i)
            if c is None or m is None:
                ok = False
                continue
            cfg["joints"][i] = {**c, **m}
        if not ok or not cfg["joints"]:
            self._set_status(
                "Couldn't read the config from the robot. Make sure it's connected and try again.",
                "error",
            )
            return
        cfg["loaded"] = True
        self._loaded = cfg
        for i, tab in enumerate(self._tabs):
            j = cfg["joints"].get(i + 1)
            if not j:
                continue
            spr = int(j.get("steps_per_rev", 3200))
            gr = int(j.get("gear_ratio", 100))
            # The reported steps/deg is what actually matters for motion. If
            # the robot still has the unset 1:1 default, substitute the real
            # mechanical ratio and recompute steps_per_rev so the joint keeps
            # the same steps/deg (the firmware stores spr and gr separately,
            # and a stale spr=72000 was computed for gr=100).
            spd_reported = spr * gr / 100.0 / 360.0
            if gr <= 100:
                gr = self.KNOWN_GEAR_RATIOS.get(i + 1, gr)
                spr = int(round(spd_reported * 360.0 * 100.0 / gr))
            spd = spd_reported
            tab.set_values({
                "max_speed": float(j.get("max_speed", 1500.0)),
                "accel": float(j.get("accel", 500.0)),
                "decel": float(j.get("decel", 500.0)),
                "jog_accel": float(j.get("jog_accel", 1000.0)),
                "jog_decel": float(j.get("jog_decel", 2000.0)),
                "steps_per_deg": spd,
                "gear_ratio": gr,
                "dir_invert": int(j.get("dir_inverted", 0)),
            })
        self._set_status("Config read OK — all joints loaded", "success")

    # ── Apply & Save ────────────────────────────────────────────

    def _on_apply_save(self):
        if not self._connection_manager or not self._connected:
            self._set_status("Not connected — connect first", "error")
            return
        written = []
        failures = []
        for i, tab in enumerate(self._tabs):
            v = tab.get_values()
            joint = i + 1

            # Calibration: #CFG<j>,<spr>,<gr>,<di>
            spr = tab.steps_per_rev()
            gr = v["gear_ratio"]
            di = v["dir_invert"]
            if self._connection_manager.cfg_write(joint, spr=spr, gr=gr, di=di):
                written.append(f"CFG{joint}")
            else:
                failures.append(f"CFG{joint}")

            # Motion profile: #CF<j>,<max>,<accel>,<decel>,<jog_decel>,<jog_accel>.
            # max_speed is stored as a reference for the studio's jog clamp
            # (the firmware does NOT enforce it — the studio clamps first).
            if self._connection_manager.cf_write(
                    joint,
                    max_speed=int(v["max_speed"]),
                    accel=int(v["accel"]),
                    decel=int(v["decel"]),
                    jog_accel=int(v["jog_accel"]),
                    jog_decel=int(v["jog_decel"])):
                written.append(f"CF{joint}")
            else:
                failures.append(f"CF{joint}")

        # Persist on both slaves (#CS)
        if self._connection_manager.cs_save():
            written.append("CS")
        else:
            failures.append("CS")

        if failures:
            self._set_status(
                f"Applied {len(written)} values; {len(failures)} failed: {', '.join(failures[:4])}",
                "warning",
            )
        else:
            self._set_status(f"Applied and saved to flash ({len(written)} values)", "success")

    # ── Save / Reload ───────────────────────────────────────────

    def _on_save(self):
        if not self._connection_manager:
            self._set_status("No connection manager available", "error")
            return
        if self._connection_manager.cs_save():
            self._set_status("Configuration saved to flash (both slaves)", "success")
        else:
            self._set_status("Couldn't save to flash — is the robot connected?", "error")

    def _on_reload(self):
        if not self._connection_manager:
            self._set_status("No connection manager available", "error")
            return
        if self._connection_manager.load_robot_config():
            self._set_status("Configuration reloaded from flash — press Read to refresh the panel", "success")
        else:
            self._set_status("Couldn't reload from flash", "error")

    # ── Presets ─────────────────────────────────────────────────

    def _on_apply_preset(self):
        preset = MOTION_PRESETS.get(self.preset_combo.currentText())
        if not preset:
            return
        for tab in self._tabs:
            tab.speed_spin.setValue(preset["max_speed"])
            tab.accel_spin.setValue(preset["accel"])
            tab.decel_spin.setValue(preset["decel"])
            tab.jog_accel_spin.setValue(preset["jog_accel"])
            tab.jog_decel_spin.setValue(preset["jog_decel"])
        self._set_status(f"Preset '{self.preset_combo.currentText()}' loaded into the panel — press Apply & Save to send it", "info")
