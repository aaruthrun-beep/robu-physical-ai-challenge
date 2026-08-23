"""Arctos-style program editor with target-based workflow.

Provides program tree with targets, record/update buttons,
drag-and-drop reorder, and run/stop controls.
"""

import time
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QListWidget, QListWidgetItem, QMenu, QInputDialog,
    QDoubleSpinBox, QSpinBox, QFrame, QSplitter, QCheckBox,
)
from PyQt5.QtCore import Qt, pyqtSignal, QThread
from PyQt5.QtGui import QColor, QFont

from . import palette as P


class ProgramExecutionThread(QThread):
    step_started = pyqtSignal(int, str)
    step_completed = pyqtSignal(int)
    program_completed = pyqtSignal()
    error_occurred = pyqtSignal(int, str)

    def __init__(self, targets, controller, connection_manager, direct_control=False):
        super().__init__()
        self.targets = targets
        self.controller = controller
        self.connection_manager = connection_manager
        self.direct_control = direct_control
        self._running = True
        self._paused = False

    def run(self):
        for i, target in enumerate(self.targets):
            if not self._running:
                break
            while self._paused and self._running:
                self.msleep(50)
            if not self._running:
                break

            name = target.get("name", f"Target {i+1}")
            self.step_started.emit(i, name)

            try:
                import math
                # targets store DEGREES (user-facing); the controller/sim
                # expect radians.
                joints = [math.radians(j) for j in target.get("joints", [0.0] * 6)]
                speed = target.get("speed", 50)
                gripper = target.get("gripper", None)
                delay = target.get("delay", 0)

                if self.controller:
                    self.controller.move_joints(joints, speed=speed, send_to_robot=self.direct_control)

                if gripper is not None and self.connection_manager:
                    self.connection_manager.set_gripper(gripper)

                if delay > 0:
                    time.sleep(delay)

                self.step_completed.emit(i)
            except Exception as e:
                self.error_occurred.emit(i, str(e))

        self.program_completed.emit()

    def stop(self):
        self._running = False

    def pause(self):
        self._paused = True

    def resume(self):
        self._paused = False


class ProgramPanel(QWidget):
    """Arctos-style program tree with targets."""

    program_changed = pyqtSignal(object)
    targets_changed = pyqtSignal(int)  # target count — lets the floating overlay grow

    def __init__(self, parent=None):
        super().__init__(parent)
        self.targets = []
        self._selected_idx = -1
        self._sim = None
        self._controller = None
        self._connection_manager = None
        self._exec_thread = None
        self._setup_ui()

    def set_simulation(self, sim):
        self._sim = sim

    def set_gripper(self, gripper_01):
        """Sync the program's gripper spinbox from the gripper panel (0..1)."""
        if hasattr(self, "gripper_spinbox"):
            try:
                self.gripper_spinbox.setValue(gripper_01 * 100.0)
            except Exception:
                pass

    def set_controller(self, controller):
        self._controller = controller

    def set_connection_manager(self, cm):
        self._connection_manager = cm

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        toolbar = QHBoxLayout()
        toolbar.setContentsMargins(8, 6, 8, 6)
        toolbar.setSpacing(6)
        title = QLabel("Program")
        title.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
        toolbar.addWidget(title)
        toolbar.addStretch()

        for text, color, callback in [
            ("Record", P.DARK_SUCCESS, self._record_target),
            ("Update", P.DARK_ACCENT, self._update_target),
            ("Delete", P.DARK_ERROR, self._delete_target),
        ]:
            btn = QPushButton(text)
            btn.setFixedHeight(30)
            btn.setMinimumWidth(0)
            btn.setStyleSheet(P.btn_style(color, font_size=11, padding="2px 8px"))
            btn.clicked.connect(callback)
            toolbar.addWidget(btn)
        layout.addLayout(toolbar)

        self.target_list = QListWidget()
        self.target_list.setDragDropMode(QListWidget.InternalMove)
        self.target_list.setStyleSheet(f"""
            QListWidget {{
                background: transparent; color: {P.DARK_TEXT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 6px;
                font-size: 12px; font-family: 'Consolas', monospace;
                outline: none; padding: 4px;
            }}
            QListWidget::item {{
                padding: 8px 10px; border-bottom: 1px solid {P.DARK_BORDER_SOFT};
                border-radius: 4px;
            }}
            QListWidget::item:selected {{ background: {P.DARK_ACCENT}; color: #1a1a16; }}
            QListWidget::item:hover {{ background: {P.DARK_BUTTON_HOVER}; }}
        """)
        self.target_list.currentRowChanged.connect(self._on_select)
        self.target_list.setContextMenuPolicy(Qt.CustomContextMenu)
        self.target_list.customContextMenuRequested.connect(self._context_menu)
        layout.addWidget(self.target_list, 1)

        params_frame = QFrame()
        params_frame.setStyleSheet(f"QFrame {{ background: transparent; border-top: 1px solid {P.DARK_BORDER}; }}")
        params_layout = QHBoxLayout(params_frame)
        params_layout.setContentsMargins(8, 6, 8, 6)
        params_layout.setSpacing(6)

        for label_text, attr_name, default, rng in [
            ("Speed:", "speed_spinbox", 50, (1, 100)),
            ("Delay:", "delay_spinbox", 0.5, (0, 60)),
            ("Trans:", "transition_spinbox", 1.5, (0.1, 30)),
            ("Gripper:", "gripper_spinbox", 0, (0, 100)),
        ]:
            lbl = QLabel(label_text)
            lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; border: none; background: transparent;")
            params_layout.addWidget(lbl)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(rng[0], rng[1])
            spinbox.setValue(default)
            spinbox.setFixedWidth(62)
            spinbox.setStyleSheet(P.input_style(font_size=11))
            setattr(self, attr_name, spinbox)
            params_layout.addWidget(spinbox)

        layout.addWidget(params_frame)

        # ── Direct Control Toggle ─────────────────────────────────
        self._direct_control_check = QCheckBox("Direct Control (real robot)")
        self._direct_control_check.setStyleSheet(f"color: {P.DARK_ACCENT}; font-size: 12px; font-weight: bold; spacing: 6px; background: transparent; border: none;")
        layout.addWidget(self._direct_control_check)

        exec_row = QHBoxLayout()
        exec_row.setContentsMargins(8, 6, 8, 6)
        exec_row.setSpacing(6)

        self.run_btn = QPushButton("▶ Run Program")
        self.run_btn.setFixedHeight(34)
        self.run_btn.setStyleSheet(P.success_btn_style(font_size=12, padding="4px 14px"))
        self.run_btn.clicked.connect(self._run_program)
        exec_row.addWidget(self.run_btn)

        self.stop_btn = QPushButton("⏹ Stop")
        self.stop_btn.setFixedHeight(34)
        self.stop_btn.setStyleSheet(P.danger_btn_style(font_size=12, padding="4px 14px"))
        self.stop_btn.clicked.connect(self._stop_program)
        self.stop_btn.setEnabled(False)
        exec_row.addWidget(self.stop_btn)
        layout.addLayout(exec_row)

    def _brighten(self, c):
        try:
            h = c.lstrip("#")
            r, g, b = int(h[:2], 16), int(h[2:4], 16), int(h[4:], 16)
            return f"#{min(255,r+30):02x}{min(255,g+30):02x}{min(255,b+30):02x}"
        except Exception:
            return c

    def _record_target(self):
        if not self._sim or "astra" not in self._sim.robots:
            return
        # sim joints are RADIANS; store degrees (the user-facing unit).
        import math
        joints_rad = self._sim.get_joint_positions("astra")
        joints_deg = [math.degrees(j) for j in joints_rad]
        target = {
            "name": f"Target {len(self.targets) + 1}",
            "joints": joints_deg,
            "gripper": self.gripper_spinbox.value() / 100.0,
            "speed": self.speed_spinbox.value(),
            "delay": self.delay_spinbox.value(),
            "transition_time": self.transition_spinbox.value(),
        }
        self.targets.append(target)
        self._refresh_list()
        self.program_changed.emit(self)

    def _update_target(self):
        idx = self.target_list.currentRow()
        if idx < 0 or idx >= len(self.targets):
            return
        if not self._sim or "astra" not in self._sim.robots:
            return
        import math
        joints_rad = self._sim.get_joint_positions("astra")
        self.targets[idx]["joints"] = [math.degrees(j) for j in joints_rad]
        self._refresh_list()

    def _delete_target(self):
        idx = self.target_list.currentRow()
        if idx < 0 or idx >= len(self.targets):
            return
        self.targets.pop(idx)
        self._refresh_list()
        self.program_changed.emit(self)

    def _refresh_list(self):
        self.target_list.blockSignals(True)
        self.target_list.clear()
        for i, target in enumerate(self.targets):
            name = target.get("name", f"Target {i+1}")
            joints = target.get("joints", [0.0] * 6)
            joints_str = ", ".join(f"{j:.1f}" for j in joints[:3])
            g = target.get("gripper")
            grip_str = f" | Grip {g*100:.0f}%" if g is not None else ""
            text = f"#{i+1}  {name}  [{joints_str}...]{grip_str}"
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, i)
            self.target_list.addItem(item)
        self.target_list.blockSignals(False)
        self.targets_changed.emit(len(self.targets))

    def _on_select(self, row):
        self._selected_idx = row

    def _context_menu(self, pos):
        item = self.target_list.itemAt(pos)
        if item is None:
            return
        idx = item.data(Qt.UserRole)

        menu = QMenu(self)
        menu.setStyleSheet(f"""
            QMenu {{ background: {P.DARK_PANEL}; color: {P.DARK_TEXT}; border: 1px solid {P.DARK_BORDER};
                    border-radius: 6px; font-size: 12px; padding: 4px; }}
            QMenu::item {{ padding: 6px 20px 6px 14px; border-radius: 4px; }}
            QMenu::item:selected {{ background: {P.DARK_ACCENT}; color: #1a1a16; }}
        """)
        rename_action = menu.addAction("Rename")
        delete_action = menu.addAction("Delete")
        action = menu.exec_(self.target_list.mapToGlobal(pos))
        if action == rename_action:
            name, ok = QInputDialog.getText(self, "Rename Target", "Name:", text=self.targets[idx]["name"])
            if ok and name:
                self.targets[idx]["name"] = name
                self._refresh_list()
        elif action == delete_action:
            self.targets.pop(idx)
            self._refresh_list()
            self.program_changed.emit(self)

    def _run_program(self):
        if not self.targets:
            return
        self._exec_thread = ProgramExecutionThread(
            self.targets, self._controller, self._connection_manager,
            direct_control=self.direct_control_checked
        )
        self._exec_thread.step_started.connect(self._on_step_started)
        self._exec_thread.program_completed.connect(self._on_program_completed)
        self._exec_thread.error_occurred.connect(self._on_step_error)
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self._exec_thread.start()

    def _stop_program(self):
        if self._exec_thread:
            self._exec_thread.stop()
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def _on_step_started(self, idx, name):
        self.target_list.setCurrentRow(idx)

    def _on_program_completed(self):
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.target_list.setCurrentRow(-1)

    def _on_step_error(self, idx, error):
        pass

    @property
    def direct_control_checked(self):
        return hasattr(self, '_direct_control_check') and self._direct_control_check.isChecked()

    def new_program(self):
        self.targets = []
        self._refresh_list()
        self._selected_idx = -1

    def save_program(self, path):
        import json
        data = {
            "name": "Program",
            "targets": self.targets,
            "params": {
                "speed": self.speed_spinbox.value(),
                "delay": self.delay_spinbox.value(),
                "transition": self.transition_spinbox.value(),
            }
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load_program(self, path):
        import json
        with open(path) as f:
            data = json.load(f)
        self.targets = data.get("targets", [])
        params = data.get("params", {})
        if "speed" in params:
            self.speed_spinbox.setValue(params["speed"])
        if "delay" in params:
            self.delay_spinbox.setValue(params["delay"])
        if "transition" in params:
            self.transition_spinbox.setValue(params["transition"])
        self._refresh_list()
