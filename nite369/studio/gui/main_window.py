"""Astra Studio Pro v1.0 — Advanced Robot Control IDE."""

import os
import sys
import time
import json
import math
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter,
    QMenuBar, QMenu, QAction, QStatusBar, QMessageBox, QFileDialog,
    QLabel, QPushButton, QApplication, QToolBar, QInputDialog,
    QSpinBox, QDoubleSpinBox, QShortcut, QTabWidget,
    QStackedWidget, QButtonGroup, QDockWidget, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QKeySequence, QColor, QPalette, QMouseEvent

from .threejs_viewport import ThreeJSViewport
from ..control.urdf_viz_bridge import UrdfVizBridge
from .jog_panel import JointControlPanel
from .connection_panel import ConnectionPanel
from .messages_panel import MessagesPanel
from .program_editor import ProgramPanel
from .themes import ThemeManager
from .encoder_monitor import EncoderMonitorPanel
from .tmc_config import TMCConfigPanel
from .motion_config import MotionConfigPanel
from .system_monitor import SystemMonitorPanel
from .kinematic_config import KinematicConfigPanel
from .path_planning_panel import PathPlanningPanel
from .gripper_panel import GripperControlPanel
from .led_panel import LEDControlPanel
from .console_panel import ConsolePanel
from .robot_library import LibraryPanel
from .homing_panel import HomingPanel
from .macro_waypoint_panel import MacroWaypointPanel
from ..core import SimulationEngine, RobotModel, DHArm
from ..control import (
    CommandServer, create_default_handlers, RobotController,
    ConnectionManager, ConnectionMode,
)
from ..settings import Settings
from . import palette as P


APP_TITLE = "Nite 369"
APP_VERSION = "1.0.0"

# ── Dark theme (consistent with the "dark" palette in gui/palette.py) ─
DARK_STYLE = f"""
QMainWindow {{
    background-color: {P.DARK_BG};
}}
QToolBar {{
    background: {P.DARK_PANEL};
    border-bottom: 1px solid {P.DARK_BORDER};
    spacing: 6px;
    padding: 4px 8px;
    qproperty-iconSize: 18px 18px;
}}
QToolBar::separator {{
    background: {P.DARK_BORDER};
    width: 1px;
    margin: 4px 6px;
}}
QStatusBar {{
    background: {P.DARK_BG};
    color: {P.DARK_TEXT_DIM};
    font-size: 11px;
    border-top: 1px solid {P.DARK_BORDER};
    padding: 2px 8px;
}}
QStatusBar QLabel {{
    color: {P.DARK_TEXT_DIM};
    border: none;
    background: transparent;
    padding: 2px 8px;
}}
QLabel {{
    color: {P.DARK_TEXT};
    font-size: 12px;
    background: transparent;
    border: none;
}}
QGroupBox {{
    color: {P.DARK_TEXT};
    font-size: 12px;
    font-weight: bold;
    border: 1px solid {P.DARK_BORDER};
    border-radius: 4px;
    margin-top: 10px;
    padding: 14px 10px 10px 10px;
    background: {P.DARK_PANEL};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: {P.DARK_ACCENT};
}}
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: {P.DARK_INPUT};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
    padding: 4px 8px;
    font-size: 12px;
    selection-background-color: {P.DARK_ACCENT};
    selection-color: #1a1a16;
}}
QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover, QLineEdit:hover,
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{
    border: 1px solid {P.DARK_ACCENT};
}}
QComboBox::drop-down {{
    border: none;
    width: 18px;
}}
QComboBox QAbstractItemView {{
    background: {P.DARK_PANEL};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER};
    selection-background-color: {P.DARK_ACCENT};
    selection-color: #1a1a16;
    font-size: 12px;
}}
QPushButton {{
    background: {P.DARK_BUTTON};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER};
    border-radius: 4px;
    padding: 6px 14px;
    font-size: 12px;
    font-weight: bold;
    min-height: 20px;
}}
QPushButton:hover {{
    background: {P.DARK_BUTTON_HOVER};
    border: 1px solid {P.DARK_ACCENT};
}}
QPushButton:pressed {{
    background: {P.DARK_ACCENT};
    color: #1a1a16;
    padding-top: 2px;
}}
QPushButton:disabled {{
    background: {P.DARK_BUTTON};
    color: {P.DARK_TEXT_MUTED};
    border: 1px solid {P.DARK_BORDER_SOFT};
}}
QSlider::groove:horizontal {{
    background: {P.DARK_SLIDER_TRACK};
    height: 6px;
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
}}
QSlider::handle:horizontal {{
    background: {P.DARK_SLIDER_HANDLE};
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 8px;
}}
QSlider::handle:horizontal:hover {{
    background: {P.DARK_ACCENT_HOVER};
}}
QSlider::sub-page:horizontal {{
    background: {P.DARK_ACCENT};
    border-radius: 3px;
}}
QTextEdit {{
    background: {P.DARK_INPUT};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 12px;
    padding: 6px;
}}
QTreeWidget, QListWidget {{
    background: {P.DARK_INPUT};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
    font-size: 12px;
    outline: none;
    padding: 2px;
}}
QTreeWidget::item, QListWidget::item {{
    padding: 6px 8px;
    border-bottom: 1px solid {P.DARK_BORDER_SOFT};
}}
QTreeWidget::item:hover, QListWidget::item:hover {{
    background: {P.DARK_BUTTON_HOVER};
}}
QTreeWidget::item:selected, QListWidget::item:selected {{
    background: {P.DARK_ACCENT};
    color: #1a1a16;
}}
QTabWidget::pane {{
    border: 1px solid {P.DARK_BORDER};
    background: {P.DARK_PANEL};
    border-radius: 3px;
}}
QTabBar::tab {{
    background: {P.DARK_BUTTON};
    color: {P.DARK_TEXT_DIM};
    border: 1px solid {P.DARK_BORDER};
    padding: 6px 18px;
    font-size: 12px;
    font-weight: bold;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background: {P.DARK_PANEL};
    color: {P.DARK_ACCENT};
    border-bottom: 2px solid {P.DARK_ACCENT};
}}
QTabBar::tab:hover:!selected {{
    background: {P.DARK_BUTTON_HOVER};
    color: {P.DARK_TEXT};
}}
QCheckBox {{
    color: {P.DARK_TEXT};
    font-size: 12px;
    spacing: 6px;
    background: transparent;
    border: none;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
    background: {P.DARK_INPUT};
}}
QCheckBox::indicator:hover {{
    border: 1px solid {P.DARK_ACCENT};
}}
QCheckBox::indicator:checked {{
    background: {P.DARK_ACCENT};
    border: 1px solid {P.DARK_ACCENT};
}}
QMenu {{
    background: {P.DARK_PANEL};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
    font-size: 12px;
    padding: 4px;
}}
QMenu::item {{
    padding: 6px 24px 6px 16px;
    border-radius: 3px;
}}
QMenu::item:selected {{
    background: {P.DARK_ACCENT};
    color: #1a1a16;
}}
QMenu::separator {{
    background: {P.DARK_BORDER};
    height: 1px;
    margin: 4px 8px;
}}
QScrollBar:vertical {{
    background: {P.DARK_BG};
    width: 10px;
    border: none;
}}
QScrollBar::handle:vertical {{
    background: {P.DARK_BORDER};
    border-radius: 5px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{
    background: {P.DARK_TEXT_MUTED};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QMenuBar {{
    background: {P.DARK_BG};
    color: {P.DARK_TEXT};
    border-bottom: 1px solid {P.DARK_BORDER};
    padding: 2px;
    font-size: 12px;
}}
QMenuBar::item {{
    padding: 5px 12px;
    border-radius: 3px;
}}
QMenuBar::item:selected {{
    background: {P.DARK_ACCENT};
    color: #1a1a16;
}}
QProgressBar {{
    background: {P.DARK_INPUT};
    border: 1px solid {P.DARK_BORDER};
    border-radius: 3px;
    text-align: center;
    font-size: 11px;
    color: {P.DARK_TEXT};
    min-height: 16px;
}}
QProgressBar::chunk {{
    background: {P.DARK_ACCENT};
    border-radius: 3px;
}}
QToolTip {{
    background: {P.DARK_PANEL};
    color: {P.DARK_TEXT};
    border: 1px solid {P.DARK_ACCENT};
    padding: 4px 6px;
}}
"""


class _FloatingProgramOverlay(QWidget):
    """Floating, borderless, fully-transparent program panel.

    Sits on top of the 3D viewport. No frame, no background fill — the
    robot shows through everywhere except the panel widgets themselves.
    Drag the slim title strip to move it; drag the bottom-right corner
    handle to resize freely (no size limit).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.SubWindow)
        self._drag_offset = None      # (widget_pos - mouse_pos) while dragging
        self._resizing = False
        self._min_w, self._min_h = 120, 80
        self.setMinimumSize(self._min_w, self._min_h)

    # ── Drag-to-move (title strip) ───────────────────────────────────
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            pos = event.pos()
            # Bottom-right 24x24 corner = resize zone.
            if (pos.x() >= self.width() - 24 and pos.y() >= self.height() - 24):
                self._resizing = True
                self._drag_offset = event.globalPos() - self.frameGeometry().bottomRight()
            else:
                self._drag_offset = event.globalPos() - self.frameGeometry().topLeft()
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._drag_offset is None:
            return
        if self._resizing:
            br = event.globalPos() - self._drag_offset
            w = max(self._min_w, br.x() - self.frameGeometry().x())
            h = max(self._min_h, br.y() - self.frameGeometry().y())
            self.resize(w, h)
        else:
            self.move(event.globalPos() - self._drag_offset)
        event.accept()

    def mouseReleaseEvent(self, event):
        self._drag_offset = None
        self._resizing = False
        event.accept()
        super().mouseReleaseEvent(event)


class MainWindow(QMainWindow):
    # Thread-safe joystick command delivery: the joystick poll thread emits
    # this, Qt delivers it on the main thread immediately (no polling lag).
    _joy_cmd = pyqtSignal(str, object)

    def __init__(self):
        super().__init__()
        self.settings = Settings()
        self.sim = None
        self.server = None
        self.controller = None
        self.connection_manager = ConnectionManager(self)
        self.connection_manager.errorOccurred.connect(self._on_connection_error)
        # Joystick controller for robot jogging — CONTINUOUS (hold-to-run).
        # The controller runs on a background thread; its callbacks emit a
        # signal delivered on the Qt main thread immediately (no lag/stack).
        from ..control.joystick_controller import JoystickController
        self.joystick = JoystickController(step_deg=2.0, repeat_interval=0.12)
        self._joy_cmd.connect(self._apply_joy_cmd)
        self.joystick.on_jog_start = self._emit_joy_start
        self.joystick.on_jog_stop = self._emit_joy_stop
        # Live 3D URDF viewer (urdf-viz companion window).
        # DISABLED for the video demo — the embedded Three.js viewport
        # (ThreeJSViewport) shows the robot; don't pop the external Chrome
        # urdf-viz window on top.
        self.urdf_viz = UrdfVizBridge()
        # QTimer.singleShot(800, self.urdf_viz.start)
        self._theme = self.settings.get("theme", "dark")
        self._apply_stylesheet()
        self._setup_window()
        self._setup_central()
        self._setup_menu()
        self._setup_toolbar()
        self._setup_statusbar()
        self._setup_shortcuts()
        self._init_engine()
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(500)

    def _setup_window(self):
        self.setWindowTitle(f"{APP_TITLE}  v{APP_VERSION}")
        # Launch maximized (main.py calls showMaximized). The 1024x700 size
        # is only the fallback if maximization fails on some platforms.
        self.resize(1024, 700)
        self.setMinimumSize(900, 600)

    def _setup_menu(self):
        mb = self.menuBar()

        file_menu = mb.addMenu("&File")
        self._add_action(file_menu, "&New Session", "Ctrl+N", self._new_session)
        file_menu.addSeparator()
        self._add_action(file_menu, "Save Program", "Ctrl+S", self._save_program)
        self._add_action(file_menu, "Open Program...", "Ctrl+P", self._load_program)
        file_menu.addSeparator()
        import_menu = file_menu.addMenu("Import 3D &Model")
        self._add_action(import_menu, "Import &STL...", "", self._import_stl)
        self._add_action(import_menu, "Import S&TEP...", "", self._import_step)
        import_menu.addSeparator()
        self._add_action(import_menu, "&Clear All Meshes", "", self._clear_meshes)
        file_menu.addSeparator()
        self._add_action(file_menu, "E&xit", "Alt+F4", self.close)

        robot_menu = mb.addMenu("&Robot")
        self._add_action(robot_menu, "Import Robot (URDF/STL/STEP)...", "", self.library_panel.import_robot)
        self._add_action(robot_menu, "Build Robot from STL Parts...", "", self._build_robot_from_stl)
        self._add_action(robot_menu, "Show Library", "Ctrl+L", self._show_library)
        robot_menu.addSeparator()
        self._add_action(robot_menu, "Load Default Astra", "", self._reset_sim)

        sim_menu = mb.addMenu("&Simulation")
        self._add_action(sim_menu, "▶ Start", "F5", self._start_sim)
        self._add_action(sim_menu, "■ Pause", "F6", self._stop_sim)
        self._add_action(sim_menu, "↺ Reset", "F7", self._reset_sim)

        control_menu = mb.addMenu("&Control")
        self._add_action(control_menu, "Start Server", "", self._start_server)
        self._add_action(control_menu, "Stop Server", "", self._stop_server)
        control_menu.addSeparator()
        self._add_action(control_menu, "Test Connection", "", self._test_connection)

        view_menu = mb.addMenu("&View")
        for theme_name in ["retro", "light", "dark", "high_contrast"]:
            action = view_menu.addAction(f"Theme: {theme_name.replace('_', ' ').title()}")
            action.triggered.connect(lambda checked, t=theme_name: self._set_theme(t))
        view_menu.addSeparator()
        self._add_action(view_menu, "Reset Camera", "Home", self._reset_camera)
        self._add_action(view_menu, "Clear Imported Meshes", "", self._clear_meshes)
        # stl-orbit style viewer toggles (also available on the viewport overlay)
        self._act_wire = QAction("Wireframe", self)
        self._act_wire.setCheckable(True)
        self._act_wire.triggered.connect(lambda c: self.viewport.set_wireframe(c))
        view_menu.addAction(self._act_wire)
        self._act_grid = QAction("Show Grid", self)
        self._act_grid.setCheckable(True)
        self._act_grid.setChecked(True)
        self._act_grid.triggered.connect(lambda c: self.viewport.set_grid_visible(c))
        view_menu.addAction(self._act_grid)
        self._act_rotate = QAction("Auto-Rotate", self)
        self._act_rotate.setCheckable(True)
        self._act_rotate.triggered.connect(lambda c: self.viewport.set_auto_rotate(c))
        view_menu.addAction(self._act_rotate)
        self._add_action(view_menu, "Fit View", "F", self.viewport.fit_view)
        view_menu.addSeparator()
        for name, tab in [
            ("Connection", 0),
            ("Jog", 1),
            ("TMC", 2),
            ("Encoders", 3),
            ("Motion", 4),
            ("Homing", 5),
            ("System", 6),
            ("Path", 7),
            ("Kinematics", 8),
            ("Gripper", 9),
            ("LED", 10),
            ("Console", 11),
            ("Library", 12),
            ("Macros", 13),
        ]:
            view_menu.addAction(f"Go to: {name}").triggered.connect(
                lambda checked, t=tab: (self._tabs.setCurrentIndex(t),
                                        self._sync_tab_buttons(t))
            )

        help_menu = mb.addMenu("&Help")
        self._add_action(help_menu, "About", "", self._show_about)
        self._add_action(help_menu, "Keyboard Shortcuts", "", self._show_shortcuts)

    def _add_action(self, menu, text, shortcut, callback):
        a = QAction(text, self)
        if shortcut:
            a.setShortcut(shortcut)
        a.triggered.connect(callback)
        menu.addAction(a)
        return a

    def _setup_toolbar(self):
        tb = QToolBar("Main")
        tb.setMovable(False)
        tb.setIconSize(__import__('PyQt5.QtCore', fromlist=['QSize']).QSize(18, 18))
        tb.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self.addToolBar(Qt.TopToolBarArea, tb)

        def add_tool_button(text, tip, cb, style=None):
            btn = QPushButton(text)
            btn.setToolTip(tip)
            btn.setFixedHeight(34)
            btn.setStyleSheet(style or P.btn_style(P.DARK_BUTTON, font_size=11, padding="2px 12px"))
            btn.clicked.connect(cb)
            tb.addWidget(btn)
            return btn

        # ── File group ───────────────────────────────────────────
        add_tool_button("New", "New Session (Ctrl+N)", self._new_session)
        add_tool_button("Save", "Save Program (Ctrl+S)", self._save_program)
        add_tool_button("Open", "Open Program (Ctrl+P)", self._load_program)
        add_tool_button("Library", "Robot Library", self._show_library)
        tb.addSeparator()

        # ── Simulation group ──────────────────────────────────────
        add_tool_button("▶ Start", "Start Simulation (F5)", self._start_sim,
                        P.success_btn_style(font_size=11, padding="2px 12px"))
        add_tool_button("■ Pause", "Pause Simulation (F6)", self._stop_sim,
                        P.warning_btn_style(font_size=11, padding="2px 12px"))
        add_tool_button("↺ Reset", "Reset Simulation (F7)", self._reset_sim,
                        P.btn_style(P.DARK_BUTTON, font_size=11, padding="2px 12px"))
        tb.addSeparator()

        # ── Motion group ──────────────────────────────────────────
        for label, attr, default, suffix, width in [
            ("Speed:", "speed_spin", 50, "%", 60),
            ("Delay:", "delay_spin", 0.5, "s", 60),
            ("Trans:", "trans_spin", 1.5, "s", 60),
        ]:
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {P.DARK_TEXT_DIM}; font-size: 12px; font-weight: bold;")
            tb.addWidget(lbl)
            spin = QDoubleSpinBox()
            spin.setRange(0, 100 if "Speed" in label else 60)
            spin.setValue(default)
            spin.setFixedWidth(width)
            spin.setStyleSheet(P.input_style(font_size=11, padding="3px 6px"))
            setattr(self, attr, spin)
            tb.addWidget(spin)
            u = QLabel(suffix)
            u.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
            tb.addWidget(u)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        tb.addWidget(spacer)

        self.status_indicator = QLabel("Simulation Mode")
        self.status_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 13px; font-weight: bold;")
        tb.addWidget(self.status_indicator)

    def _setup_central(self):
        # ── Central: 6/10 viewport (left) + 4/10 control tabs (right) ─
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        main_split = QSplitter(Qt.Horizontal)
        main_split.setHandleWidth(0)
        main_split.setChildrenCollapsible(False)
        main_split.setStyleSheet(f"QSplitter::handle {{ background: {P.DARK_BORDER}; }}")

        # ── LEFT 6/10: viewport (top) + console (bottom 2/10) ─────
        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(1)

        self.viewport = ThreeJSViewport()
        self.viewport.set_connection_manager(self.connection_manager)
        self.viewport.mesh_selected.connect(self._on_mesh_selected)
        self.viewport.drag_finished.connect(self._on_ee_drag)
        left_layout.addWidget(self.viewport, 8)   # 8 parts (of 10)

        # Auto-load the bundled Nite369 URDF once the JS page is ready so
        # the viewport shows the articulated robot (meshes attach when the
        # user imports Base/Link1..Link6 STLs).
        try:
            import os as _os
            urdf_path = _os.path.join(
                _os.path.dirname(_os.path.dirname(__file__)),
                "gui", "stl_embed", "nite369.urdf")
            if _os.path.exists(urdf_path):
                self._load_urdf_into_viewport(urdf_path)
        except Exception:
            pass

        # Bottom console (2/10, under viewport only)
        self.messages_panel = MessagesPanel()
        self.messages_panel.setMinimumHeight(100)
        left_layout.addWidget(self.messages_panel, 2)   # 2 parts (of 10)

        # ── Route all robot serial traffic to the activity log ─────
        # Every command sent and every reply received from the robot
        # appears here, so the log is a complete record of the session.
        cm = self.connection_manager
        cm.messageReceived.connect(lambda msg: self.messages_panel.log(msg, "info"))
        cm.commandSent.connect(lambda msg: self.messages_panel.log(f">> {msg}", "command"))
        cm.errorOccurred.connect(lambda msg: self.messages_panel.log(msg, "error"))
        cm.connectionStateChanged.connect(
            lambda st: self.messages_panel.log(f"Connection: {st}", "command"))
        self._log_command = self.messages_panel.log
        main_split.addWidget(left_col)

        # ── RIGHT 4/10: Chrome-style control tabs ─────────────────
        self.connection_panel = ConnectionPanel(self.connection_manager)
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect_requested)

        self.program_panel = ProgramPanel()
        self.program_panel.set_simulation(self.sim)

        self.library_panel = LibraryPanel()
        self.library_panel.robot_selected.connect(self._load_robot_from_library)

        # Build control panels
        # Grouped so the tab bar stays readable: related panels sit together.
        page_configs = [
            ("Connect", self.connection_panel),
            ("Jog", JointControlPanel),
            ("Motion", MotionConfigPanel),
            ("Homing", HomingPanel),
            ("TMC", TMCConfigPanel),
            ("Encoders", EncoderMonitorPanel),
            ("System", SystemMonitorPanel),
            ("Path", PathPlanningPanel),
            ("Kinematics", KinematicConfigPanel),
            ("Gripper", GripperControlPanel),
            ("LED", LEDControlPanel),
            ("Console", ConsolePanel),
            ("Library", self.library_panel),
            ("Macros", MacroWaypointPanel),
        ]
        self._page_widgets = []
        # QStackedWidget + two-row toggle button bar instead of a paged
        # QTabWidget — all sections are visible, no left/right page arrows.
        self._tabs = QStackedWidget()

        # Build the two-row button tab bar. Rows of up to 7 buttons each so
        # everything fits without scrolling; add more sections by appending
        # to page_configs (the bar wraps automatically). Active tab uses the
        # brand green; hover uses brand blue.
        self._tab_buttons = []
        tab_cols = 7
        tab_rows_container = QWidget()
        rows_v = QVBoxLayout(tab_rows_container)
        rows_v.setContentsMargins(0, 0, 0, 4)
        rows_v.setSpacing(4)

        for row_start in range(0, len(page_configs), tab_cols):
            row_layout = QHBoxLayout()
            row_layout.setSpacing(4)
            for label, _ in page_configs[row_start:row_start + tab_cols]:
                btn = QPushButton(label)
                btn.setCheckable(True)
                btn.setFixedHeight(28)
                btn.setStyleSheet(
                    f"""
                    QPushButton {{
                        background: {P.DARK_BUTTON}; color: {P.DARK_TEXT_DIM};
                        border: 1px solid {P.DARK_BORDER}; border-radius: 4px;
                        font-size: 11px; font-weight: bold; padding: 0 4px;
                    }}
                    QPushButton:hover {{ background: {P.DARK_BUTTON_HOVER};
                        color: {P.DARK_ACCENT2}; border: 1px solid {P.DARK_ACCENT2}; }}
                    QPushButton:checked {{
                        background: {P.DARK_ACCENT}; color: #1a1a16;
                        border: 1px solid {P.DARK_ACCENT};
                    }}
                    """)
                btn.clicked.connect(
                    lambda checked, b=btn: self._select_tab_button(b))
                row_layout.addWidget(btn, 1)
                self._tab_buttons.append(btn)
            rows_v.addLayout(row_layout)
        # Make the first button the active tab.
        if self._tab_buttons:
            self._tab_buttons[0].setChecked(True)

        for i, (label, panel_or_cls) in enumerate(page_configs):
            panel = panel_or_cls() if isinstance(panel_or_cls, type) else panel_or_cls
            self._page_widgets.append(panel)
            self._tabs.addWidget(panel)

        self.connection_panel = self._page_widgets[0]
        self.jog_panel = self._page_widgets[1]
        self.motion_config = self._page_widgets[2]
        self.homing_panel = self._page_widgets[3]
        self.tmc_config = self._page_widgets[4]
        self.encoder_monitor = self._page_widgets[5]
        self.system_monitor = self._page_widgets[6]
        self.path_planning_panel = self._page_widgets[7]
        self.kinematic_config = self._page_widgets[8]
        self.grip_panel = self._page_widgets[9]
        self.led_panel = self._page_widgets[10]
        self.console_panel = self._page_widgets[11]
        self.macros_panel = self._page_widgets[13]

        # Wrap the two-row tab bar + stacked pages in one column for the split.
        self._tab_panel = QWidget()
        tab_panel_layout = QVBoxLayout(self._tab_panel)
        tab_panel_layout.setContentsMargins(0, 0, 0, 0)
        tab_panel_layout.setSpacing(0)
        tab_panel_layout.addWidget(tab_rows_container)
        tab_panel_layout.addWidget(self._tabs, 1)

        # Collapse toggle: lets the viewport take the full width. The button
        # sits in a slim header row above the tab bar.
        tab_header = QHBoxLayout()
        tab_header.setContentsMargins(4, 4, 4, 0)
        self._collapse_btn = QPushButton("≫  Hide Panels")
        self._collapse_btn.setCheckable(True)
        self._collapse_btn.setFixedHeight(22)
        self._collapse_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=10, padding="0px 8px"))
        self._collapse_btn.toggled.connect(self._toggle_tab_panel)
        tab_header.addStretch()
        tab_header.addWidget(self._collapse_btn)
        tab_panel_layout.insertLayout(0, tab_header)

        main_split.addWidget(self._tab_panel)

        # 8:2 proportion — the 3D viewport is the star; the control tabs
        # panel is a compact side bar.
        main_split.setStretchFactor(0, 8)
        main_split.setStretchFactor(1, 2)
        main_split.setSizes([1200, 400])
        # Cap the tabs panel so huge sizeHints can't crush the viewport.
        self._tab_panel.setMaximumWidth(460)
        self._main_split = main_split

        root.addWidget(main_split)

        # ── Panel wiring ───────────────────────────────────────────
        self.encoder_monitor.set_connection_manager(self.connection_manager)
        self.tmc_config.set_connection_manager(self.connection_manager)
        self.motion_config.set_connection_manager(self.connection_manager)
        self.homing_panel.set_connection_manager(self.connection_manager)
        self.system_monitor.set_connection_manager(self.connection_manager)
        self.path_planning_panel.trajectory_execute.connect(self._on_path_execute)
        self.kinematic_config.fk_updated.connect(self._on_kinematic_fk_updated)
        self.kinematic_config.robot_loaded.connect(self._on_robot_model_loaded)
        self.kinematic_config.fk_updated.connect(self._sync_jog_from_kinematics)
        self.jog_panel.set_connection_manager(self.connection_manager)
        self.jog_panel.home_requested.connect(self._on_jog_home)
        # Drive the embedded 3D robot from the Jog panel sliders in real time.
        self.jog_panel.joint_moved.connect(self._on_jog_joints)
        # World-frame translation jog -> validated JS PoE IK in the viewer.
        self.jog_panel.world_jog_requested.connect(self._on_world_jog)
        self.grip_panel.gripper_command.connect(self._on_gripper_command)
        # Keep the program editor's gripper value in sync with the gripper panel.
        self.grip_panel.gripper_position_changed.connect(self.program_panel.set_gripper)
        self.led_panel.led_command.connect(self._on_led_command)
        self.console_panel.set_connection_manager(self.connection_manager)
        self.macros_panel.set_connection_manager(self.connection_manager)

        # ── Translucent program overlay (over the viewport) ────────
        # Borderless + fully transparent: the robot shows through; only the
        # panel's own widgets paint. Drag the title strip to move, drag the
        # bottom-right corner to resize freely (no fixed width/height).
        self._program_overlay = _FloatingProgramOverlay(self.viewport)
        overlay_layout = QVBoxLayout(self._program_overlay)
        overlay_layout.setContentsMargins(8, 8, 8, 8)
        overlay_layout.setSpacing(6)
        overlay_title = QHBoxLayout()
        lbl = QLabel("Program")
        lbl.setStyleSheet(f"font-size: 13px; font-weight: bold; color: {P.DARK_ACCENT}; background: transparent; border: none;")
        overlay_title.addWidget(lbl)
        overlay_title.addStretch()
        self._overlay_close_btn = QPushButton("✕")
        self._overlay_close_btn.setFixedSize(22, 22)
        self._overlay_close_btn.setStyleSheet(
            f"QPushButton {{ background: {P.DARK_BUTTON}; color: {P.DARK_TEXT}; border: none; border-radius: 4px; font-size: 12px; font-weight: bold; }}"
            f"QPushButton:hover {{ background: {P.DARK_ERROR}; color: white; }}"
        )
        self._overlay_close_btn.clicked.connect(lambda: self._program_overlay.hide())
        overlay_title.addWidget(self._overlay_close_btn)
        overlay_layout.addLayout(overlay_title)
        overlay_layout.addWidget(self.program_panel, 1)
        # The web page's status bar is 24px tall at the top of the viewport —
        # start below it so the overlay never covers the J1-J6/XYZ readout.
        self._overlay_top_margin = 30
        self._program_overlay.move(8, self._overlay_top_margin)
        self._program_overlay.resize(380, int(self.viewport.height() * 0.6))
        self._program_overlay.show()

        # Auto-grow taller as targets are added to the program.
        self.program_panel.targets_changed.connect(self._grow_program_overlay)
        self._grow_program_overlay(len(self.program_panel.targets))

        # Keep overlay pinned to the left on viewport resize
        self.viewport._program_overlay = self._program_overlay

    def _select_tab_button(self, btn):
        """Activate the page matching a tab-bar button."""
        try:
            idx = self._tab_buttons.index(btn)
        except ValueError:
            return
        self._tabs.setCurrentIndex(idx)
        for i, b in enumerate(self._tab_buttons):
            b.setChecked(i == idx)

    def _toggle_tab_panel(self, checked):
        """Collapse/expand the right-side control panel so the 3D viewport
        gets the full width when the user wants to inspect the robot."""
        if checked:
            self._tab_panel.hide()
            self._collapse_btn.setText("≪  Show Panels")
        else:
            self._tab_panel.show()
            self._collapse_btn.setText("≫  Hide Panels")

    def _sync_tab_buttons(self, index):
        """Highlight the button for the current stacked page index."""
        for i, b in enumerate(self._tab_buttons):
            b.setChecked(i == index)

    def _show_library(self):
        """Switch to the Library tab."""
        self._tabs.setCurrentWidget(self.library_panel)
        if hasattr(self, "_tab_buttons"):
            idx = self._page_widgets.index(self.library_panel)
            self._sync_tab_buttons(idx)

    def _on_mesh_selected(self, index):
        """Show which imported mesh was clicked in the viewport."""
        meshes = self.viewport.get_meshes()
        if 0 <= index < len(meshes):
            m = meshes[index]
            self.statusBar().showMessage(
                f"Selected mesh: {m.name} — {len(m.faces)} faces", 4000
            )

    def _load_robot_from_library(self, name):
        """Load a robot from the library into the whole pipeline."""
        from .robot_library import _scan_robot_entries
        urdf = None
        for e in _scan_robot_entries():
            if e["name"] == name:
                urdf = e["urdf"]
                break
        if not urdf or not os.path.exists(urdf):
            self.log(f"Couldn't find the robot '{name}' on disk")
            return
        self._load_robot_urdf(urdf, name)

    def _build_robot_from_stl(self):
        """Open the multi-STL robot builder and load the built robot."""
        from .robot_builder import RobotBuilderDialog
        dlg = RobotBuilderDialog(self)
        if dlg.exec_() and getattr(dlg, "built_urdf", None):
            self.library_panel.refresh()
            self._load_robot_urdf(dlg.built_urdf, dlg.built_robot_name)

    def _setup_statusbar(self):
        sb = self.statusBar()
        self._status_robot = QLabel("  Robot: Astra 6-DOF")
        self._status_mode = QLabel("  Mode: Simulation")
        self._status_connection = QLabel("  Connection: None")
        self._status_joints = QLabel("  Joints: —")
        self._status_xyz = QLabel("  TCP: —")
        for w in [self._status_robot, self._status_mode, self._status_connection,
                  self._status_joints, self._status_xyz]:
            sb.addPermanentWidget(w)

        # ── Joystick status button (bottom-right) ────────────────
        # Dull = disconnected, GREEN = connected, RED = error.
        # Click to (re)connect / enable the joystick.
        self._joy_btn = QPushButton("●  JOYSTICK")
        self._joy_btn.setFixedWidth(130)
        self._joy_btn.setCursor(Qt.PointingHandCursor)
        self._joy_btn.setStyleSheet(
            "QPushButton { background: #3a4256; color: #8a93a8; border: none;"
            " border-radius: 10px; padding: 3px 10px; font-weight: bold; }"
            "QPushButton:hover { background: #4a5468; }")
        self._joy_btn.clicked.connect(self._toggle_joystick)
        sb.addPermanentWidget(self._joy_btn)

    def _setup_shortcuts(self):
        shortcuts = {
            "T": self._record_target,
            "R": self._run_program,
            "S": self._stop_program,
            "G": self._toggle_gizmo,
            "Delete": self._delete_target,
            "Escape": self._cancel_action,
        }
        for key, callback in shortcuts.items():
            QShortcut(QKeySequence(key), self).activated.connect(callback)

    def _init_engine(self):
        try:
            self.log("Starting the simulation engine…")
            self.sim = SimulationEngine(gui=False)
            self.sim.start()
            self.viewport.set_simulation(self.sim)
            self.viewport.start()
            self.log("Simulation engine is ready")

            robot_path = os.path.join(
                os.path.dirname(__file__), "..", "assets", "robots", "astra", "astra.urdf"
            )
            if os.path.exists(robot_path):
                self.sim.load_robot("astra", robot_path)
                self.sim.set_camera(2.0, 45, -30, [0, 0, 0.5])
                self.log("Astra robot loaded (6-DOF, 760 mm reach)")

            self.controller = RobotController(self.sim, self.connection_manager)
            self.program_panel.set_simulation(self.sim)
            self.program_panel.set_controller(self.controller)
            self.program_panel.set_connection_manager(self.connection_manager)
            self.jog_panel.set_simulation(self.sim)
            # Enable the World Jog (Cartesian IK) immediately with the default
            # Astra DH arm, so it works without loading the Kinematics panel.
            try:
                from astra_studio.core.kinematics import create_astra_dh
                self.jog_panel.configure_from_dh_arm(create_astra_dh())
            except Exception:
                pass
            self._status_mode.setText("  Mode: Ready")
            self.log("Astra Studio Pro is ready")
        except Exception as e:
            self.log(f"Astra Studio hit a problem while starting: {e}")

    def _on_connect_requested(self, protocol, params):
        success = False
        if protocol == "grbl":
            success = self.connection_manager.connect_grbl(
                params["port"], params.get("baud_rate", 115200)
            )
        elif protocol == "ethernet":
            success = self.connection_manager.connect_ethernet(
                params["host"], params["port"], params.get("timeout", 5.0)
            )
        elif protocol == "nite_ethernet":
            success = self.connection_manager.connect_nite_ethernet(
                params["host"], params.get("port", 23), params.get("timeout", 5.0)
            )
        elif protocol == "nite_serial":
            success = self.connection_manager.connect_nite_serial(
                params["port"], params.get("baud_rate", 115200)
            )
        else:
            success = False

        if success:
            label_map = {
                "grbl": "GRBL",
                "ethernet": "Ethernet",
                "nite_ethernet": "Nite 369 (Ethernet)",
                "nite_serial": "Nite 369 (Serial)",
            }
            display = label_map.get(protocol, protocol.upper())
            self.log(f"Connected via {display}")
            self._status_connection.setText(f"  Connection: {display}")
            self.status_indicator.setText(f"Connected ({display})")
            self.status_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 11px; font-weight: bold;")
            # Update sub-panels
            self.encoder_monitor.set_connected(True)
            self.tmc_config.set_connected(True)
            self.motion_config.set_connected(True)
        else:
            self.log(f"Couldn't connect using {protocol}. Check the port and settings, then try again.")

    def _on_connection_error(self, message):
        """Surface transport/protocol errors into the Activity Log."""
        self.log(message)

    def _on_disconnect_requested(self):
        self.connection_manager.disconnect()
        self.log("Disconnected")
        self._status_connection.setText("  Connection: None")
        self.status_indicator.setText("Simulation Mode")
        self.status_indicator.setStyleSheet(f"color: {P.DARK_SUCCESS}; font-size: 11px; font-weight: bold;")
        self.encoder_monitor.set_connected(False)
        self.tmc_config.set_connected(False)
        self.motion_config.set_connected(False)

    def _emergency_stop(self):
        if self.connection_manager.is_connected:
            self.connection_manager.stop()
            self.log("Emergency stop sent")
        if self.sim:
            self.sim.running = False
            self._status_mode.setText("  Mode: Stopped")

    def _unlock_robot(self):
        if self.connection_manager.is_connected:
            self.connection_manager.unlock()
            self.log("Robot unlocked")

    def _start_sim(self):
        if self.sim:
            self.sim.running = True
            self._status_mode.setText("  Mode: Running")
            self.log("Simulation started")

    def _stop_sim(self):
        if self.sim:
            self.sim.running = False
            self._status_mode.setText("  Mode: Paused")
            self.log("Simulation paused")

    def _reset_sim(self):
        if self.sim:
            self.sim.reset()
            robot_path = os.path.join(
                os.path.dirname(__file__), "..", "assets", "robots", "astra", "astra.urdf"
            )
            if os.path.exists(robot_path):
                self.sim.load_robot("astra", robot_path)
                self.sim.set_camera(2.0, 45, -30, [0, 0, 0.5])
            self._status_mode.setText("  Mode: Ready")
            self.log("Simulation reset")

    def _reset_camera(self):
        self.viewport.reset_view()
        self.log("Camera view reset")

    def _toggle_program_overlay(self):
        if self._program_overlay.isVisible():
            self._program_overlay.hide()
        else:
            # Center the overlay over the viewport, but keep it below the
            # top status bar (24px web bar + margin) so it never overlaps it.
            top = getattr(self, "_overlay_top_margin", 30)
            cx = self.viewport.width() // 2 - self._program_overlay.width() // 2
            cy = top + (self.viewport.height() - top) // 2 - self._program_overlay.height() // 2
            self._program_overlay.move(max(8, cx), max(top, cy))
            self._program_overlay.show()
            self._program_overlay.raise_()

    def _grow_program_overlay(self, target_count):
        """Grow the floating program overlay as targets are added.

        Each target row is ~34px; add the header/params/buttons and cap the
        height so it never fills the whole viewport below the status bar
        (the user can still drag the corner to make it bigger or smaller).
        """
        if not hasattr(self, "_program_overlay") or self._program_overlay is None:
            return
        vp_h = self.viewport.height()
        top = getattr(self, "_overlay_top_margin", 30)
        base = 170                      # title + params + run/stop row
        per_row = 34
        wanted = base + target_count * per_row
        capped = min(wanted, int((vp_h - top) * 0.92))
        current = self._program_overlay.height()
        if capped > current:            # only grow; never shrink below user size
            self._program_overlay.resize(self._program_overlay.width(), capped)

    def _record_target(self):
        self.program_panel._record_target()

    def _run_program(self):
        self.program_panel._run_program()

    def _stop_program(self):
        self.program_panel._stop_program()

    def _toggle_gizmo(self):
        self.viewport.select_end_effector()

    def _delete_target(self):
        self.program_panel._delete_target()

    def _cancel_action(self):
        self.viewport.gizmo.hide()
        self.viewport._selected_link = -1

    def _start_server(self):
        if self.server is None:
            try:
                port = self.settings["control"]["server_port"]
                self.server = CommandServer(port=port)
                self.server.set_simulation(self.sim)
                for name, handler in create_default_handlers(self.sim).items():
                    self.server.register_handler(name, handler)
                self.server.start()
                self.log(f"TCP command server started on port {port}")
            except Exception as e:
                self.log(f"Couldn't start the command server: {e}")

    def _stop_server(self):
        if self.server:
            self.server.stop()
            self.server = None
            self.log("Server stopped")

    def _test_connection(self):
        try:
            from ..control import RobotClient
            port = self.settings["control"]["server_port"]
            with RobotClient("127.0.0.1", port, timeout=3) as client:
                j = client.get_joints()
                positions = [f"{v:.2f}" for v in j.get("positions", [])]
                self.log(f"Server responded — joints: {positions}")
        except ConnectionRefusedError:
            self.log("Connection refused — start the server first")
        except Exception as e:
            self.log(f"Connection test failed: {e}")

    def _new_session(self):
        self._reset_sim()
        self.program_panel.new_program()
        self.log("New session started")

    def _load_robot(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Robot URDF", "", "URDF (*.urdf);;All (*.*)"
        )
        if path and self.sim:
            name = os.path.basename(path).replace(".urdf", "").replace(".URDF", "")
            self._load_robot_urdf(path, name)

    def _load_robot_urdf(self, urdf_path, name):
        """Load a URDF robot into the simulation + kinematics pipeline."""
        if not self.sim:
            return
        try:
            self.sim.load_robot(name, urdf_path)
            self._status_robot.setText(f"  Robot: {name}")
            self.log(f"Loaded robot: {name}")
            # Also load into kinematics panel
            self.kinematic_config.load_robot_from_urdf(urdf_path)
            # Switch to the Kinematics tab so the user sees the loaded config
            self._tabs.setCurrentWidget(self.kinematic_config)
            idx = self._page_widgets.index(self.kinematic_config)
            self._sync_tab_buttons(idx)
        except Exception as e:
            self.log(f"Couldn't load the robot '{name}': {e}")

    def _save_program(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Program", "", "Robot Program (*.rp);;All (*.*)"
        )
        if path:
            if not path.endswith(".rp"):
                path += ".rp"
            self.program_panel.save_program(path)
            self.log(f"Program saved: {os.path.basename(path)}")

    def _load_program(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Program", "", "Robot Program (*.rp);;All (*.*)"
        )
        if path:
            try:
                self.program_panel.load_program(path)
                self.log(f"Program loaded: {os.path.basename(path)}")
            except Exception as e:
                self.log(f"Couldn't open that program: {e}")

    def _import_stl(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Import STL Model", "",
            "STL Files (*.stl *.STL);;All Files (*.*)"
        )
        if path:
            self._import_mesh_background(path, "STL")

    def _import_step(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Import STEP Model", "",
            "STEP Files (*.step *.stp *.STEP *.STP);;All Files (*.*)"
        )
        if path:
            self._import_mesh_background(path, "STEP")

    def _import_mesh_background(self, path, kind):
        """Load a mesh off the UI thread, then add it to the viewport."""
        from PyQt5.QtCore import QThread, pyqtSignal, QObject

        class _Loader(QThread):
            done = pyqtSignal(object)
            failed = pyqtSignal(str)

            def __init__(self, p):
                super().__init__()
                self._p = p

            def run(self):
                try:
                    from ..core.mesh_loader import load_mesh
                    self.done.emit(load_mesh(self._p))
                except Exception as e:
                    self.failed.emit(str(e))

        self.statusBar().showMessage(f"Loading {kind}…", 0)
        self._mesh_loader = _Loader(path)
        self._mesh_loader.done.connect(
            lambda mesh: self._on_mesh_loaded(mesh, path, kind)
        )
        self._mesh_loader.failed.connect(
            lambda err: self._on_mesh_failed(path, kind, err)
        )
        self._mesh_loader.start()

    def _on_mesh_loaded(self, mesh, path, kind):
        self.statusBar().clearMessage()
        try:
            self.viewport.add_mesh(mesh)
            n_faces = len(mesh.faces)
            self.log(f"Imported {kind}: {os.path.basename(path)} ({n_faces} faces)")
            self.statusBar().showMessage(
                f"Imported: {os.path.basename(path)} — {n_faces} faces", 5000
            )
        except Exception as e:
            self.log(f"Couldn't import the {kind} file: {e}")
            QMessageBox.warning(self, "Import Error", f"Failed to import {kind}:\n{e}")

    def _on_mesh_failed(self, path, kind, err):
        self.statusBar().clearMessage()
        self.log(f"Couldn't import the {kind} file: {err}")
        QMessageBox.warning(self, "Import Error", f"Failed to import {kind}:\n{err}")

    def _clear_meshes(self):
        self.viewport.clear_meshes()
        self.log("All imported meshes cleared")

    def _show_settings(self):
        self.log("Settings — use the View menu to change the theme")

    def _set_theme(self, theme_name):
        self._theme = theme_name
        self.settings["theme"] = theme_name
        self.settings.save()
        self._apply_stylesheet()
        if hasattr(self, 'viewport'):
            self.viewport.set_theme(theme_name)
        self.log(f"Theme: {theme_name}")

    def _apply_stylesheet(self):
        if self._theme == "retro":
            self.setStyleSheet(ThemeManager.get_stylesheet("retro"))
        elif self._theme == "dark":
            self.setStyleSheet(DARK_STYLE)
        else:
            self.setStyleSheet(ThemeManager.get_stylesheet(self._theme))

    def _show_about(self):
        from .splash import load_brand_pixmap
        # Embed the AVANI logo + Tesla portrait in the HTML about box.
        def _data_uri(pm, fmt="PNG"):
            if pm is None:
                return ""
            from PyQt5.QtCore import QBuffer, QIODevice
            import base64
            buf = QBuffer()
            buf.open(QIODevice.WriteOnly)
            pm.save(buf, fmt)
            return "data:image/png;base64," + base64.b64encode(bytes(buf.data())).decode()
        logo_uri = _data_uri(load_brand_pixmap("Gemini_Generated_Image", 180, invert=True))
        tesla_uri = _data_uri(load_brand_pixmap("images (19)", 180, invert=True))
        imgs = ""
        if logo_uri:
            imgs += f'<img src="{logo_uri}" height="90" style="margin-right:14px;">'
        if tesla_uri:
            imgs += f'<img src="{tesla_uri}" height="110" style="vertical-align:middle;">'
        QMessageBox.about(self, APP_TITLE,
            f"<table><tr><td>{imgs}</td></tr></table>"
            f"<h2>{APP_TITLE} v{APP_VERSION}</h2>"
            "<p>Advanced Robotics Simulation &amp; Control IDE</p>"
            "<p>6-DOF Articulated Arm · GRBL / Custom Firmware / Ethernet</p>"
            "<p><i>Avani Dynamics</i></p>"
            "<hr>"
            "<p>Simulation: PyBullet · UI: PyQt5</p>"
        )

    def _show_shortcuts(self):
        QMessageBox.information(self, "Keyboard Shortcuts",
            "<b>Program</b><br>"
            "T &nbsp;&nbsp;&nbsp; Record Target<br>"
            "R &nbsp;&nbsp;&nbsp; Run Program<br>"
            "S &nbsp;&nbsp;&nbsp; Stop Program<br>"
            "Del &nbsp; Delete Target<br>"
            "<br><b>Simulation</b><br>"
            "F5 &nbsp;&nbsp; Start<br>"
            "F6 &nbsp;&nbsp; Pause<br>"
            "F7 &nbsp;&nbsp; Reset<br>"
            "<br><b>Viewport</b><br>"
            "L-drag &nbsp; Orbit<br>"
            "R-drag &nbsp; Pan<br>"
            "Scroll &nbsp;&nbsp; Zoom<br>"
            "G &nbsp;&nbsp;&nbsp;&nbsp; Gizmo mode<br>"
            "Esc &nbsp;&nbsp;&nbsp; Cancel"
        )

    def _get_active_robot(self):
        """Get the name of the first available robot in the simulation."""
        if self.sim and self.sim.robots:
            for name in self.sim.robots:
                return name
        return None

    def _load_urdf_into_viewport(self, urdf_path):
        """Load the Nite369 URDF into the embedded Three.js viewer."""
        try:
            with open(urdf_path, "r", encoding="utf-8") as f:
                urdf_text = f.read()
            self.viewport.load_urdf(urdf_text)
            self.log(f"URDF loaded into viewport: {os.path.basename(urdf_path)}")
        except Exception as e:
            self.log(f"URDF viewport load failed: {e}")

    def _on_jog_joints(self, values_deg):
        """Drive the embedded 3D robot from the Jog panel sliders."""
        try:
            self.viewport.set_joints(list(values_deg)[:6])
        except Exception:
            pass

    def _set_joy_state(self, state):
        """off -> dull grey, on -> green, error -> red."""
        if state == "on":
            self._joy_btn.setStyleSheet(
                "QPushButton { background: #7CB342; color: #0c0f14;"
                " border: none; border-radius: 10px; padding: 3px 10px;"
                " font-weight: bold; }"
                "QPushButton:hover { background: #8fc55a; }")
            self._joy_btn.setText("●  JOYSTICK")
        elif state == "error":
            self._joy_btn.setStyleSheet(
                "QPushButton { background: #ff5c7a; color: #ffffff;"
                " border: none; border-radius: 10px; padding: 3px 10px;"
                " font-weight: bold; }"
                "QPushButton:hover { background: #ff7a92; }")
            self._joy_btn.setText("✖  JOYSTICK")
        else:
            self._joy_btn.setStyleSheet(
                "QPushButton { background: #3a4256; color: #8a93a8;"
                " border: none; border-radius: 10px; padding: 3px 10px;"
                " font-weight: bold; }"
                "QPushButton:hover { background: #4a5468; }")
            self._joy_btn.setText("●  JOYSTICK")

    def _toggle_joystick(self):
        """Click the status button to (re)connect / enable the joystick."""
        if getattr(self, "joystick", None) is None:
            return
        if self.joystick.connected:
            # Disable: stop the poll thread AND reset state so a later
            # click can cleanly reconnect.
            self.joystick.disconnect()
            self.joystick.stop()
            self._set_joy_state("off")
            self.statusBar().showMessage("Joystick disabled", 2000)
        else:
            # Enable: do a FULL clean restart (stop any stale thread first).
            self.joystick.stop()
            self.joystick.running = False
            self.joystick._thread = None
            self.joystick.start()          # spawns a fresh poll thread
            self.joystick.connect()        # init pygame + find device
            st = self.joystick.status()
            if st.get("connected"):
                self._set_joy_state("on")
                self.statusBar().showMessage(
                    f"Joystick connected: {st.get('name')}", 3000)
            else:
                self._set_joy_state("error")
                self.statusBar().showMessage(
                    "Joystick not found — connect one and click again", 3000)

    def _emit_joy_start(self, joint, direction):
        """Thread-safe: called from the joystick poll thread; emit signal."""
        try:
            self._joy_cmd.emit("start", (joint, direction))
        except Exception:
            pass

    def _emit_joy_stop(self):
        """Thread-safe: called from the joystick poll thread; emit signal."""
        try:
            self._joy_cmd.emit("stop", None)
        except Exception:
            pass

    def _apply_joy_cmd(self, kind, payload):
        """Main thread: apply the continuous-jog command immediately."""
        try:
            if kind == "stop":
                if (self.connection_manager and
                        self.connection_manager.is_connected):
                    self.connection_manager.jog_stop()
            else:
                joint, direction = payload
                self._on_joystick_jog(joint, direction)
        except Exception:
            pass

    def _on_joystick_jog(self, joint, direction):
        """Start CONTINUOUS jog of a joint (hold-to-run) on the real robot.

        #JC<joint>,<dir>,<speed> runs the motor continuously until #H —
        true continuous motion with the firmware's own accel/decel ramps.
        """
        if getattr(self, "connection_manager", None) is None:
            return
        try:
            if self.connection_manager.is_connected:
                # 1-based joint for the robot command.
                self.connection_manager.jog_start(joint + 1, direction)
        except Exception:
            pass

    def _on_world_jog(self, dx_mm, dy_mm, dz_mm):
        """World-frame translation jog -> embedded viewer's validated IK."""
        try:
            self.viewport.jog_world(dx_mm, dy_mm, dz_mm)
        except Exception:
            pass

    def _on_ee_drag(self, new_joints_deg):
        """End-effector gizmo drag completed — update sim + send move to robot."""
        # Compute deltas from the panel's current joints BEFORE updating.
        old = list(self.jog_panel.values[:6])
        deltas = [new_joints_deg[i] - old[i] for i in range(min(len(new_joints_deg), len(old)))]
        while len(deltas) < 6:
            deltas.append(0.0)
        # Always update the jog panel + simulation, so the frame can be used
        # to manipulate the model even without a robot connected.
        try:
            self.jog_panel.set_joints(new_joints_deg)
            self.jog_panel._apply_joints()
        except Exception:
            pass
        # Send the coordinated move to the real robot if connected.
        if not self.connection_manager or not self.connection_manager.is_connected:
            return
        cm = self.connection_manager
        cm.coordinated_move(deltas[:6])

    def _on_kinematic_fk_updated(self, joint_angles):
        """Apply FK joint angles from kinematics panel to simulation."""
        # Drive the urdf-viz 3D viewer (companion window).
        try:
            self.urdf_viz.set_joints(list(joint_angles)[:6])
        except Exception:
            pass
        # Drive the embedded Three.js STL robot (articulate on first use).
        try:
            self.viewport.articulate()
            self.viewport.set_joints(list(joint_angles)[:6])
        except Exception:
            pass
        if not self.sim:
            return
        robot_name = self._get_active_robot()
        if not robot_name:
            return
        angles_rad = [math.radians(a) if max(abs(aa) for aa in joint_angles) > 6.28 else a
                     for a in joint_angles]
        self.sim.set_joint_positions(robot_name, angles_rad)
        self.log(f"Applied FK angles: {[f'{a:.1f}' for a in joint_angles]}")

    def _on_robot_model_loaded(self, robot_model):
        """Called when a new robot model is loaded in the kinematics panel."""
        if not robot_model:
            return

        # Load the robot into the simulation (3D model in viewport)
        if self.sim and robot_model.urdf_path and os.path.exists(robot_model.urdf_path):
            try:
                self.sim.load_robot(robot_model.name, robot_model.urdf_path)
                self.sim.set_camera(2.0, 45, -30, [0, 0, 0.5])
                self.log(f"Robot loaded into the simulation: {robot_model.name}")
            except Exception as e:
                self.log(f"Couldn't load the robot into the simulation: {e}")

        # Update status bar
        self._status_robot.setText(f"  Robot: {robot_model.name}")
        self.log(f"Robot model configured: {robot_model.name}")

        # Share DH arm with path planning panel
        dh_arm = self.kinematic_config.get_dh_arm()
        if dh_arm:
            self.path_planning_panel.set_dh_arm(dh_arm)

        # Configure jog panel with joint names/limits from the loaded robot
        if dh_arm:
            self.jog_panel.configure_from_dh_arm(dh_arm)

    def _on_path_execute(self, trajectory):
        """Execute a generated trajectory on the simulation."""
        if not self.sim or not self.sim.running:
            self.log("Start the simulation first (F5)")
            return
        n_pts = len(trajectory.points) if hasattr(trajectory, 'points') else 0
        if n_pts == 0:
            self.log("The trajectory is empty — nothing to execute")
            return
        self.log(f"Executing trajectory: {n_pts} points over {trajectory.total_time:.1f} s")
        # Animate the trajectory in simulation via a timer
        self._traj_index = 0
        self._trajectory = trajectory
        dt = trajectory.total_time / max(n_pts, 1)
        interval_ms = max(10, int(dt * 1000))
        if not hasattr(self, '_traj_timer'):
            from PyQt5.QtCore import QTimer
            self._traj_timer = QTimer()
            self._traj_timer.timeout.connect(self._step_trajectory)
        self._traj_timer.start(interval_ms)

    def _sync_jog_from_kinematics(self, joint_angles):
        """Sync jog panel joint values when FK is applied from kinematics panel."""
        if hasattr(self, 'jog_panel'):
            self.jog_panel.set_joints(joint_angles)

    def _on_jog_home(self):
        """Handle jog panel home request — reset robot to home everywhere."""
        robot_name = self._get_active_robot()
        if self.sim and robot_name:
            n = self.jog_panel.num_joints if hasattr(self, 'jog_panel') else 6
            self.sim.set_joint_positions(robot_name, [0.0] * n)
        # Return the jog panel + embedded viewport to home too, so the robot
        # visually snaps back immediately (not only after a slider nudge).
        try:
            self.jog_panel.set_joints([0.0] * self.jog_panel.num_joints)
        except Exception:
            pass
        try:
            self.viewport.home()
        except Exception:
            pass
        self.log("Robot returned to its home position")

    def _on_gripper_command(self, cmd):
        """Handle gripper command from GripperControlPanel."""
        if self.connection_manager.is_connected:
            self.connection_manager.send_command(cmd)
            self.log(f"Gripper command sent: {cmd}")
        else:
            self.log(f"Gripper command (not connected): {cmd}")

    def _on_led_command(self, cmd):
        """Handle LED command from LEDControlPanel."""
        if self.connection_manager.is_connected:
            self.connection_manager.send_command(cmd)
            self.log(f"LED command sent: {cmd}")
        else:
            self.log(f"LED command (not connected): {cmd}")

    def _step_trajectory(self):
        """Advance one step along the active trajectory."""
        if self._trajectory is None or self._traj_index >= len(self._trajectory.points):
            self._traj_timer.stop()
            self.log("Trajectory execution finished")
            self._trajectory = None
            return
        pt = self._trajectory.points[self._traj_index]
        robot_name = self._get_active_robot()
        if robot_name:
            self.sim.set_joint_positions(robot_name, pt.positions)
        self._traj_index += 1

    def _update_status(self):
        if not self.sim:
            return
        robot_name = self._get_active_robot()
        if robot_name:
            joints = self.sim.get_joint_positions(robot_name)
            rev = self.sim.get_revolute_joints(robot_name)
            joint_vals = [joints[j["index"]] for j in rev]
            txt = "  ".join(f"{v:5.1f}" for v in joint_vals[:6])
            self._status_joints.setText(f"  Joints: {txt}")
            pose = self.sim.get_endeffector_pose(robot_name)
            if pose:
                p = pose["position"]
                self._status_xyz.setText(f"  TCP: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

        if self.connection_manager.is_connected:
            status = self.connection_manager.get_status()
            state = status.get("state", "unknown")
            self._status_mode.setText(f"  Mode: {state.title()}")

    def log(self, msg):
        self.messages_panel.info(msg)

    def resizeEvent(self, event):
        """Keep the viewport:controls split at ~8:2 on window resize."""
        super().resizeEvent(event)
        if hasattr(self, "_main_split"):
            total = self._main_split.width()
            if total > 100:
                # 8:2 = viewport 80%, tabs 20%; tabs max 460 keeps viewport dominant.
                tabs_w = min(int(total * 0.2), 460)
                self._main_split.setSizes([max(total - tabs_w, 480), tabs_w])

    def closeEvent(self, event):
        self._status_timer.stop()
        self.viewport.stop()
        self._stop_server()
        self.connection_manager.disconnect()
        if self.sim:
            self.sim.stop()
        event.accept()
