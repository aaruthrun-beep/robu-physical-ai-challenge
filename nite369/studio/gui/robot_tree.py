from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QTreeWidget, QTreeWidgetItem, QLabel,
    QPushButton, QHBoxLayout, QMenu, QInputDialog, QLineEdit,
    QHeaderView,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor


class RobotTree(QWidget):
    object_selected = pyqtSignal(str)
    object_deleted = pyqtSignal(str)
    add_object_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(200)
        self.sim = None
        self._setup_ui()

    def set_simulation(self, sim):
        self.sim = sim

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(3)

        title_row = QHBoxLayout()
        title = QLabel("Scene Explorer")
        title.setStyleSheet("color: #aaa; font-size: 11px; font-weight: bold;")
        title_row.addWidget(title)
        title_row.addStretch()

        add_btn = QPushButton("+")
        add_btn.setFixedSize(20, 20)
        add_btn.setStyleSheet("""
            QPushButton { background: #2ECC71; color: white; border: none;
                         border-radius: 3px; font-size: 12px; font-weight: bold; }
            QPushButton:hover { background: #3DDB81; }
        """)
        add_btn.clicked.connect(self._show_add_menu)
        title_row.addWidget(add_btn)
        layout.addLayout(title_row)

        self.tree = QTreeWidget()
        self.tree.setHeaderHidden(True)
        self.tree.setIndentation(16)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._context_menu)
        self.tree.setStyleSheet("""
            QTreeWidget {
                background: #0d0d1a; color: #ccc; border: 1px solid #1a1a2e;
                border-radius: 3px; font-size: 11px; outline: none;
            }
            QTreeWidget::item { padding: 4px 2px; border-bottom: 1px solid #111128; }
            QTreeWidget::item:hover { background: #1a1a3a; }
            QTreeWidget::item:selected { background: #2a2a5a; color: #fff; }
        """)
        layout.addWidget(self.tree, 1)

        btn_btn = QHBoxLayout()
        for text, cb in [
            ("Expand All", self.tree.expandAll),
            ("Collapse", self.tree.collapseAll),
            ("Refresh", self._refresh),
        ]:
            btn = QPushButton(text)
            btn.setFixedHeight(22)
            btn.setStyleSheet("""
                QPushButton {
                    background: #1e1e3a; color: #aaa; border: 1px solid #2a2a5a;
                    border-radius: 3px; font-size: 9px; padding: 1px 6px;
                }
                QPushButton:hover { background: #2e2e5a; }
            """)
            btn.clicked.connect(cb)
            btn_btn.addWidget(btn)
        layout.addLayout(btn_btn)

    def _show_add_menu(self):
        menu = QMenu(self)
        menu.setStyleSheet("""
            QMenu { background: #1a1a2e; color: #ccc; border: 1px solid #2a2a4a; font-size: 10px; }
            QMenu::item { padding: 4px 16px 4px 8px; }
            QMenu::item:selected { background: #3a3a6a; }
        """)
        for label, type_name in [("Box", "box"), ("Cylinder", "cylinder"),
                                  ("Sphere", "sphere"), ("Workbench", "workbench"),
                                  ("Conveyor", "conveyor")]:
            action = menu.addAction(label)
            action.setData(type_name)
        action = menu.exec_(self.sender().mapToGlobal(self.sender().rect().bottomLeft()))
        if action:
            self.add_object_requested.emit(action.data())

    def _context_menu(self, pos):
        item = self.tree.itemAt(pos)
        if item is None:
            return
        menu = QMenu(self)
        menu.setStyleSheet("""
            QMenu { background: #1a1a2e; color: #ccc; border: 1px solid #2a2a4a; font-size: 10px; }
            QMenu::item { padding: 4px 16px 4px 8px; }
            QMenu::item:selected { background: #3a3a6a; }
        """)
        delete_action = menu.addAction("Delete")
        rename_action = menu.addAction("Rename")
        action = menu.exec_(self.tree.mapToGlobal(pos))
        if action == delete_action:
            name = item.text(0).split(" (")[0]
            self.object_deleted.emit(name)
        elif action == rename_action:
            name = item.text(0).split(" (")[0]
            new_name, ok = QInputDialog.getText(self, "Rename", "New name:", text=name)
            if ok and new_name:
                item.setText(0, new_name)

    def build_tree(self, robot_name="astra"):
        self.tree.clear()
        root = QTreeWidgetItem(self.tree, [f"Robot: {robot_name}"])
        root.setExpanded(True)
        root.setForeground(0, QColor("#4A9BE8"))

        if self.sim:
            info = self.sim.get_joint_info(robot_name)
        else:
            info = []

        joints_item = QTreeWidgetItem(root, [f"Joints ({len(info)})"])
        for j in info:
            jtype = {0: "Revolute", 1: "Prismatic", 4: "Fixed"}.get(j["type"], "Unknown")
            limits = f"[{j['lower_limit']:.2f}, {j['upper_limit']:.2f}]" if j["type"] != 4 else ""
            QTreeWidgetItem(joints_item, [f"  {j['name']}  ({jtype})  {limits}"])

        links_item = QTreeWidgetItem(root, [f"Links ({len(info) + 1})"])
        QTreeWidgetItem(links_item, ["base_link"])
        for j in info:
            if j["parent_index"] >= 0:
                QTreeWidgetItem(links_item, [f"link_{j['index']}"])

        ee_item = QTreeWidgetItem(root, ["End Effector"])
        pose = self.sim.get_endeffector_pose(robot_name) if self.sim else None
        if pose:
            pos = pose["position"]
            QTreeWidgetItem(ee_item, [f"Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"])

        scene_item = QTreeWidgetItem(self.tree, ["Scene Objects"])
        if self.sim and self.sim.bodies:
            for name in sorted(self.sim.bodies.keys()):
                if name != "ground":
                    pos = self.sim.get_body_position(name)
                    if pos:
                        QTreeWidgetItem(scene_item, [f"{name}  ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"])
                    else:
                        QTreeWidgetItem(scene_item, [name])
        else:
            QTreeWidgetItem(scene_item, ["<empty>"])

        poses_item = QTreeWidgetItem(self.tree, ["Saved Poses"])
        if self.sim:
            poses = self.sim.get_saved_poses()
            for name in sorted(poses.keys()):
                QTreeWidgetItem(poses_item, [f"{name}"])
        if poses_item.childCount() == 0:
            QTreeWidgetItem(poses_item, ["<none>"])

    def _refresh(self):
        self.build_tree()

    def update_pose(self, name):
        self._refresh()
