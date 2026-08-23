"""RoboDK-style robot builder — assemble a robot from multiple STL parts.

The user adds STL files as links, sets each link's joint type (revolute /
prismatic / fixed), axis, limits, and an offset from its parent, then
"Build Robot" generates a multi-link URDF, copies the meshes into the
robot library, and loads it through the normal simulation pipeline.
"""

import os
import shutil

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
    QPushButton, QLabel, QLineEdit, QComboBox, QDoubleSpinBox,
    QFileDialog, QMessageBox, QRadioButton, QButtonGroup, QGroupBox,
    QFormLayout, QCheckBox,
)
from PyQt5.QtCore import Qt

from . import palette as P
from .robot_library import ROBOTS_DIR, _sanitize_name, _thumb_path_for, THUMBS_DIR, LibraryPanel

JOINT_TYPES = ["revolute", "prismatic", "fixed"]


class LinkRow:
    """Data holder for one link in the builder list."""

    def __init__(self, name="", mesh="", joint="revolute", axis=(0, 0, 1),
                 lo=-180.0, hi=180.0, offset=(0.0, 0.0, 0.0)):
        self.name = name
        self.mesh = mesh          # absolute path to the STL
        self.joint = joint        # revolute / prismatic / fixed
        self.axis = axis          # (x, y, z)
        self.lo = lo              # degrees (radians in URDF)
        self.hi = hi
        self.offset = offset      # (x, y, z) meters from parent


class RobotBuilderDialog(QDialog):
    """Wizard for building a multi-STL robot as a kinematic chain."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Build Robot from STL Parts")
        self.setModal(True)
        self.resize(680, 560)
        self.setStyleSheet(P.dock_style())

        self._links = []          # list[LinkRow]
        self._setup_ui()

    # ── UI ─────────────────────────────────────────────────────────

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title = QLabel("Build a robot from STL parts — each part becomes a link.")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 14px; font-weight: bold; background: transparent; border: none;")
        title.setWordWrap(True)
        layout.addWidget(title)

        hint = QLabel("Links connect in order: link 1 hangs off the base, link 2 off link 1, …")
        hint.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        # ── Link list ──────────────────────────────────────────────
        self.link_list = QListWidget()
        self.link_list.setStyleSheet(f"""
            QListWidget {{
                background: {P.DARK_INPUT}; color: {P.DARK_TEXT};
                border: 1px solid {P.DARK_BORDER}; border-radius: 6px;
                font-size: 12px; outline: none; padding: 4px;
            }}
            QListWidget::item {{ padding: 6px 8px; border-radius: 4px; }}
            QListWidget::item:selected {{ background: {P.DARK_ACCENT}; color: #1a1a16; }}
        """)
        self.link_list.currentRowChanged.connect(self._on_select)
        layout.addWidget(self.link_list, 1)

        # ── Link actions ───────────────────────────────────────────
        row = QHBoxLayout()
        for text, cb in [
            ("+ Add Link", self._add_link),
            ("− Remove", self._remove_link),
            ("↑", lambda: self._move(-1)),
            ("↓", lambda: self._move(1)),
        ]:
            b = QPushButton(text)
            b.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11, padding="3px 10px"))
            b.clicked.connect(cb)
            row.addWidget(b)
        row.addStretch()
        layout.addLayout(row)

        # ── Selected link editor ───────────────────────────────────
        self.editor = QGroupBox("Selected Link")
        self.editor.setStyleSheet(P.groupbox_style())
        form = QFormLayout(self.editor)
        form.setSpacing(6)

        name_row = QHBoxLayout()
        self.name_input = QLineEdit()
        self.name_input.setStyleSheet(P.input_style())
        name_row.addWidget(self.name_input, 1)
        self.mesh_btn = QPushButton("Browse STL…")
        self.mesh_btn.setStyleSheet(P.btn_style(P.DARK_BUTTON, font_size=11, padding="3px 10px"))
        self.mesh_btn.clicked.connect(self._browse_mesh)
        name_row.addWidget(self.mesh_btn)
        self.mesh_label = QLabel("")
        self.mesh_label.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        form.addRow("Name:", name_row)
        form.addRow("Mesh:", self.mesh_label)

        # Joint type
        jt_row = QHBoxLayout()
        self.jt_group = QButtonGroup(self)
        for text, val in [("Revolute", "revolute"), ("Prismatic", "prismatic"), ("Fixed", "fixed")]:
            rb = QRadioButton(text)
            rb.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 12px; background: transparent; border: none;")
            rb.setProperty("joint", val)
            self.jt_group.addButton(rb)
            jt_row.addWidget(rb)
        self.jt_group.buttonClicked.connect(self._on_joint_type)
        form.addRow("Joint:", jt_row)

        # Axis
        ax_row = QHBoxLayout()
        self.axis_combo = QComboBox()
        self.axis_combo.addItems(["X", "Y", "Z"])
        self.axis_combo.setStyleSheet(P.input_style())
        ax_row.addWidget(self.axis_combo)
        ax_row.addStretch()
        form.addRow("Axis:", ax_row)

        # Limits (degrees)
        lim_row = QHBoxLayout()
        self.lo_spin = QDoubleSpinBox()
        self.lo_spin.setRange(-360, 360)
        self.lo_spin.setValue(-180)
        self.lo_spin.setStyleSheet(P.input_style())
        lim_row.addWidget(self.lo_spin)
        lim_row.addWidget(QLabel("° to"))
        self.hi_spin = QDoubleSpinBox()
        self.hi_spin.setRange(-360, 360)
        self.hi_spin.setValue(180)
        self.hi_spin.setStyleSheet(P.input_style())
        lim_row.addWidget(self.hi_spin)
        lim_row.addWidget(QLabel("°"))
        lim_row.addStretch()
        form.addRow("Limits:", lim_row)

        # Offset from parent (meters)
        off_row = QHBoxLayout()
        self.ox = QDoubleSpinBox(); self.ox.setRange(-10, 10); self.ox.setDecimals(3); self.ox.setValue(0.0)
        self.oy = QDoubleSpinBox(); self.oy.setRange(-10, 10); self.oy.setDecimals(3); self.oy.setValue(0.0)
        self.oz = QDoubleSpinBox(); self.oz.setRange(-10, 10); self.oz.setDecimals(3); self.oz.setValue(0.0)
        for s in (self.ox, self.oy, self.oz):
            s.setStyleSheet(P.input_style())
            s.setSingleStep(0.05)
            off_row.addWidget(s)
        off_row.addStretch()
        form.addRow("Offset x,y,z (m):", off_row)

        layout.addWidget(self.editor)

        # ── Build row ──────────────────────────────────────────────
        build_row = QHBoxLayout()
        build_row.addWidget(QLabel("Robot name:"))
        self.robot_name = QLineEdit("my_robot")
        self.robot_name.setStyleSheet(P.input_style())
        build_row.addWidget(self.robot_name, 1)
        self.build_btn = QPushButton("Build Robot")
        self.build_btn.setStyleSheet(P.success_btn_style(font_size=13))
        self.build_btn.clicked.connect(self._build)
        build_row.addWidget(self.build_btn)
        layout.addLayout(build_row)

    # ── Link management ────────────────────────────────────────────

    def _add_link(self):
        idx = len(self._links)
        row = LinkRow(name=f"link{idx+1}")
        self._links.append(row)
        item = QListWidgetItem(f"{idx+1}. link{idx+1}  —  (no mesh)")
        self.link_list.addItem(item)
        self.link_list.setCurrentRow(idx)

    def _remove_link(self):
        idx = self.link_list.currentRow()
        if idx < 0:
            return
        self._links.pop(idx)
        self.link_list.takeItem(idx)

    def _move(self, d):
        idx = self.link_list.currentRow()
        j = idx + d
        if idx < 0 or j < 0 or j >= len(self._links):
            return
        self._links[idx], self._links[j] = self._links[j], self._links[idx]
        self._refresh_list()
        self.link_list.setCurrentRow(j)

    def _refresh_list(self):
        self.link_list.blockSignals(True)
        self.link_list.clear()
        for i, row in enumerate(self._links):
            mesh = os.path.basename(row.mesh) if row.mesh else "(no mesh)"
            self.link_list.addItem(f"{i+1}. {row.name}  —  {mesh}  ({row.joint})")
        self.link_list.blockSignals(False)

    def _on_select(self, idx):
        if idx < 0 or idx >= len(self._links):
            return
        row = self._links[idx]
        self.name_input.setText(row.name)
        self.mesh_label.setText(os.path.basename(row.mesh) if row.mesh else "No mesh selected")
        # Joint radio
        for rb in self.jt_group.buttons():
            rb.setChecked(rb.property("joint") == row.joint)
        axis_idx = [0, 1, 2].index([1, 0, 0] if row.axis == (1, 0, 0)
                                   else [0, 1, 0] if row.axis == (0, 1, 0)
                                   else [0, 0, 1]) if row.axis in ((1,0,0),(0,1,0),(0,0,1)) else 2
        self.axis_combo.setCurrentIndex(axis_idx)
        self.lo_spin.setValue(row.lo)
        self.hi_spin.setValue(row.hi)
        self.ox.setValue(row.offset[0])
        self.oy.setValue(row.offset[1])
        self.oz.setValue(row.offset[2])

    def _browse_mesh(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select STL for this link", "",
            "Mesh (*.stl *.obj *.STL *.OBJ);;All Files (*.*)"
        )
        if not path:
            return
        idx = self.link_list.currentRow()
        if idx < 0:
            return
        self._links[idx].mesh = path
        self._links[idx].name = self.name_input.text().strip() or os.path.splitext(os.path.basename(path))[0]
        self.mesh_label.setText(os.path.basename(path))
        self._refresh_list()
        self.link_list.setCurrentRow(idx)

    def _on_joint_type(self, btn):
        idx = self.link_list.currentRow()
        if idx >= 0:
            self._links[idx].joint = btn.property("joint")
            self._refresh_list()

    # ── Build ──────────────────────────────────────────────────────

    def _commit_edits(self):
        """Write the editor fields back to the selected link."""
        idx = self.link_list.currentRow()
        if idx < 0:
            return
        row = self._links[idx]
        row.name = self.name_input.text().strip() or row.name
        for rb in self.jt_group.buttons():
            if rb.isChecked():
                row.joint = rb.property("joint")
        ax = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[self.axis_combo.currentText()]
        row.axis = ax
        row.lo = self.lo_spin.value()
        row.hi = self.hi_spin.value()
        row.offset = (self.ox.value(), self.oy.value(), self.oz.value())

    def _build(self):
        self._commit_edits()
        if not self._links:
            QMessageBox.warning(self, "No Links", "Add at least one link first.")
            return
        for row in self._links:
            if not row.mesh:
                QMessageBox.warning(self, "Missing Mesh", f"Link '{row.name}' has no mesh.")
                return
        if row.lo > row.hi:
            QMessageBox.warning(self, "Bad Limits", f"Lower limit > upper limit on '{row.name}'.")

        name = _sanitize_name(self.robot_name.text().strip()) or "my_robot"
        dest = os.path.join(ROBOTS_DIR, name)
        dest_urdf_dir = os.path.join(dest, "urdf")
        dest_mesh_dir = os.path.join(dest, "meshes")
        os.makedirs(dest_urdf_dir, exist_ok=True)
        os.makedirs(dest_mesh_dir, exist_ok=True)

        # Copy meshes and build the URDF
        import math
        lines = [f'<?xml version="1.0"?>', f'<robot name="{name}">']
        mesh_files = []
        for i, row in enumerate(self._links):
            mesh_name = f"link{i+1}.stl"
            shutil.copy2(row.mesh, os.path.join(dest_mesh_dir, mesh_name))
            mesh_files.append(mesh_name)
            lines.append(f'  <link name="{row.name}">')
            lines.append('    <visual><geometry><mesh filename="meshes/{}" scale="0.001 0.001 0.001"/></geometry></visual>'.format(mesh_name))
            lines.append('    <collision><geometry><mesh filename="meshes/{}" scale="0.001 0.001 0.001"/></geometry></collision>'.format(mesh_name))
            lines.append('  </link>')

        # Joints: link i+1 connects from link i (first link is base/fixed)
        for i in range(1, len(self._links)):
            parent = self._links[i-1].name
            child = self._links[i].name
            joint = self._links[i].joint
            ax = self._links[i].axis
            off = self._links[i].offset
            lo_rad = math.radians(self._links[i].lo)
            hi_rad = math.radians(self._links[i].hi)
            lines.append(f'  <joint name="j{i}" type="{joint}">')
            lines.append(f'    <parent link="{parent}"/>')
            lines.append(f'    <child link="{child}"/>')
            lines.append(f'    <origin xyz="{off[0]} {off[1]} {off[2]}"/>')
            if joint != "fixed":
                lines.append(f'    <axis xyz="{ax[0]} {ax[1]} {ax[2]}"/>')
                lines.append(f'    <limit lower="{lo_rad:.6f}" upper="{hi_rad:.6f}" effort="10" velocity="1"/>')
            lines.append('  </joint>')
        lines.append('</robot>')

        dest_urdf = os.path.join(dest_urdf_dir, f"{name}.urdf")
        with open(dest_urdf, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        # Thumbnail
        os.makedirs(THUMBS_DIR, exist_ok=True)
        LibraryPanel._write_thumbnail(name, _thumb_path_for(dest_urdf))

        self.accept()
        # Signal the caller to load + refresh library
        self.built_robot_name = name
        self.built_urdf = dest_urdf
