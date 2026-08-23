"""Robot Library panel — import and manage custom robots.

Ported from Arctos Studio's robot model:

- Robots live as folders under ``assets/robots/<Name>/{urdf,meshes,config}``
  (a ROS-style package). The panel *scans* that folder — there is no
  registry database.
- Thumbnails are cached centrally in ``assets/robots/thumbs/<hash>.png``,
  keyed by a hash of the robot's URDF path (Arctos's approach), so the
  same robot always maps to the same thumbnail.
- Mesh references in the URDF — ``../meshes/...``, ``meshes/...``, or
  ROS ``package://<pkg>/...`` — are resolved to the copied ``meshes/``
  folder on import, so the library copy is self-contained.
- A ``config/joint_names_<name>.yaml`` (ROS convention) is read if present
  to order the joints; otherwise URDF joint order is used.
- Per-robot control tuning (gear ratios, inverted axes, gripper commands)
  is stored in ``assets/robots/robot_preferences.json`` keyed by robot
  name, matching Arctos's ``settings/robot_preferences.json``.
"""

import hashlib
import json
import os
import re
import shutil

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLabel,
    QFileDialog, QMessageBox, QScrollArea, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QFont

from . import palette as P

# Bundled robots ship in the package assets (read-only in a frozen exe).
BUNDLED_ROBOTS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "robots")

# User-imported robots + prefs live in a writable per-user dir so they work
# when frozen (sys._MEIPASS is read-only at runtime).
def _user_robots_dir():
    try:
        from main import user_data_dir
        return os.path.join(user_data_dir(), "robots")
    except Exception:
        return BUNDLED_ROBOTS_DIR

ROBOTS_DIR = _user_robots_dir()
THUMBS_DIR = os.path.join(ROBOTS_DIR, "thumbs")
PREFS_FILE = os.path.join(ROBOTS_DIR, "robot_preferences.json")

MESH_EXTS = (".dae", ".stl", ".obj", ".STL", ".DAE", ".OBJ", ".ply", ".PLY")


# ── Helpers ─────────────────────────────────────────────────────────

def _sanitize_name(base):
    """Turn a robot name into a safe folder name."""
    return re.sub(r"[^\w\- ]", "", base).strip().replace(" ", "_") or "Imported_Robot"


def _thumb_path_for(urdf_path):
    """Central hash-keyed thumbnail path (Arctos-style)."""
    h = hashlib.md5(urdf_path.replace("\\", "/").encode("utf-8")).hexdigest()
    return os.path.join(THUMBS_DIR, f"{h}.png")


def _scan_folder(folder, builtin_names):
    entries = []
    if not os.path.isdir(folder):
        return entries
    for name in sorted(os.listdir(folder)):
        sub = os.path.join(folder, name)
        if not os.path.isdir(sub) or name == "thumbs":
            continue
        urdf = _find_primary_urdf(sub)
        if not urdf:
            continue
        entries.append({
            "name": name,
            "urdf": urdf,
            "thumb": _thumb_path_for(urdf),
            "joints": _count_joints(urdf),
            "builtin": name in builtin_names,
        })
    return entries


def _scan_robot_entries():
    """Scan bundled + user robot folders (Arctos scans folders).

    Bundled robots ship read-only in the package; user-imported robots live
    in the per-user writable dir. User robots win on name collisions.
    """
    entries = {}
    for e in _scan_folder(BUNDLED_ROBOTS_DIR, builtin_names={"astra", "Astra"}):
        entries[e["name"]] = e
    for e in _scan_folder(ROBOTS_DIR, builtin_names=set()):
        entries[e["name"]] = e  # user copy overrides bundled
    return list(entries.values())


def _find_primary_urdf(folder):
    """Find the primary URDF: prefer ``urdf/<name>.urdf``, else first .urdf."""
    name = os.path.basename(folder)
    cand = os.path.join(folder, "urdf", f"{name}.urdf")
    if os.path.exists(cand):
        return cand
    urdf_dir = os.path.join(folder, "urdf")
    if os.path.isdir(urdf_dir):
        for f in sorted(os.listdir(urdf_dir)):
            if f.lower().endswith(".urdf"):
                return os.path.join(urdf_dir, f)
    for f in sorted(os.listdir(folder)):
        if f.lower().endswith(".urdf"):
            return os.path.join(folder, f)
    return None


def _count_joints(urdf_path):
    try:
        from ..core.urdf_parser import URDFModel
        m = URDFModel.load(urdf_path)
        return sum(1 for j in m.joints.values() if j.is_movable)
    except Exception:
        return 0


def _read_joint_names(folder):
    """Read ``config/joint_names_*.yaml`` if present (Arctos joint order)."""
    config_dir = os.path.join(folder, "config")
    if not os.path.isdir(config_dir):
        return None
    for f in sorted(os.listdir(config_dir)):
        if f.lower().startswith("joint_names") and f.lower().endswith((".yaml", ".yml")):
            try:
                with open(os.path.join(config_dir, f), encoding="utf-8") as fh:
                    text = fh.read()
                # Parse:  controller_joint_names: ['', 'j1', 'j2', ...]
                m = re.search(r"\[([^\]]*)\]", text)
                if m:
                    names = [x.strip().strip("'\"") for x in m.group(1).split(",") if x.strip().strip("'\"")]
                    return names or None
            except Exception:
                return None
    return None


def _load_prefs():
    """Per-robot control tuning, keyed by robot name (Arctos robot_preferences)."""
    if os.path.exists(PREFS_FILE):
        try:
            with open(PREFS_FILE, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def _save_prefs(prefs):
    os.makedirs(ROBOTS_DIR, exist_ok=True)
    with open(PREFS_FILE, "w", encoding="utf-8") as f:
        json.dump(prefs, f, indent=2)


# ── Robot card ──────────────────────────────────────────────────────

class RobotCard(QFrame):
    """A clickable card showing a robot's thumbnail, name, and joint count."""

    clicked = pyqtSignal(str)

    def __init__(self, entry, parent=None):
        super().__init__(parent)
        self.entry = entry
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedHeight(150)
        self.setStyleSheet(P.card_style(radius=10, padding="8px"))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(6)

        self.thumb = QLabel()
        self.thumb.setFixedHeight(84)
        self.thumb.setAlignment(Qt.AlignCenter)
        self.thumb.setStyleSheet(
            f"background: {P.DARK_INPUT}; border-radius: 6px; border: 1px solid {P.DARK_BORDER_SOFT};"
        )
        thumb_path = entry.get("thumb")
        if thumb_path and os.path.exists(thumb_path):
            pm = QPixmap(thumb_path)
            if not pm.isNull():
                self.thumb.setPixmap(pm.scaled(120, 84, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.thumb.setText(entry.get("name", "?")[:2].upper())
            self.thumb.setStyleSheet(
                f"background: {P.DARK_INPUT}; color: {P.DARK_ACCENT}; border-radius: 6px; "
                f"border: 1px solid {P.DARK_BORDER_SOFT}; font-size: 28px; font-weight: bold;"
            )
        layout.addWidget(self.thumb)

        name = QLabel(entry.get("name", "Unknown"))
        name.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 13px; font-weight: bold; background: transparent; border: none;")
        name.setAlignment(Qt.AlignCenter)
        layout.addWidget(name)

        meta = QLabel(f"{entry.get('joints', '?')} axes" + ("  ·  built-in" if entry.get("builtin") else ""))
        meta.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        meta.setAlignment(Qt.AlignCenter)
        layout.addWidget(meta)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.entry.get("name", ""))
        super().mouseReleaseEvent(event)


# ── Library panel ───────────────────────────────────────────────────

class LibraryPanel(QWidget):
    """Grid of robot cards with an Import button."""

    robot_selected = pyqtSignal(str)  # robot name

    def __init__(self, parent=None):
        super().__init__(parent)
        self._entries = []
        self._setup_ui()
        self.refresh()

    # ── UI ─────────────────────────────────────────────────────────

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        header = QHBoxLayout()
        title = QLabel("Robot Library")
        title.setStyleSheet(f"color: {P.DARK_TEXT}; font-size: 15px; font-weight: bold; background: transparent; border: none;")
        header.addWidget(title)
        header.addStretch()

        self.import_btn = QPushButton("Import Robot…")
        self.import_btn.setStyleSheet(P.accent_btn_style(font_size=12))
        self.import_btn.clicked.connect(self.import_robot)
        header.addWidget(self.import_btn)
        layout.addLayout(header)

        hint = QLabel("Import a URDF robot (meshes are copied alongside). Click a card to load it into the scene.")
        hint.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 11px; background: transparent; border: none;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        self._cards_host = QWidget()
        self._cards_host.setStyleSheet("background: transparent;")
        self._cards_grid = QGridLayout(self._cards_host)
        self._cards_grid.setContentsMargins(0, 0, 0, 0)
        self._cards_grid.setSpacing(10)
        self._cards_grid.setColumnStretch(0, 1)
        self._cards_grid.setColumnStretch(1, 1)
        scroll.setWidget(self._cards_host)
        layout.addWidget(scroll, 1)

    def refresh(self):
        """Scan the robots folder and rebuild the card grid (Arctos-style)."""
        self._entries = _scan_robot_entries()
        while self._cards_grid.count():
            item = self._cards_grid.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
        if not self._entries:
            empty = QLabel("No robots yet. Press Import Robot… to add your first custom robot.")
            empty.setStyleSheet(f"color: {P.DARK_TEXT_MUTED}; font-size: 12px; background: transparent; border: none;")
            empty.setWordWrap(True)
            self._cards_grid.addWidget(empty, 0, 0, 1, 2)
            return
        for i, entry in enumerate(self._entries):
            card = RobotCard(entry)
            card.clicked.connect(self._on_card_clicked)
            self._cards_grid.addWidget(card, i // 2, i % 2)
        self._cards_grid.setRowStretch(len(self._entries) // 2 + 1, 1)

    # ── Import ─────────────────────────────────────────────────────

    def import_robot(self):
        """Open a URDF / STL / STEP / OBJ dialog and import it as a robot.

        - URDF: copied + meshes resolved (Arctos-style).
        - STL/STEP/OBJ: wrapped in a minimal single-link URDF so it can be
          loaded as a robot (e.g. a moving base / first link) and appears
          in the library.
        """
        path, _ = QFileDialog.getOpenFileName(
            self, "Import Robot", "",
            "Robot Models (*.urdf *.stl *.step *.stp *.obj);;"
            "URDF (*.urdf);;STL (*.stl);;STEP (*.step *.stp);;OBJ (*.obj);;All Files (*.*)"
        )
        if not path:
            return
        try:
            ext = os.path.splitext(path)[1].lower()
            if ext in (".stl", ".step", ".stp", ".obj"):
                name, joints = self._import_mesh_as_robot(path)
            else:
                name, joints = self._copy_into_library(path)
        except Exception as e:
            QMessageBox.warning(self, "Import Failed", f"Couldn't import the robot:\n{e}")
            return

        self.refresh()
        self.robot_selected.emit(name)
        QMessageBox.information(
            self, "Robot Imported",
            f"Imported '{name}' with {joints} joints.\n\n"
            f"It was copied to:\n{os.path.join(ROBOTS_DIR, name)}\n\n"
            "It is now loaded in the scene and available in the library."
        )

    def _import_mesh_as_robot(self, mesh_path):
        """Wrap an STL/STEP/OBJ into a minimal single-link URDF robot package.

        STEP files are converted to STL first (pybullet can't load .step in a
        URDF) using cadquery. Returns (name, joint_count).
        """
        base = os.path.splitext(os.path.basename(mesh_path))[0]
        name = _sanitize_name(base)

        dest = os.path.join(ROBOTS_DIR, name)
        dest_urdf_dir = os.path.join(dest, "urdf")
        dest_mesh_dir = os.path.join(dest, "meshes")
        os.makedirs(dest_urdf_dir, exist_ok=True)
        os.makedirs(dest_mesh_dir, exist_ok=True)

        ext = os.path.splitext(mesh_path)[1].lower()
        if ext in (".step", ".stp"):
            # Convert STEP -> STL via cadquery so pybullet can render it.
            mesh_name = f"{base}.stl"
            try:
                import cadquery as cq
                shape = cq.importers.importStep(mesh_path)
                cq.exporters.export(shape, os.path.join(dest_mesh_dir, mesh_name))
            except Exception as e:
                raise RuntimeError(f"Couldn't convert STEP to STL for the robot: {e}")
        else:
            mesh_name = os.path.basename(mesh_path)
            shutil.copy2(mesh_path, os.path.join(dest_mesh_dir, mesh_name))

        # Minimal single-link URDF (fixed base, no joints -> 0 movable joints).
        dest_urdf = os.path.join(dest_urdf_dir, f"{name}.urdf")
        with open(dest_urdf, "w", encoding="utf-8") as f:
            f.write(f"""<?xml version="1.0"?>
<robot name="{name}">
  <link name="base_link">
    <visual>
      <geometry><mesh filename="meshes/{mesh_name}" scale="0.001 0.001 0.001"/></geometry>
    </visual>
    <collision>
      <geometry><mesh filename="meshes/{mesh_name}" scale="0.001 0.001 0.001"/></geometry>
    </collision>
  </link>
</robot>
""")

        # Thumbnail (hash-keyed, Arctos-style)
        os.makedirs(THUMBS_DIR, exist_ok=True)
        thumb = _thumb_path_for(dest_urdf)
        self._write_thumbnail(name, thumb)

        joints = 0  # single fixed link
        return name, joints

    def _copy_into_library(self, urdf_path):
        """Copy a URDF + meshes into assets/robots/<Name>/ and make it self-contained.

        Returns (name, joint_count).
        """
        from ..core.urdf_parser import URDFModel

        model = URDFModel.load(urdf_path)
        base = model.name if getattr(model, "name", None) else os.path.splitext(os.path.basename(urdf_path))[0]
        name = _sanitize_name(base)

        dest = os.path.join(ROBOTS_DIR, name)
        dest_urdf_dir = os.path.join(dest, "urdf")
        dest_mesh_dir = os.path.join(dest, "meshes")
        os.makedirs(dest_urdf_dir, exist_ok=True)
        os.makedirs(dest_mesh_dir, exist_ok=True)

        # Resolve mesh files: relative ../meshes, meshes, or package:// layouts.
        src_dir = os.path.dirname(os.path.abspath(urdf_path))
        mesh_locations = self._collect_mesh_locations(urdf_path, src_dir)
        copied = {}
        for rel_path, abs_path in mesh_locations.items():
            if os.path.isfile(abs_path):
                dest_file = os.path.join(dest_mesh_dir, os.path.basename(rel_path))
                shutil.copy2(abs_path, dest_file)
                copied[rel_path] = os.path.basename(rel_path)

        # Copy URDF, rewriting mesh references to the copied meshes/ folder.
        dest_urdf = os.path.join(dest_urdf_dir, f"{name}.urdf")
        self._copy_urdf_rewriting_meshes(urdf_path, dest_urdf, copied)

        # Copy a ROS joint-names config if present.
        config_dir = os.path.join(src_dir, "config")
        if os.path.isdir(config_dir):
            dest_config = os.path.join(dest, "config")
            os.makedirs(dest_config, exist_ok=True)
            for f in os.listdir(config_dir):
                if f.lower().startswith("joint_names"):
                    shutil.copy2(os.path.join(config_dir, f), os.path.join(dest_config, f))

        # Thumbnail: hash-keyed, cached centrally (Arctos-style).
        os.makedirs(THUMBS_DIR, exist_ok=True)
        thumb = _thumb_path_for(os.path.join(dest_urdf_dir, f"{name}.urdf"))
        self._write_thumbnail(name, thumb)

        joints = _count_joints(dest_urdf)
        return name, joints

    @staticmethod
    def _collect_mesh_locations(urdf_path, src_dir):
        """Return {mesh_rel_path: absolute_path} for every mesh the URDF references.

        Handles ``../meshes/x``, ``meshes/x``, and ROS ``package://<pkg>/...``.
        For ``package://`` we first try exact path matches under the robot's
        source dir and the app's robots tree; if that fails we fall back to
        finding the file by basename anywhere under the source ``meshes/``
        tree (handles ``.../meshes/<Name>/visual/x.stl`` layouts).
        """
        locations = {}
        with open(urdf_path, encoding="utf-8", errors="replace") as f:
            text = f.read()

        # Index all mesh files under the source folder for basename fallback.
        basename_index = {}
        for dirpath, _dirs, files in os.walk(src_dir):
            for fn in files:
                if fn.lower().endswith(MESH_EXTS):
                    basename_index.setdefault(fn, os.path.join(dirpath, fn))

        for m in re.finditer(r'filename="([^"]+)"', text):
            raw = m.group(1)
            if not any(raw.lower().endswith(ext) for ext in MESH_EXTS):
                continue
            candidates = []
            if raw.startswith("package://"):
                pkg_path = raw[len("package://"):]          # pkg/meshes/x.stl
                tail = pkg_path.split("/", 1)[1] if "/" in pkg_path else pkg_path
                # 1) under the robot's source folder
                candidates.append(os.path.join(src_dir, tail))
                # 2) under the app's robots tree (sibling robot reference)
                candidates.append(os.path.join(ROBOTS_DIR, tail))
                # 3) basename anywhere under source (Arctos layouts)
                if os.path.basename(tail) in basename_index:
                    candidates.append(basename_index[os.path.basename(tail)])
            else:
                cleaned = re.sub(r"^\.\./", "", raw)
                candidates.append(os.path.normpath(os.path.join(src_dir, cleaned)))
                candidates.append(os.path.normpath(os.path.join(src_dir, "meshes", os.path.basename(cleaned))))
                if os.path.basename(cleaned) in basename_index:
                    candidates.append(basename_index[os.path.basename(cleaned)])
            found = next((c for c in candidates if os.path.isfile(c)), None)
            if found:
                locations[raw] = found
        return locations

    @staticmethod
    def _copy_urdf_rewriting_meshes(src_urdf, dest_urdf, copied):
        """Copy a URDF, rewriting every mesh filename to the copied meshes/ folder.

        ``copied`` maps the original mesh reference -> the basename now in
        ``assets/robots/<Name>/meshes/``.
        """
        with open(src_urdf, encoding="utf-8", errors="replace") as f:
            text = f.read()

        def _fix(m):
            raw = m.group(1)
            base = copied.get(raw)
            if base:
                return f'filename="meshes/{base}"'
            # Fallback: strip package:// and path prefixes, keep filename.
            path = re.sub(r"^package://[^/]+/", "", raw)
            path = re.sub(r"^\.\./", "", path)
            return f'filename="meshes/{os.path.basename(path)}"'

        text = re.sub(r'filename="([^"]+)"', _fix, text)
        with open(dest_urdf, "w", encoding="utf-8") as f:
            f.write(text)

    @staticmethod
    def _write_thumbnail(name, thumb_path):
        """Write a branded thumbnail PNG (placeholder, no rendering dependency)."""
        try:
            from PyQt5.QtGui import QPixmap, QPainter, QColor

            pm = QPixmap(240, 160)
            pm.fill(QColor(P.DARK_INPUT))
            painter = QPainter(pm)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setPen(QColor(P.DARK_ACCENT))
            painter.setFont(QFont("Segoe UI", 40, QFont.Bold))
            painter.drawText(pm.rect(), Qt.AlignCenter, name[:2].upper())
            painter.setPen(QColor(P.DARK_TEXT_DIM))
            painter.setFont(QFont("Segoe UI", 11))
            painter.drawText(pm.rect().adjusted(0, 60, 0, 0), Qt.AlignHCenter | Qt.AlignTop, name)
            painter.end()
            os.makedirs(os.path.dirname(thumb_path), exist_ok=True)
            pm.save(thumb_path)
        except Exception:
            pass

    # ── Public accessors ───────────────────────────────────────────

    def get_entries(self):
        """Return the scanned robot entries."""
        return list(self._entries)

    def get_prefs(self, name):
        """Per-robot tuning prefs (Arctos robot_preferences.json pattern)."""
        return _load_prefs().get(name, {})

    def set_prefs(self, name, prefs):
        """Store per-robot tuning prefs."""
        all_prefs = _load_prefs()
        all_prefs[name] = prefs
        _save_prefs(all_prefs)

    def _on_card_clicked(self, name):
        self.robot_selected.emit(name)
