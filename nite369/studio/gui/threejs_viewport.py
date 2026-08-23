"""ThreeJSViewport — embeds stl-orbit's Three.js STL viewer as the Studio viewport.

Replaces the pyqtgraph procedural viewport with the web-based Three.js
viewer (stl-orbit mechanism). STL meshes are passed from Python (trimesh ->
binary STL bytes -> JS ArrayBuffer) and articulated as a 6-axis robot, with
joint angles driven from the Python IK/FK pipeline.

Implements the same public API as the old Viewport3D so main_window works
unchanged:
  add_mesh, clear_meshes, get_meshes, set_simulation, start/stop,
  set_theme, reset_view, fit_view, set_wireframe, set_grid_visible,
  set_auto_rotate, set_joints, mesh_selected, gizmo, _program_overlay,
  select_end_effector, _selected_link, rect, mapTo, set_connection_manager
"""

import os
import struct
import json as _json
import functools
import numpy as np
from PyQt5.QtCore import Qt, QUrl, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtGui import QColor

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings
    _WEBENGINE = True
except Exception as _e:
    _WEBENGINE = False
    _WEBENGINE_ERR = repr(_e)
else:
    _WEBENGINE_ERR = None

# Chromium flags: force software WebGL + unblock the GPU so the Three.js
# scene renders even on machines without accelerated OpenGL (the usual
# cause of a black QtWebEngine viewport). Must be set before the first
# QWebEngineView is created.
if _WEBENGINE:
    try:
        from PyQt5.QtCore import QCoreApplication
        QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
        os.environ.setdefault(
            "QTWEBENGINE_CHROMIUM_FLAGS",
            "--ignore-gpu-blocklist --enable-unsafe-swiftshader "
            "--disable-gpu-driver-bug-workarounds --use-gl=angle "
            "--use-angle=swiftshader "
            # Stability: stop the network-service subprocess crash loop and
            # the GPU-cache access-denied spam on this machine.
            "--no-sandbox --disable-gpu-shader-disk-cache "
            "--disable-features=NetworkService,NetworkServiceInProcess "
            "--in-process-gpu")
    except Exception:
        pass


class _MeshStub:
    """Minimal mesh wrapper the JS viewer understands.

    Keeps the same shape as mesh_loader.LoadedMesh (name, vertices, faces)
    so existing import code works unchanged. Vertices are preserved at real
    scale — no normalization — so imported parts keep their true size and
    the URDF's meter-based assembly stays correct.
    """
    def __init__(self, name, vertices, faces, scale_to_fit=False):
        self.name = name
        self.vertices = np.asarray(vertices, dtype=np.float32)
        self.faces = np.asarray(faces, dtype=np.int32)

    def to_binary_stl(self):
        """Serialize vertices+faces to a binary STL byte string."""
        tris = self.faces
        if tris.ndim == 2 and tris.shape[1] == 4:
            tris = tris[:, :3]
        n = len(tris)
        buf = bytearray(84 + 50 * n)
        struct.pack_into('<I', buf, 80, n)
        v = self.vertices.astype(np.float64)
        for i, (a, b, c) in enumerate(tris):
            p0, p1, p2 = v[a], v[b], v[c]
            normal = np.cross(p1 - p0, p2 - p0)
            ln = np.linalg.norm(normal)
            if ln > 0:
                normal /= ln
            off = 84 + i * 50
            struct.pack_into('<12fH', buf, off,
                             normal[0], normal[1], normal[2],
                             p0[0], p0[1], p0[2],
                             p1[0], p1[1], p1[2],
                             p2[0], p2[1], p2[2], 0)
        return bytes(buf)


class ThreeJSViewport(QWidget):
    mesh_selected = pyqtSignal(int)
    # Emitted once the STL meshes + URDF are fully loaded in the JS viewer.
    load_finished = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 200)
        self.sim = None
        self.connection_manager = None
        self._load_reported = False

        self._meshes = []          # _MeshStub list (for get_meshes())
        self._mesh_map = {}        # {filename_lower.stl: (verts, faces)}
        self._urdf_text = None     # URDF XML currently loaded
        self._joints = [0.0] * 6
        self._auto_rotate = False
        self._wireframe = False

        # Gizmo compatibility — the Three.js viewer has its own selection
        # highlight; keep a tiny stub so main_window's gizmo calls no-op.
        self.gizmo = _GizmoStub()
        self._selected_link = -1
        # The program overlay is attached by main_window (a child widget);
        # declare it here so the attribute always exists.
        self._program_overlay = None

        self._build_ui()

        self._ready = False
        self._pending = []
        self._js_ready_timer = QTimer(self)
        self._js_ready_timer.setInterval(100)
        self._js_ready_timer.timeout.connect(self._flush_pending)
        self._js_ready_timer.start()

    # ── UI ──────────────────────────────────────────────────

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        if not _WEBENGINE:
            try:
                with open(os.path.join(os.path.dirname(__file__), "_webengine_diag.txt"), "w") as f:
                    f.write("err=%s\nfile=%s\n" % (_WEBENGINE_ERR, __file__))
            except Exception:
                pass
            layout.addWidget(QLabel(
                "The Three.js STL viewer needs PyQtWebEngine.\n\n"
                "Install it and RESTART the app:\n"
                "  pip install PyQtWebEngine\n\n"
                "(If you just installed it, close and reopen the Studio\n"
                "— the running instance was started before the install.)",
                self))
            return

        html_dir = os.path.join(os.path.dirname(__file__), "stl_embed")
        # Serve the page + meshes over loopback HTTP — Chromium's file://
        # security model blocks XHR/fetch of local files (STLLoader needs it).
        self._server = None
        self._server_thread = None
        try:
            import http.server
            import socketserver
            import threading

            class _NoLog(http.server.SimpleHTTPRequestHandler):
                def log_message(self, *a):
                    pass

            handler = functools.partial(_NoLog, directory=html_dir)
            self._server = socketserver.ThreadingTCPServer(("127.0.0.1", 0), handler)
            self._server_port = self._server.server_address[1]
            self._server_thread = threading.Thread(
                target=self._server.serve_forever, daemon=True)
            self._server_thread.start()
            self._web = QWebEngineView(self)
            self._web.setUrl(QUrl("http://127.0.0.1:%d/index.html" % self._server_port))
            layout.addWidget(self._web)
        except Exception as e:
            layout.addWidget(QLabel(
                "Couldn't start the STL viewer server: %r" % (e,), self))

    def _js(self, script):
        """Run JS in the embedded page; queue until the page is ready."""
        if not _WEBENGINE or not hasattr(self, "_web"):
            return
        if self._ready:
            try:
                self._web.page().runJavaScript(script)
            except Exception:
                self._pending.append(script)
        else:
            self._pending.append(script)

    def _flush_pending(self):
        if not _WEBENGINE or not hasattr(self, "_web"):
            return
        try:
            self._web.page().runJavaScript(
                "typeof window.astra !== 'undefined'", self._on_js_ready)
            # Poll for the robot being fully loaded so the startup splash can
            # stay up until the STL meshes are actually in the scene.
            if not self._load_reported:
                self._web.page().runJavaScript(
                    "window.astra && window.astra.getState ? "
                    "window.astra.getState() : ''", self._on_load_state)
        except Exception:
            pass

    def _on_load_state(self, state):
        if state == "loaded" and not self._load_reported:
            self._load_reported = True
            self.load_finished.emit()

    def _on_js_ready(self, ok):
        if ok:
            self._ready = True
            self._js_ready_timer.stop()
            for s in self._pending:
                self._web.page().runJavaScript(s)
            self._pending.clear()

    # ── Mesh API (matches old Viewport3D) ───────────────────

    def add_mesh(self, mesh):
        """Register an imported STL part so the URDF can use it.

        Stores vertices+faces keyed by filename (lowercased, minus .stl),
        which load_urdf()'s mesh_map feeds into the URDF's link meshes.
        """
        if hasattr(mesh, "vertices") and hasattr(mesh, "faces"):
            stub = _MeshStub(getattr(mesh, "name", "mesh"),
                             mesh.vertices, mesh.faces)
        else:
            stub = mesh
        self._meshes.append(stub)
        # Key by the mesh's name (Base/Link1..Link6) or a default.
        key = getattr(stub, "name", "mesh").lower()
        if not key.endswith(".stl"):
            key += ".stl"
        self._mesh_map[key] = (np.asarray(stub.vertices, dtype=np.float32),
                               np.asarray(stub.faces, dtype=np.int32))
        self._js("window.astra.loadURDF(%s, %s);" % (
            _json.dumps(self._urdf_text or ""), _json.dumps(self._mesh_map_js())))

    def clear_meshes(self):
        self._meshes.clear()
        self._mesh_map.clear()

    def get_meshes(self):
        return self._meshes

    # ── View controls ───────────────────────────────────────

    def set_wireframe(self, on):
        self._wireframe = bool(on)
        self._js("window.astra.setWireframe(%s);" % ("true" if on else "false"))

    def set_grid_visible(self, on):
        self._js("window.astra.setGridVisible && window.astra.setGridVisible(%s);"
                 % ("true" if on else "false"))

    def set_auto_rotate(self, on):
        self._auto_rotate = bool(on)
        self._js("window.astra.setAutoRotate && window.astra.setAutoRotate(%s);"
                 % ("true" if on else "false"))

    def fit_view(self):
        self._js("window.astra.fitCamera();")

    def reset_view(self):
        self._js("window.astra.resetCamera();")

    # ── URDF robot API ────────────────────────────────────────

    def _build_default_meshes(self):
        """Load the REAL decimated robot meshes from disk so the URDF robot
        is visible immediately, without any user import.

        Meshes are the .m.stl files (raw mm geometry / 1000 -> meters, real
        world positions) that the generated nite369.urdf references by name.
        They are copied next to index.html (stl_embed/) so the JS STLLoader
        can fetch them via file:// — keeping the exact full-res geometry.
        """
        if self._mesh_map:
            return
        meshes_dir = os.path.join(os.path.dirname(__file__), "stl_embed")
        # Ensure the real mesh files are beside the page.
        try:
            import shutil
            # __file__ = .../astra_studio/astra_studio/gui/threejs_viewport.py
            # project root = up 3 from __file__.
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))))
            src_dir = os.path.join(project_root, "tools", "urdf-viz")
            for f in ["base1.m.stl", "a2.m.stl", "a3.m.stl", "a4.m.stl",
                      "a5.m.stl", "a6.m.stl", "a7.m.stl"]:
                src = os.path.join(src_dir, f)
                dst = os.path.join(meshes_dir, f)
                if os.path.exists(src) and not os.path.exists(dst):
                    shutil.copyfile(src, dst)
        except Exception as e:
            print("[threejs_viewport] mesh copy failed:", e)

    def load_urdf(self, urdf_text, mesh_map=None):
        """Load a URDF into the embedded Three.js viewer.

        The URDF references the real robot meshes (base1.m.stl..a7.m.stl)
        which are copied next to the page and fetched by STLLoader. The
        optional mesh_map (small/imported parts) is passed to JS for the
        fallback path.
        """
        if not urdf_text:
            return
        self._urdf_text = urdf_text
        if mesh_map:
            for k, (v, f) in mesh_map.items():
                self._mesh_map[k] = (np.asarray(v, dtype=np.float32),
                                     np.asarray(f, dtype=np.int32))
        self._build_default_meshes()
        self._js("window.astra.loadURDF(%s, %s);" % (
            _json.dumps(urdf_text), _json.dumps(self._mesh_map_js())))

    def _mesh_map_js(self):
        """Serialize the mesh map for the JS bridge."""
        out = {}
        for name, (verts, faces) in self._mesh_map.items():
            out[name] = {
                "vertices": [[float(x) for x in row] for row in verts],
                "faces": [[int(x) for x in row] for row in faces],
            }
        return out

    # ── End-effector drag readback ───────────────────────────────

    drag_finished = pyqtSignal(list)   # new joint angles (deg) after drag
    _drag_timer = None
    _prev_drag_joints = None

    def _start_drag_poll(self):
        """Start polling for drag completion (once, on first call)."""
        if ThreeJSViewport._drag_timer is not None:
            return
        ThreeJSViewport._drag_timer = QTimer(self)
        ThreeJSViewport._drag_timer.setInterval(50)
        ThreeJSViewport._drag_timer.timeout.connect(self._poll_drag)
        ThreeJSViewport._drag_timer.start()

    def _poll_drag(self):
        """Poll JS for drag state; emit drag_finished when a drag ends."""
        if not self._ready or not hasattr(self, '_web'):
            return
        def _cb(result):
            if isinstance(result, dict) and result.get('ended'):
                joints = result.get('joints')
                if joints and joints != ThreeJSViewport._prev_drag_joints:
                    ThreeJSViewport._prev_drag_joints = list(joints)
                    self.drag_finished.emit(list(joints))
        self._web.page().runJavaScript(
            "JSON.stringify({ended: !!window.astra._dragEnded, "
            "joints: window.astra.lastJoints});",
            _cb)
        # Clear the flag after reading.
        self._js("window.astra._dragEnded = false;")

    def set_joints(self, deg):
        """Drive the URDF robot's joints with 6 angles (degrees)."""
        self._joints = [float(x) for x in deg]
        self._js("window.astra.setJoints(%s);" % (self._joints,))
        self._start_drag_poll()

    def home(self):
        """Snap the embedded robot back to the home pose (all joints 0)."""
        self._joints = [0.0] * 6
        self._js("window.astra.home && window.astra.home();")

    def jog_world(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0):
        """Translate the tool along world X/Y/Z (mm) via the JS PoE IK."""
        self._js("window.astra.jogWorld && window.astra.jogWorld(%s, %s, %s);"
                 % (float(dx_mm), float(dy_mm), float(dz_mm)))

    def articulate(self):
        """No-op — the URDF hierarchy is loaded by load_urdf()."""
        pass

    def load_urdf_file(self, urdf_path, mesh_map=None):
        """Convenience: load a URDF file from disk."""
        try:
            with open(urdf_path, "r", encoding="utf-8") as f:
                self.load_urdf(f.read(), mesh_map)
        except Exception as e:
            print("[threejs_viewport] URDF load failed:", e)

    # ── Robot / simulation compatibility stubs ──────────────

    def set_simulation(self, sim_engine):
        self.sim = sim_engine

    def set_connection_manager(self, cm):
        self.connection_manager = cm

    def start(self):
        pass

    def stop(self):
        pass

    def set_theme(self, theme_name):
        pass

    def select_end_effector(self):
        pass

    # ── Misc compatibility ──────────────────────────────────

    def rect(self):
        return super().rect()

    def mapTo(self, parent, p):
        return super().mapTo(parent, p)

    def resizeEvent(self, event):
        super().resizeEvent(event)

    def closeEvent(self, event):
        # Stop the loopback HTTP server so the port is freed.
        try:
            if self._server is not None:
                self._server.shutdown()
                self._server.server_close()
        except Exception:
            pass
        super().closeEvent(event)


class _GizmoStub(QObject):
    """Minimal gizmo stand-in so main_window's gizmo calls no-op."""

    def __init__(self):
        super().__init__()
        self._visible = False

    def show(self):
        self._visible = True

    def hide(self):
        self._visible = False

    def setVisible(self, v):
        self._visible = bool(v)

    def isVisible(self):
        return self._visible
