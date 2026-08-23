"""UrdfVizBridge — launches urdf-viz as the live 3D URDF viewer and drives
its joints over the HTTP API (http://127.0.0.1:7777).

urdf-viz is a proven Rust URDF viewer (openrr/urdf-viz) with a web I/O
interface: POST /set_joint_positions {"names":[...],"positions":[...]}.
This replaces the QtWebEngine embedded viewer, which cannot composite WebGL
on this machine (black viewport).

The viewer is a companion window; the Studio sends the same joint angles it
would send to the real robot / simulation.
"""

import json
import os
import subprocess
import sys
import time
import urllib.request
import urllib.error

DEFAULT_PORT = 7777
_BASE = "http://127.0.0.1:%d"

# Location of the urdf-viz binary + our Nite369 URDF (bundled with the app).
# The tools/ folder sits at the project root (astra_studio/tools/urdf-viz).
_TOOL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "tools", "urdf-viz")
URDF_VIZ_EXE = os.path.join(_TOOL_DIR, "urdf-viz.exe")
URDF_PATH = os.path.join(_TOOL_DIR, "nite369.urdf")

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


class UrdfVizBridge:
    def __init__(self, port=DEFAULT_PORT):
        self.port = port
        self._proc = None

    def _url(self, path):
        return (_BASE % self.port) + path

    # ── Lifecycle ──────────────────────────────────────────────

    def is_running(self):
        return self._proc is not None and self._proc.poll() is None

    def start(self):
        """Launch urdf-viz with our URDF (if not already running)."""
        if self.is_running():
            return True
        if not os.path.exists(URDF_VIZ_EXE):
            print("[urdf-viz] binary not found:", URDF_VIZ_EXE)
            return False
        if not os.path.exists(URDF_PATH):
            print("[urdf-viz] URDF not found:", URDF_PATH)
            return False
        try:
            self._proc = subprocess.Popen(
                [URDF_VIZ_EXE, URDF_PATH, "-p", str(self.port), "-m"],
                cwd=_TOOL_DIR,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # Wait for the HTTP API to come up.
            for _ in range(30):
                if self.ping():
                    return True
                time.sleep(0.2)
            print("[urdf-viz] started but HTTP API not responding")
            return False
        except Exception as e:
            print("[urdf-viz] launch failed:", e)
            return False

    def stop(self):
        if self.is_running():
            self._proc.terminate()
            self._proc = None

    # ── HTTP API ───────────────────────────────────────────────

    def _post(self, path, payload):
        req = urllib.request.Request(
            self._url(path),
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read().decode())
        except (urllib.error.URLError, OSError):
            return None

    def ping(self):
        try:
            with urllib.request.urlopen(self._url("/get_joint_positions"),
                                        timeout=1) as r:
                r.read()
                return True
        except Exception:
            return False

    def set_joints(self, angles_deg):
        """Drive the URDF robot's joints. angles_deg: 6 values in degrees."""
        if not self.is_running():
            if not self.start():
                return False
        # urdf-viz expects radians.
        positions = [float(a) * 3.141592653589793 / 180.0 for a in angles_deg]
        return self._post("/set_joint_positions",
                          {"names": JOINT_NAMES, "positions": positions})

    def get_joints(self):
        """Return current joint positions (degrees) or None."""
        data = self._post("/get_joint_positions", {})
        if data and "positions" in data:
            return [p * 180.0 / 3.141592653589793 for p in data["positions"]]
        return None
