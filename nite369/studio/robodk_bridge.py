"""RoboDK bridge for the Tesla (Nite369) arm.

The studio runs under the system Python, which cannot import `robodk`
because RoboDK's embedded site-packages bundles an old `enum` package that
breaks Python 3.10's stdlib enum. Instead, this bridge shells out to
`robodk_cli.py` running under RoboDK's own embedded Python (which has a
working robodk package).

The CLI reads one JSON request on stdin and writes one JSON response on
stdout. Windows pipes hang with the embedded interpreter, so we drive it
via temporary files (request file + output file), which is reliable.
"""

import json
import os
import shutil
import subprocess
import tempfile

import numpy as np

# Common RoboDK install roots (Windows)
_DEFAULT_ROBODK_ROOTS = [
    r"C:\RoboDK",
    r"D:\SOFTWARE\RoboDK",
    r"C:\Program Files\RoboDK",
    os.path.expandvars(r"%LOCALAPPDATA%\RoboDK"),
]


def _find_robodk_root():
    env = os.environ.get("ROBODK_HOME")
    if env and os.path.isdir(env):
        return env
    for root in _DEFAULT_ROBODK_ROOTS:
        if os.path.isdir(root):
            return root
    return None


def _find_embedded_python(root):
    candidates = [
        os.path.join(root, "Python-Embedded", "python.exe"),
        os.path.join(root, "bin", "python.exe"),
        os.path.join(root, "Python", "python.exe"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return shutil.which("python")


def _find_cli_path():
    here = os.path.dirname(os.path.abspath(__file__))
    cli = os.path.join(here, "robodk_cli.py")
    return cli if os.path.isfile(cli) else None


class RoboDKBridge:
    def __init__(self, robot_name="Nite369"):
        self.robot_name = robot_name
        self.connected = False
        self._tried = False
        self._python = None
        self._cli = None
        self._robot = None
        self._port = None

    @property
    def robot(self):
        return self._robot

    @property
    def port(self):
        return self._port

    def _resolve(self):
        root = _find_robodk_root()
        if not root:
            return False
        py = _find_embedded_python(root)
        cli = _find_cli_path()
        if not py or not cli:
            return False
        self._python = py
        self._cli = cli
        return True

    def _call(self, op, **kwargs):
        """Run one CLI op via a temp-file request, return the parsed dict."""
        if not self._resolve():
            self.connected = False
            return None
        req = {"op": op}
        req.update(kwargs)
        req_json = json.dumps(req) + "\n"

        for port in (20500, 20501):
            with tempfile.NamedTemporaryFile("w", delete=False, suffix=".req") as fin:
                fin.write(req_json)
                in_path = fin.name
            out_path = in_path + ".out"
            try:
                with open(in_path, "r") as f_in, open(out_path, "w") as f_out:
                    proc = subprocess.run(
                        [self._python, self._cli, str(port)],
                        stdin=f_in, stdout=f_out,
                        stderr=subprocess.DEVNULL, text=True, timeout=15,
                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                    )
                with open(out_path, "r") as f_out:
                    line = f_out.read().strip()
                if line:
                    resp = json.loads(line)
                    if resp.get("ok"):
                        self.connected = True
                        if port != self._port:
                            self._port = port
                            self._robot = resp.get("robot")
                        return resp
            except Exception:
                pass
            finally:
                try:
                    os.unlink(in_path)
                    os.unlink(out_path)
                except OSError:
                    pass
        self.connected = False
        return None

    def ping(self):
        """Check connectivity — returns True if a robot was found."""
        resp = self._call("ping")
        return resp is not None and resp.get("ok", False)

    # ── public API ───────────────────────────────────────────────

    @property
    def robot(self):
        return self._robot

    @property
    def port(self):
        return self._port

    def fk(self, joints_deg):
        """Forward kinematics: joints (deg) -> 4x4 pose (numpy) or None."""
        resp = self._call("fk", joints=list(float(j) for j in joints_deg))
        if resp is None or "pose" not in resp:
            return None
        return np.array(resp["pose"], dtype=float)

    def ik(self, pose_np, initial_guess=None, retries=8):
        """Inverse kinematics: 4x4 pose -> joint solution (deg) or None."""
        pose = np.asarray(pose_np, dtype=float).tolist()
        guess = list(initial_guess) if initial_guess is not None else [0.0] * 6
        for i in range(retries):
            g = guess if i == 0 else [0.0] * 6
            resp = self._call("ik", pose=pose, initial=g)
            if resp is not None and "joints" in resp:
                joints = [float(j) for j in resp["joints"]]
                if len(joints) >= 6:
                    return joints[:6]
        return None

    def move_joints(self, joints_deg, blocking=True):
        """Move the RoboDK model to a joint position (sim only)."""
        self._call("move", joints=list(float(j) for j in joints_deg))

    def get_joints(self):
        """Current RoboDK joint values (deg) or None."""
        resp = self._call("get_joints")
        if resp is None or "joints" not in resp:
            return None
        return [float(j) for j in resp["joints"]]

    def get_dh(self):
        """Dump the RoboDK robot's DH/link table (list of dicts) or None."""
        resp = self._call("get_dh")
        if resp is None or "dh" not in resp:
            return None
        if resp.get("robot"):
            self._robot = resp["robot"]
        return resp["dh"]

    def close(self):
        """No-op — each call is a short-lived subprocess (no persistent proc)."""
        pass
