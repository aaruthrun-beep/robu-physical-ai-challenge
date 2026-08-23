"""
Unified command manager. The SAME collision checker guards both the simulator
and real hardware; nothing is ever sent without passing validation.
"""
import time, math
from . import kinematics as kin
from . import collision as col
from . import planner
from .workspace import classify, SAFE


class Rejection(Exception):
    pass


class CommandManager:
    def __init__(self, cfg, backend, assembly=+1):
        self.cfg = cfg
        self.backend = backend
        self.assembly = assembly
        self.log = []           # rejected commands, for the UI history panel

    # ---- validation -------------------------------------------------------
    def validate_target(self, x, y):
        """Return (angles, assembly, report) for a safe target, or raise Rejection."""
        state, assembly, rep = classify(self.cfg, x, y)
        if state != SAFE:
            reason = "unreachable (no IK solution)" if rep is None else rep.reason
            self._reject(f"target ({x:.0f},{y:.0f})", reason)
            raise Rejection(reason)
        sol = kin.ik(self.cfg, x, y, assembly)
        return sol, assembly, rep

    def validate_jog(self, motor, delta):
        """Check a single-motor jog; raise Rejection if the step collides."""
        cur = self.backend.read_angles()
        t = list(cur)
        t[motor] += delta
        rep = col.check_angles(self.cfg, t[0], t[1], self.assembly)
        if not rep.ok:
            self._reject(f"jog motor{motor} {math.degrees(delta):+.1f} deg", rep.reason)
            raise Rejection(rep.reason)
        return tuple(t)

    # ---- execution --------------------------------------------------------
    def move_to(self, x, y):
        """Validate + plan a collision-checked path, then command the backend."""
        sol, assembly, _ = self.validate_target(x, y)
        self.assembly = assembly
        traj, fail = planner.plan(self.cfg, self.backend.read_angles(), sol, assembly)
        if traj is None:
            f, rep = fail
            self._reject(f"path to ({x:.0f},{y:.0f})",
                         f"collision at {f*100:.0f}% of the move: {rep.reason}")
            raise Rejection(f"path collides at {f*100:.0f}%: {rep.reason}")
        if hasattr(self.backend, "send_trajectory"):
            self.backend.send_trajectory(traj)
        else:
            self.backend.send_angles(*sol)
        return sol

    def jog(self, motor, delta):
        t = self.validate_jog(motor, delta)
        self.backend.send_angles(*t)
        return t

    def _reject(self, what, why):
        self.log.append((time.strftime("%H:%M:%S"), what, why))
        self.log = self.log[-100:]
