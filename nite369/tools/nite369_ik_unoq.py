"""nite369_ik_unoq.py
====================
Standalone IK controller for the Nite369 6-DOF robot arm, built to run on
an "Uno Q" mini-PC (28 GB storage / 4 GB RAM) as a self-contained unit —
no Astra Studio, no Qt, just Python + numpy + socket.

What it does:
  - Forward kinematics (FK)   : joint angles (deg) -> tool pose (mm, deg)
  - Inverse kinematics (IK)   : tool pose (mm + deg) -> joint angles (deg)
  - Sends joint moves to the Nite369 MASTER Pico over Ethernet (TCP :23)
    using the #MV / #M commands the master firmware understands.

The DH table is the active nominal table from the Nite369 v1.0 RoboDK
model (standard DH, meters), matching Astra Studio's create_astra_dh():

  J1: a=0,     alpha=0,    d=0.290, theta_off=0
  J2: a=0.080, alpha=-90°, d=0,     theta_off=-90°
  J3: a=0.350, alpha=0,    d=0,     theta_off=0
  J4: a=0.045, alpha=-90°, d=0.335, theta_off=0
  J5: a=-0.004,alpha=+90°, d=0,     theta_off=0
  J6: a=0,     alpha=-90°, d=0.044, theta_off=180°

Home (all zeros) = tool at [459, 0, 685] mm.

Usage:
    python nite369_ik_unoq.py                          # interactive CLI
    python nite369_ik_unoq.py fk 10 20 -30 40 50 60    # FK
    python nite369_ik_unoq.py ik 300 0 600 0 0 0       # IK to x y z rx ry rz
    python nite369_ik_unoq.py move 10 20 -30 40 50 60  # send #M move to robot
    python nite369_ik_unoq.py jog 1 200                # continuous jog joint 1 fwd
    python nite369_ik_unoq.py joy                      # joystick jog (hold-to-run)

Joystick mapping (same as Astra Studio):
    left stick X   -> J1      left stick Y  -> J2
    right pad 0-3  -> J3 (up/down), J4 (left/right)
    buttons 4,5,6  -> J5 / J6 wrist combos
Press Ctrl+C to quit joy mode.
"""

import math
import socket
import sys

import numpy as np

# ---------------------------------------------------------------------------
# Nite369 DH parameters (standard DH)
# ---------------------------------------------------------------------------

# (a, alpha_deg, d, theta_off_deg, gear_ratio)
DH = [
    (0.000,    0.0,  0.290,    0.0, 22.8),
    (0.080,  -90.0,  0.000,  -90.0, 45.96),
    (0.350,    0.0,  0.000,    0.0, 40.78125),
    (0.045,  -90.0,  0.335,    0.0, 24.75),
    (-0.004,  90.0,  0.000,    0.0, 25.0),
    (0.000,  -90.0,  0.044,  180.0, 25.0),
]

# Joint limits (deg) — from the studio config
JOINT_LIMITS = [
    (-180.0, 180.0),
    (-200.0, 200.0),
    (-200.0, 200.0),
    (-200.0, 200.0),
    (-200.0, 200.0),
    (-200.0, 200.0),
]

MASTER_HOST = "192.168.1.50"
MASTER_PORT = 23


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def dh_transform(a, alpha_rad, d, theta_rad):
    """Standard DH transform."""
    ct, st = math.cos(theta_rad), math.sin(theta_rad)
    ca, sa = math.cos(alpha_rad), math.sin(alpha_rad)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,       d],
        [0,        0,        0,       1],
    ])


def fk(joints_deg):
    """Forward kinematics: 6 joint angles (deg) -> 4x4 tool pose.

    Uses the SAME convention as the studio's DHArm.forward() so the result
    matches the studio's tool pose at home [0.459, 0, 0.685] m.
    """
    T = np.eye(4)
    for i in range(6):
        a, alpha_deg, d, th_off_deg, _ = DH[i]
        theta_rad = math.radians(joints_deg[i] + th_off_deg)
        T = T @ dh_transform(a, math.radians(alpha_deg), d, theta_rad)
    return T


def fk_xyzuvw(joints_deg):
    """FK -> [x, y, z, rx, ry, rz] (mm, deg) using rotation-vector."""
    T = fk(joints_deg)
    pos = T[:3, 3] * 1000.0  # m -> mm
    R = T[:3, :3]
    # rotation vector from R
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = math.acos(cos_a)
    if ang < 1e-9:
        rv = [0.0, 0.0, 0.0]
    else:
        rv = [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
        rv = [v * ang / (2.0 * math.sin(ang)) for v in rv]
        rv = [math.degrees(v) for v in rv]
    return [pos[0], pos[1], pos[2], rv[0], rv[1], rv[2]]


# ---------------------------------------------------------------------------
# Inverse kinematics (numerical, scipy-free fallback = Jacobian pseudoinverse)
# ---------------------------------------------------------------------------

def ik(target_pos_mm, target_orient=None, seed=None, max_iter=200, tol=1e-6):
    """Inverse kinematics: target [x,y,z] mm (+ optional 3x3 rotation).

    Returns 6 joint angles in DEGREES, or None on failure. Uses scipy's
    Levenberg-Marquardt (the same proven solver the studio uses) with
    random restarts. Falls back to a damped-Jacobian solve if scipy is
    unavailable.
    """
    target = np.array(target_pos_mm, dtype=float) / 1000.0  # mm -> m
    use_orient = target_orient is not None

    if seed is None:
        seed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    seed_rad = np.array(seed, dtype=float) * math.pi / 180.0

    def pose(q):
        T = np.eye(4)
        for i in range(6):
            a, alpha_deg, d, th_off_deg, _ = DH[i]
            T = T @ dh_transform(a, math.radians(alpha_deg), d, q[i] + math.radians(th_off_deg))
        return T

    def residual(q):
        T = pose(q)
        perr = target - T[:3, 3]
        if not use_orient:
            return perr
        R_cur = T[:3, :3]
        R_err = target_orient @ R_cur.T
        oerr = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ])
        return np.concatenate([perr, oerr * 0.3])

    bounds = ([-math.pi * 1.5] * 6, [math.pi * 1.5] * 6)

    # ---- scipy LM with restarts ----
    try:
        from scipy.optimize import least_squares
    except ImportError:
        least_squares = None

    if least_squares is not None:
        guesses = [seed_rad, np.zeros(6)]
        rng = np.random.default_rng(0)
        for _ in range(8):
            guesses.append(rng.uniform(-2.5, 2.5, size=6))
        best = None
        best_cost = float("inf")
        for g in guesses:
            try:
                res = least_squares(residual, g, bounds=bounds,
                                    xtol=1e-8, ftol=1e-8, gtol=1e-8,
                                    max_nfev=max_iter)
            except Exception:
                continue
            if res.cost < best_cost:
                best_cost = res.cost
                best = res.x
        if best is None or best_cost > 1e-6:
            return None
        deg = np.degrees(best)
        for i in range(6):
            lo, hi = JOINT_LIMITS[i]
            if deg[i] < lo or deg[i] > hi:
                return None
        return deg.tolist()

    # ---- scipy-free fallback: damped Jacobian with restarts ----
    rng = np.random.default_rng(0)
    guesses = [seed_rad, np.zeros(6)]
    for _ in range(6):
        guesses.append(rng.uniform(-2.0, 2.0, size=6))

    def jac_num(q):
        d = 1e-6
        n = 6 if use_orient else 3
        J = np.zeros((n, 6))
        for i in range(6):
            qp = q.copy(); qp[i] += d
            qm = q.copy(); qm[i] -= d
            J[:, i] = (residual(qp) - residual(qm)) / (2 * d)
        return J

    for g in guesses:
        th = g.copy()
        converged = False
        for it in range(max_iter):
            err = residual(th)
            T = pose(th)
            pos_err = np.linalg.norm(target - T[:3, 3])
            if pos_err < 0.005:
                converged = True
                break
            J = jac_num(th)
            lam = 0.5 + 2.0 * pos_err
            n_ = J.shape[0]
            JJT = J @ J.T + lam * np.eye(n_)
            try:
                dq = J.T @ np.linalg.solve(JJT, err)
            except np.linalg.LinAlgError:
                break
            sn = np.linalg.norm(dq)
            if sn > 0.05:
                dq = dq * (0.05 / sn)
            best_t = th
            best_n = pos_err
            alpha = 1.0
            for _ in range(8):
                cand = np.clip(th + alpha * dq, -math.pi * 1.5, math.pi * 1.5)
                n2 = np.linalg.norm(target - pose(cand)[:3, 3])
                if n2 < best_n:
                    best_n = n2
                    best_t = cand
                alpha *= 0.5
            if best_n >= pos_err - 1e-12 and it > 5:
                break
            th = best_t
        if converged:
            deg = np.degrees(th)
            for i in range(6):
                lo, hi = JOINT_LIMITS[i]
                if deg[i] < lo or deg[i] > hi:
                    converged = False
                    break
            if converged:
                return deg.tolist()
    return None


# ---------------------------------------------------------------------------
# Ethernet transport to the Nite369 master
# ---------------------------------------------------------------------------

def send_command(cmd, timeout=3.0):
    """Send one #-command to the master over TCP, return the reply."""
    cmd = cmd if cmd.startswith("#") else "#" + cmd
    try:
        s = socket.create_connection((MASTER_HOST, MASTER_PORT), timeout=timeout)
        s.settimeout(timeout)
        s.sendall((cmd + "\n").encode())
        buf = b""
        end = s.gettimeout()
        while True:
            try:
                chunk = s.recv(1024)
                if not chunk:
                    break
                buf += chunk
                if b">" in buf:
                    break
            except socket.timeout:
                break
        s.close()
        return buf.decode("ascii", errors="replace").strip()
    except OSError as e:
        return f"ERR: {e}"


def move_joints(joints_deg, speed=500):
    """Send a 6-joint relative move (#M) to the master."""
    parts = ",".join(f"{float(v):.4f}" for v in joints_deg[:6])
    return send_command(f"M{parts}")


def jog_joint(joint_no, direction, speed=2000):
    """Continuous jog (#JC). joint_no 1-6, direction +1/-1."""
    return send_command(f"JC{joint_no},{1 if direction > 0 else -1},{speed}")


def halt():
    return send_command("H")


# ---------------------------------------------------------------------------
# Joystick jog (hold-to-run) — same mapping as Astra Studio
# ---------------------------------------------------------------------------

def joy_mode(speed=2000):
    """Poll the USB joystick and jog the robot continuously.

    left stick X -> J1, left stick Y -> J2
    right pad buttons 0=up,1=right,2=down,3=left -> J3 (up/down), J4 (left/right)
    buttons 4,5,6 -> J5/J6 wrist combos (4 -> J6 left, 6 -> J6 right,
                                         4+5 -> J5 up, 5+6 -> J5 down)
    """
    try:
        import pygame
    except ImportError:
        print("Need pygame: pip install pygame")
        return

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found")
        return
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Joystick: {joy.get_name()}  (Ctrl+C to quit)")

    active = set()  # set of (joint_no_1based, direction)
    debounce = [None, 0]

    def current_jogs():
        """Return a SET of (joint, dir) for ALL held directions — supports
        multi-axis jogging (e.g. left stick X+Y at once, or pad + wrist)."""
        axes = [joy.get_axis(i) for i in range(min(joy.get_numaxes(), 4))]
        buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
        out = set()
        # left stick X -> J1, Y -> J2  (both can be active together)
        if abs(axes[0]) > 0.5:
            out.add((1, 1 if axes[0] > 0 else -1))
        if abs(axes[1]) > 0.5:
            out.add((2, 1 if axes[1] > 0 else -1))
        # right pad: 0=up 1=right 2=down 3=left -> J3 (up/down), J4 (left/right)
        left = buttons[3] and not buttons[1]
        right = buttons[1] and not buttons[3]
        up = buttons[0] and not buttons[2]
        down = buttons[2] and not buttons[0]
        if left:
            out.add((4, -1))
        if right:
            out.add((4, 1))
        if up:
            out.add((3, 1))
        if down:
            out.add((3, -1))
        # wrist combos: 4+5 -> J5 up, 5+6 -> J5 down, 4 -> J6 left, 6 -> J6 right
        b4, b5, b6 = buttons[4], buttons[5], buttons[6]
        if b4 and b5:
            out.add((5, 1))
        elif b5 and b6:
            out.add((5, -1))
        else:
            if b4:
                out.add((6, -1))
            if b6:
                out.add((6, 1))
        return out

    try:
        while True:
            pygame.event.pump()
            jogs = current_jogs()
            # debounce 3 polls (~60ms)
            if jogs == debounce[0]:
                debounce[1] += 1
            else:
                debounce[0] = jogs
                debounce[1] = 1
            if debounce[1] < 3:
                continue
            if jogs != active:
                # halt all, then start the new set (multi-axis)
                if active:
                    halt()
                active = jogs
                for (j, d) in sorted(active):
                    print(f">> JC{j},{d} @ {speed}", flush=True)
                    jog_joint(j, d, speed)
            import time
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        halt()
        pygame.quit()
        print("\nStopped.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return

    cmd = args[0].lower()
    if cmd == "fk":
        q = [float(x) for x in args[1:7]]
        print("joints:", [round(v, 2) for v in q])
        print("tool xyzuvw:", [round(v, 3) for v in fk_xyzuvw(q)])
    elif cmd == "ik":
        target = [float(x) for x in args[1:7]]
        sol = ik(target[:3], None, seed=None)
        if sol is None:
            print("IK FAILED — target out of reach")
        else:
            print("IK:", [round(v, 2) for v in sol])
            print("FK check:", [round(v, 2) for v in fk_xyzuvw(sol)])
    elif cmd == "move":
        q = [float(x) for x in args[1:7]]
        print("reply:", move_joints(q))
    elif cmd == "jog":
        j = int(args[1])
        d = int(args[2]) if len(args) > 2 else 1
        print("reply:", jog_joint(j, d))
    elif cmd == "joy":
        speed = int(args[1]) if len(args) > 1 else 2000
        joy_mode(speed)
    elif cmd == "halt":
        print("reply:", halt())
    else:
        print("unknown command:", cmd)
        print(__doc__)


if __name__ == "__main__":
    main()
