"""
Closed-loop kinematics for the planar 5-bar parallel robot.

Frame: origin at the midpoint of the two motor shafts, +X right, +Y forward.
Left motor A1 = (-d/2, 0), right motor A2 = (+d/2, 0).
theta = world angle of each proximal link from +X, CCW positive.

The two elbow (passive) joints are DERIVED from theta; the end-effector is the
circle-circle intersection of the two distal links. Two assembly modes:
    assembly = +1  -> elbow-up   (left sign +1, right sign -1)
    assembly = -1  -> elbow-down (left sign -1, right sign +1)
"""
import math
from typing import Optional, Tuple


def signs(assembly: int):
    return (+1, -1) if assembly >= 0 else (-1, +1)


def _leg_angle(x, y, Ax, Ay, a, d, s) -> Optional[float]:
    """Proximal angle so the distal link (length d) exactly reaches (x,y). None if impossible."""
    vx, vy = x - Ax, y - Ay
    D = math.hypot(vx, vy)
    if D > a + d or D < abs(a - d):
        return None
    beta = math.atan2(vy, vx)
    c = (D * D + a * a - d * d) / (2 * D * a)
    c = max(-1.0, min(1.0, c))
    return beta + s * math.acos(c)


def ik(cfg, x, y, assembly: int) -> Optional[Tuple[float, float]]:
    """Target (x,y) -> (theta1, theta2) for the chosen assembly, or None if unreachable."""
    (A1x, A1y), (A2x, A2y) = cfg.bases()
    sL, sR = signs(assembly)
    t1 = _leg_angle(x, y, A1x, A1y, cfg.L1a, cfg.L2a, sL)
    t2 = _leg_angle(x, y, A2x, A2y, cfg.L1b, cfg.L2b, sR)
    if t1 is None or t2 is None:
        return None
    if cfg.theta_min is not None and (t1 < cfg.theta_min or t2 < cfg.theta_min):
        return None
    if cfg.theta_max is not None and (t1 > cfg.theta_max or t2 > cfg.theta_max):
        return None
    return t1, t2


def elbows(cfg, t1, t2):
    """Elbow (passive joint) positions from the two proximal angles."""
    (A1x, A1y), (A2x, A2y) = cfg.bases()
    C1 = (A1x + cfg.L1a * math.cos(t1), A1y + cfg.L1a * math.sin(t1))
    C2 = (A2x + cfg.L1b * math.cos(t2), A2y + cfg.L1b * math.sin(t2))
    return C1, C2


def fk(cfg, t1, t2, assembly: int) -> Optional[Tuple[float, float]]:
    """(theta1, theta2) -> end-effector (x,y) via circle-circle intersection, or None."""
    C1, C2 = elbows(cfg, t1, t2)
    ex, ey = C2[0] - C1[0], C2[1] - C1[1]
    Dc = math.hypot(ex, ey)
    if Dc < 1e-9 or Dc > cfg.L2a + cfg.L2b or Dc < abs(cfg.L2a - cfg.L2b):
        return None
    aa = (cfg.L2a * cfg.L2a - cfg.L2b * cfg.L2b + Dc * Dc) / (2 * Dc)
    hs = cfg.L2a * cfg.L2a - aa * aa
    if hs < 0:
        return None
    h = math.sqrt(hs)
    ux, uy = ex / Dc, ey / Dc
    bx, by = C1[0] + aa * ux, C1[1] + aa * uy
    # Pick the coupler on the side of the C1->C2 line consistent with the
    # assembly (left normal for elbow-up, right normal for elbow-down). This
    # matches whatever the IK produced, independent of global orientation.
    if assembly >= 0:
        return (bx - h * uy, by + h * ux)   # left normal
    return (bx + h * uy, by - h * ux)       # right normal


def jacobian_det(cfg, t1, t2, x, y) -> float:
    """Determinant of the inverse Jacobian; ~0 near a parallel singularity."""
    C1, C2 = elbows(cfg, t1, t2)
    rows = []
    for (C, a, th) in ((C1, cfg.L1a, t1), (C2, cfg.L1b, t2)):
        phi = math.atan2(y - C[1], x - C[0])
        den = (a / 1000.0) * math.sin(phi - th)
        if abs(den) < 1e-9:
            return 0.0
        rows.append((math.cos(phi) / den, math.sin(phi) / den))
    return rows[0][0] * rows[1][1] - rows[1][0] * rows[0][1]
