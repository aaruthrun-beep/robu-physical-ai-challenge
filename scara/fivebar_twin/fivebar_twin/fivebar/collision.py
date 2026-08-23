"""
3D collision checker for the 5-bar robot.

Every physical part is modelled as a capsule: a line segment (in 3D, using the
real link-plane heights) plus a radius. Two parts collide when the distance
between their segments is less than (r1 + r2 + margin). Links that share a
passive joint (e.g. the two distal links at the end-effector) legitimately touch
there, so we trim a short length off the shared end before measuring, and detect
only genuine body overlap.

z-layout (mm):  left proximal plane z=0 ; right proximal plane z=dh ;
                end-effector pin at z=dh/2 (bridges the two distal links).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from . import kinematics as kin


@dataclass
class Capsule:
    a: Tuple[float, float, float]
    b: Tuple[float, float, float]
    r: float
    name: str
    ja: Optional[str] = None   # joint id at endpoint a
    jb: Optional[str] = None   # joint id at endpoint b


@dataclass
class CollisionReport:
    ok: bool
    min_clearance: float
    worst_pair: Optional[Tuple[str, str]] = None
    reason: str = ""


# ---------- vector helpers -------------------------------------------------
def _sub(p, q): return (p[0]-q[0], p[1]-q[1], p[2]-q[2])
def _add(p, q): return (p[0]+q[0], p[1]+q[1], p[2]+q[2])
def _mul(p, s): return (p[0]*s, p[1]*s, p[2]*s)
def _dot(p, q): return p[0]*q[0] + p[1]*q[1] + p[2]*q[2]
def _norm(p):  return math.sqrt(_dot(p, p))


def _seg_seg_distance(p1, q1, p2, q2) -> float:
    """Shortest distance between two 3D segments (robust clamped solution)."""
    d1 = _sub(q1, p1)
    d2 = _sub(q2, p2)
    r = _sub(p1, p2)
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    EPS = 1e-9
    if a <= EPS and e <= EPS:
        return _norm(r)
    if a <= EPS:
        s = 0.0
        t = max(0.0, min(1.0, f / e))
    else:
        c = _dot(d1, r)
        if e <= EPS:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / denom)) if denom > EPS else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))
    c1 = _add(p1, _mul(d1, s))
    c2 = _add(p2, _mul(d2, t))
    return _norm(_sub(c1, c2))


def _trim(a, b, trim_len):
    """Move endpoint a toward b by trim_len (to ignore a shared-joint contact)."""
    v = _sub(b, a)
    L = _norm(v)
    if L < 1e-6 or trim_len <= 0:
        return a
    t = min(trim_len / L, 0.49)
    return _add(a, _mul(v, t))


def build_capsules(cfg, t1, t2, ee) -> List[Capsule]:
    """All moving + fixed capsules for a given pose."""
    (A1x, A1y), (A2x, A2y) = cfg.bases()
    C1, C2 = kin.elbows(cfg, t1, t2)
    z0, zh, ze = 0.0, cfg.dh, cfg.dh / 2.0
    A1 = (A1x, A1y, z0);  A2 = (A2x, A2y, zh)
    C1 = (C1[0], C1[1], z0);  C2 = (C2[0], C2[1], zh)
    E  = (ee[0], ee[1], ze)
    lr = cfg.link_radius
    caps = [
        Capsule(A1, C1, lr, "prox_L", "A1", "C1"),
        Capsule(A2, C2, lr, "prox_R", "A2", "C2"),
        Capsule(C1, E,  lr, "dist_L", "C1", "EE"),
        Capsule(C2, E,  lr, "dist_R", "C2", "EE"),
        # motor hubs as vertical capsules
        Capsule((A1x, A1y, -cfg.hub_height), (A1x, A1y, z0 + lr), cfg.hub_radius, "hub_L", "A1", "A1"),
        Capsule((A2x, A2y, zh - cfg.hub_height), (A2x, A2y, zh + lr), cfg.hub_radius, "hub_R", "A2", "A2"),
        # end-effector joint hardware
        Capsule(E, (E[0], E[1], ze + 1.0), cfg.ee_radius, "ee_joint", "EE", "EE"),
    ]
    for ob in cfg.obstacles:
        caps.append(Capsule((ob.x, ob.y, ob.z_lo), (ob.x, ob.y, ob.z_hi),
                            ob.radius, ob.name, None, None))
    return caps


# pairs we do NOT check (rigidly connected / meaningless)
_SKIP = {("hub_L", "prox_L"), ("hub_R", "prox_R"), ("ee_joint", "dist_L"), ("ee_joint", "dist_R")}


def check_pose(cfg, t1, t2, ee) -> CollisionReport:
    """Check one fully-specified pose. Returns the tightest clearance found."""
    caps = build_capsules(cfg, t1, t2, ee)
    worst = math.inf
    worst_pair = None
    n = len(caps)
    for i in range(n):
        for j in range(i + 1, n):
            ci, cj = caps[i], caps[j]
            key = (ci.name, cj.name)
            if key in _SKIP or (cj.name, ci.name) in _SKIP:
                continue
            ai, bi, aj, bj = ci.a, ci.b, cj.a, cj.b
            # trim shared passive joints so legitimate contact isn't a collision
            shared = {ci.ja, ci.jb} & {cj.ja, cj.jb}
            shared.discard(None)
            if shared:
                trim = cfg.ee_radius + cfg.link_radius + cfg.margin + 2.0
                if ci.ja in shared: ai = _trim(ai, bi, trim)
                if ci.jb in shared: bi = _trim(bi, ai, trim)
                if cj.ja in shared: aj = _trim(aj, bj, trim)
                if cj.jb in shared: bj = _trim(bj, aj, trim)
            dist = _seg_seg_distance(ai, bi, aj, bj)
            clearance = dist - (ci.r + cj.r)
            if clearance < worst:
                worst = clearance
                worst_pair = (ci.name, cj.name)
    ok = worst >= cfg.margin
    reason = "" if ok else (
        f"{worst_pair[0]} vs {worst_pair[1]} clearance {worst:.1f} mm "
        f"< margin {cfg.margin:.0f} mm")
    return CollisionReport(ok, worst, worst_pair, reason)


def check_angles(cfg, t1, t2, assembly) -> CollisionReport:
    """Check a pose given by motor angles (EE derived by forward kinematics)."""
    ee = kin.fk(cfg, t1, t2, assembly)
    if ee is None:
        return CollisionReport(False, -math.inf, None, "loop cannot close (distal links can't meet)")
    return check_pose(cfg, t1, t2, ee)
