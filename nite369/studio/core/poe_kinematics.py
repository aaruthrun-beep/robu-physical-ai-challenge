"""Product-of-Exponentials (PoE) kinematics for the Astra 6-DOF robot.

This module mirrors — *line for line* — the validated PoE implementation that
lives in the embedded Three.js viewer (``astra_studio/gui/stl_embed/index.html``,
the "Validated PoE model (RoboDK ground truth)" section) so that the Python
side and the JavaScript side solve exactly the same math.

Screws are expressed in the **base frame** (twist v = -w x q for a revolute
joint at pivot q), and the forward kinematics is

    T(theta) = e^[S1]theta1 * ... * e^[S6]theta6 * M

where M is the home (theta = 0) tool pose.

The pivots/axes/M constants are NOT hand-entered here: ``astra_kinematics.py``
derives them from the canonical DH table (``create_astra_dh``), which also
generates the JavaScript constants file used by the viewer.  Import this module
and use ``make_astra_poe()`` to get a model wired to the canonical DH table.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np


# ---------------------------------------------------------------------------
# Screw / exponential-coordinate primitives (identical math to the JS viewer)
# ---------------------------------------------------------------------------

def skew(w: Sequence[float]) -> np.ndarray:
    """Skew-symmetric 3x3 matrix from a 3-vector."""
    wx, wy, wz = w
    return np.array([
        [0, -wz, wy],
        [wz, 0, -wx],
        [-wy, wx, 0],
    ])


def mat_mul(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """3x3 matrix multiply (kept explicit to mirror the JS exactly)."""
    return A @ B


def screw_exp4(w: Sequence[float], v: Sequence[float], th: float):
    """Exponential of a unit screw (w, v) about the angle th.

    Returns (R, t): the 3x3 rotation and the translation of the 4x4
    exponential, exactly as the JS ``_screwExp4``.
    """
    x, y, z = w
    c, s, C = math.cos(th), math.sin(th), 1 - math.cos(th)
    R = np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])
    wv = np.array([w[1] * v[2] - w[2] * v[1],
                   w[2] * v[0] - w[0] * v[2],
                   w[0] * v[1] - w[1] * v[0]])
    IR = np.eye(3) - R
    dot = w[0] * v[0] + w[1] * v[1] + w[2] * v[2]
    t = IR @ wv + np.array(w) * dot * th
    return R, t


def exp_map(w: Sequence[float], th: float) -> np.ndarray:
    """Exponential map so(3) -> SO(3) (Rodrigues)."""
    x, y, z = w
    c, s, C = math.cos(th), math.sin(th), 1 - math.cos(th)
    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


def twist_from_pivot(axis: Sequence[float], pivot: Sequence[float]) -> np.ndarray:
    """Base-frame twist (w, v) for a revolute joint: v = -w x q."""
    w = np.asarray(axis, dtype=float)
    q = np.asarray(pivot, dtype=float)
    v = -np.cross(w, q)
    return np.concatenate([w, v])


# ---------------------------------------------------------------------------
# PoE model
# ---------------------------------------------------------------------------

@dataclass
class PoEModel:
    """A product-of-exponentials robot model.

    Attributes:
        pivots: (n, 3) array of joint-frame origins at home (m).
        axes:   (n, 3) array of joint screw axes (unit vectors, base frame).
        home:   4x4 tool pose at theta = 0.
        joint_names: optional list of joint names (length n).
    """

    pivots: np.ndarray
    axes: np.ndarray
    home: np.ndarray
    joint_names: List[str] | None = None

    def __post_init__(self):
        self.pivots = np.asarray(self.pivots, dtype=float).reshape(-1, 3)
        self.axes = np.asarray(self.axes, dtype=float).reshape(-1, 3)
        self.home = np.asarray(self.home, dtype=float).reshape(4, 4)
        if self.joint_names is None:
            self.joint_names = ["j%d" % (i + 1) for i in range(self.n_joints)]

    @property
    def n_joints(self) -> int:
        return len(self.pivots)

    def fk(self, thetas: Sequence[float]):
        """Forward kinematics. Returns (pos, rot) like the JS ``fkPoE``."""
        thetas = list(thetas)
        R = np.eye(3)
        p = np.zeros(3)
        for i in range(self.n_joints):
            q = self.pivots[i]
            w = self.axes[i]
            v = -np.cross(w, q)
            Ri, ti = screw_exp4(w, v, thetas[i])
            p = R @ ti + p
            R = mat_mul(R, Ri)
        Rf = R @ self.home[:3, :3]
        tf = R @ self.home[:3, 3] + p
        return tf, Rf

    def fk_4x4(self, thetas: Sequence[float]) -> np.ndarray:
        """Forward kinematics as a single 4x4 matrix."""
        pos, rot = self.fk(thetas)
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        return T

    def jacobian_pos(self, thetas: Sequence[float]) -> np.ndarray:
        """Analytic 3x6 position Jacobian (chain formulation).

        Column i = z_{i-1} x (p_ee - p_{i-1}), where (z_{i-1}, p_{i-1}) is the
        axis and origin of the DH frame *before* joint i at the current
        configuration.  This is the standard geometric Jacobian for a serial
        chain and matches finite-differencing the FK to ~1e-9 (the naive
        base-frame-screw transport ``w x (p_ee - q)`` with fixed pivots is
        wrong for the wrist joints, whose axes pass near the tool point).
        """
        thetas = list(thetas)
        J = np.zeros((3, self.n_joints))
        p_ee = self.fk(thetas)[0]
        # Cumulative frame before joint i: T_{0..i-1} applied to home pivots.
        R = np.eye(3)
        p = np.zeros(3)
        for i in range(self.n_joints):
            # The axis/pivot of joint i at the CURRENT config = the home
            # axis/pivot transported by the rotation/translation of the
            # previous joints only.
            w_home = self.axes[i]
            q_home = self.pivots[i]
            z = R @ w_home
            q = R @ q_home + p
            J[:, i] = np.cross(z, p_ee - q)
            # Advance the frame with joint i's screw exponential.
            w = self.axes[i]
            qh = self.pivots[i]
            v = -np.cross(w, qh)
            Ri, ti = screw_exp4(w, v, thetas[i])
            p = R @ ti + p
            R = R @ Ri
        return J

    def solve6(self, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Solve a 6x6 linear system by Gaussian elimination (mirrors JS)."""
        A = np.asarray(A, dtype=float).copy()
        b = np.asarray(b, dtype=float).copy()
        n = 6
        M = np.hstack([A, b.reshape(-1, 1)])
        for col in range(n):
            piv = col
            for r in range(col + 1, n):
                if abs(M[r, col]) > abs(M[piv, col]):
                    piv = r
            M[[col, piv]] = M[[piv, col]]
            d = M[col, col]
            if abs(d) < 1e-12:
                continue
            M[col, col:] /= d
            for r in range(n):
                if r == col:
                    continue
                f = M[r, col]
                M[r, col:] -= f * M[col, col:]
        return M[:, n]

    def ik(self, target_pos, seed, max_iter=200, tol=1e-8,
           joint_clamp=3.5, damping=0.05, max_step=0.05):
        """Damped least-squares position IK (chain-Jacobian, adaptive damping).

        Args:
            target_pos: desired [x, y, z] tool position (m).
            seed: initial joint angles (radians).
            max_iter: maximum iterations.
            tol: convergence tolerance on position error (m).
            joint_clamp: joint angle clamp magnitude (rad).
            damping: DLS damping parameter, as a *fraction of the Jacobian
                scale* (adaptive).  Too large leaves residual error; too small
                oversteps on stiff problems.
            max_step: maximum joint step per iteration (rad), to avoid
                overstep divergence.

        Returns:
            Joint angles (radians) as a list.
        """
        th = list(seed)
        for _ in range(max_iter):
            cur = self.fk(th)[0]
            err = np.asarray(target_pos, dtype=float) - cur
            n = float(np.linalg.norm(err))
            if n < tol:
                break
            J = self.jacobian_pos(th)
            # Dual-form DLS with adaptive damping: dtheta = J^T (J J^T +
            # (damp*scale)^2 I)^-1 err.  J is 3x6 (wide), so J.T@J is singular;
            # J J^T is 3x3 and well-conditioned.  Damping scales with the
            # Jacobian so it behaves the same across poses.
            JJt = J @ J.T
            scale = float(np.sqrt(np.trace(JJt) / 3.0)) or 1.0
            A = JJt + (damping * scale) ** 2 * np.eye(3)
            try:
                dth = J.T @ np.linalg.solve(A, err)
            except np.linalg.LinAlgError:
                dth = J.T @ np.linalg.pinv(A) @ err
            nd = float(np.linalg.norm(dth))
            if nd > max_step:
                dth = dth * (max_step / nd)
            alpha = 1.0
            best = list(th)
            best_n = n
            for _trial in range(12):
                cand = [th[i] + alpha * dth[i] for i in range(self.n_joints)]
                cand = [max(-joint_clamp, min(joint_clamp, v)) for v in cand]
                cn = float(np.linalg.norm(np.asarray(target_pos, dtype=float) - self.fk(cand)[0]))
                if cn < best_n:
                    best_n = cn
                    best = cand
                alpha *= 0.5
            if best_n >= n - 1e-12 and _ > 3:
                break
            th = best
        return th

    def jog_world(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, seed=None):
        """Translate the tool along world X/Y/Z (mm) via the PoE IK.

        Args:
            dx_mm, dy_mm, dz_mm: world-frame deltas in millimetres.
            seed: joint angles (radians); defaults to all zeros.

        Returns:
            New joint angles (radians) reaching the translated pose.
        """
        if seed is None:
            seed = [0.0] * self.n_joints
        cur = self.fk(seed)[0]
        target = cur + np.array([dx_mm, dy_mm, dz_mm]) / 1000.0
        return self.ik(target, seed)


# ---------------------------------------------------------------------------
# Factory wired to the canonical DH table
# ---------------------------------------------------------------------------

def make_astra_poe(arm=None) -> PoEModel:
    """Build the PoE model from the canonical Astra DH arm.

    Imported lazily so this module stays importable without the rest of the
    package; the DH table in ``create_astra_dh`` is the single source of truth,
    and the home pose M is exactly ``arm.forward([0]*6)`` (which fixes the old
    mismatch where the JS viewer's hand-entered M disagreed with the DH tool
    orientation).
    """
    if arm is None:
        from .kinematics import create_astra_dh
        arm = create_astra_dh()
    return poe_from_dh_arm(arm)


def poe_from_dh_arm(arm) -> PoEModel:
    """Derive a base-frame PoE model from a DH arm.

    For each revolute joint i the screw axis is the z-axis of the DH frame
    i-1 expressed in the base frame (evaluated at theta = 0), and the pivot is
    that frame's origin.  M is the DH forward kinematics at all-zero theta
    (including theta_offsets baked into the transform).
    """
    transforms = arm.forward_all([0.0] * arm.num_joints)
    n = arm.num_joints
    pivots = np.zeros((n, 3))
    axes = np.zeros((n, 3))
    T0 = arm.base_transform.copy()
    for i in range(n):
        if i == 0:
            T_prev = T0
        else:
            T_prev = transforms[i - 1]
        pivots[i] = T_prev[:3, 3]
        axes[i] = T_prev[:3, :3] @ np.array([0.0, 0.0, 1.0])
    home = arm.forward([0.0] * n)
    return PoEModel(pivots=pivots, axes=axes, home=home,
                    joint_names=list(arm.joint_names))
