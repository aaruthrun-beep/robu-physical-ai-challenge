"""Dual-quaternion forward kinematics — an independent cross-check.

Standard DH and PoE are algebraically very close (both are products of
homogeneous transforms), so a validation test that compares them can miss a
shared sign/order bug.  This module computes the same forward kinematics
through **unit dual quaternions** — a genuinely different algebraic route —
using only the DH parameter values as input.

Implementation notes
--------------------
A unit dual quaternion representing a rigid transform is

    q_hat = q_real + eps * q_dual,   eps^2 = 0

with real part the rotation quaternion (w, x, y, z) and dual part
``q_dual = 0.5 * t * q_real`` (t the translation as a pure quaternion).
Composition is just dual-quaternion multiplication, so the FK of a chain is
the product of the per-joint unit dual quaternions.  Per-joint transform:
``Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`` (standard DH), which is built as
three primitive dual quaternions multiplied in order.
"""

from __future__ import annotations

from typing import List, Sequence

import numpy as np


def _axis_angle(axis, angle):
    """Return a unit rotation dual quaternion (8-vector, dual part = 0)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = angle / 2.0
    s = np.sin(half)
    r = np.array([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s])
    return np.array([r[0], r[1], r[2], r[3], 0.0, 0.0, 0.0, 0.0])


def _translation(t):
    """Return a pure translation dual quaternion (8-vector)."""
    x, y, z = t
    # real part = 1 (no rotation), dual part = t/2 (pure quaternion)
    return np.array([1.0, 0.0, 0.0, 0.0, x / 2.0, y / 2.0, z / 2.0, 0.0])


def _qmul(a, b):
    """Multiply two quaternions (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _mul(a, b):
    """Multiply two dual quaternions (8-vector [w,x,y,z,wd,xd,yd,zd])."""
    ar, ad = a[:4], a[4:]
    br, bd = b[:4], b[4:]
    real = _qmul(ar, br)
    dual = _qmul(ar, bd) + _qmul(ad, br)
    return np.concatenate([real, dual])


def _rot_to_quat(R):
    """3x3 rotation matrix -> unit quaternion (w, x, y, z)."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])


def _matrix_to_dq(T):
    """4x4 homogeneous transform -> unit dual quaternion (8-vector).

    Convention used throughout this module:

        qd = 0.5 * qmul(tq, qr)     (translation premultiplied by rotation)

    with the matching inverse ``t = 2 * qmul(qd, qconj(qr))`` implemented in
    ``_dq_to_matrix``.  (This is the convention the dual-quaternion product
    ``_mul`` is defined for, so composition composes correctly.)
    """
    R = T[:3, :3]
    t = T[:3, 3]
    qr = _rot_to_quat(R)
    tq = np.array([0.0, t[0], t[1], t[2]])
    qd = 0.5 * _qmul(tq, qr)
    return np.concatenate([qr, qd])


def _dq_to_matrix(q):
    """Unit dual quaternion -> 4x4 homogeneous transform.

    Inverse of ``_matrix_to_dq``: t = 2 * qd * qconj(qr).
    """
    qr = q[:4]
    qd = q[4:]
    w, x, y, z = qr
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    # t = 2 * qd * qconj(qr)  (pure-quaternion vector part)
    t = 2 * _qmul(qd, np.array([w, -x, -y, -z]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t[1:]
    return T


def dh_joint_dq(a, alpha, d, theta):
    """Unit dual quaternion for one standard-DH joint.

    Builds the standard-DH matrix ``Rz(th) * Tz(d) * Tx(a) * Rx(alpha)`` and
    converts it to a dual quaternion with ``_matrix_to_dq``.  This avoids the
    fragile primitive-composition ordering entirely and is guaranteed to agree
    with ``DHArm.forward`` (which uses the same matrix).
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    T = np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])
    return _matrix_to_dq(T)


def dual_quaternion_fk(dh_params: Sequence[dict], base=None) -> np.ndarray:
    """Forward kinematics of a DH chain via dual quaternions.

    Args:
        dh_params: sequence of dicts with keys ``a``, ``alpha``, ``d`` and
            ``theta`` (all radians/meters; ``theta`` already includes any
            theta offset).
        base: optional 4x4 base transform (defaults to identity).

    Returns:
        4x4 homogeneous transform of the tool in the base frame.
    """
    q = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for p in dh_params:
        q = _mul(q, dh_joint_dq(p["a"], p["alpha"], p["d"], p["theta"]))
    T = _dq_to_matrix(q)
    if base is not None:
        T = np.asarray(base, dtype=float) @ T
    return T


def dual_quaternion_fk_arm(arm, joint_angles: Sequence[float]) -> np.ndarray:
    """FK of a DHArm at the given joint angles via dual quaternions.

    ``theta`` for each joint is ``joint_angles[i] + theta_offset``, matching
    ``DHArm.forward``.
    """
    params = []
    for i, p in enumerate(arm.dh_params):
        th = joint_angles[i] + p.theta_offset
        params.append({"a": p.a, "alpha": p.alpha, "d": p.d, "theta": th})
    return dual_quaternion_fk(params, base=arm.base_transform)
