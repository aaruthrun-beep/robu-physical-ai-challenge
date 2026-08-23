"""QuIK: second-order (Halley's method) inverse kinematics solver.

This is a Python/numpy port of the QuIK algorithm published in

    S. Lloyd, R. Irani, and M. Ahmadi, "Fast and Robust Inverse Kinematics
    for Serial Robots using Halley's Method," IEEE Transactions on Robotics,
    vol. 38, no. 5, pp. 2768-2780, Oct. 2022. doi: 10.1109/TRO.2022.3162954

Original C++ implementation (AGPL-3.0): https://github.com/steffanlloyd/quik

License
-------
This file is part of a derivative work of the QuIK library and is therefore
licensed under the GNU Affero General Public License v3.0 or later
(SPDX: AGPL-3.0-or-later).

QuIK's core idea is to use the *first and second* derivatives of the
kinematics function (the geometric Jacobian and the Hessian-vector product)
in a Halley (third-order Newton) update, giving roughly 2x the convergence
rate of plain Newton and dramatically fewer divergences on large initial
errors.

The port operates on an existing ``DHArm`` (standard or modified DH), so it
needs no separate Robot class: the DH table, joint types, joint direction
(Qsign), base and tool transforms all come from the arm.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

# Expose the algorithm constants used by the original C++.
ALGORITHM_QUIK = 0
ALGORITHM_NR = 1  # Newton-Raphson / Levenberg-Marquardt
ALGORITHM_BFGS = 2

BREAKREASON_TOLERANCE = 0
BREAKREASON_MIN_STEP = 1
BREAKREASON_MAX_ITER = 2
BREAKREASON_GRAD_FAILS = 3

BREAKREASON_NAMES = {
    BREAKREASON_TOLERANCE: "BREAKREASON_TOLERANCE",
    BREAKREASON_MIN_STEP: "BREAKREASON_MIN_STEP",
    BREAKREASON_MAX_ITER: "BREAKREASON_MAX_ITER",
    BREAKREASON_GRAD_FAILS: "BREAKREASON_GRAD_FAILS",
}


# ---------------------------------------------------------------------------
# Geometry helpers (port of quik::geometry)
# ---------------------------------------------------------------------------

def hgt_diff(T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
    """Twist error e (6-vector) between two homogeneous transforms.

    From [1] T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the
    Levenberg-Marquardt Method," IEEE T-RO 2011.
    e = [pos_err; rot_err], rot_err = 0.5 * (R1 @ R2^T - R2 @ R1^T) vee.
    """
    p1, p2 = T1[:3, 3], T2[:3, 3]
    R1, R2 = T1[:3, :3], T2[:3, :3]
    # (1/2) * vee(R1 R2^T - R2 R1^T)
    A = R1 @ R2.T - R2 @ R1.T
    rot_err = 0.5 * np.array([A[2, 1] - A[1, 2],
                              A[0, 2] - A[2, 0],
                              A[1, 0] - A[0, 1]])  # note sign convention
    return np.concatenate([p1 - p2, rot_err])


def hgt_inv(T: np.ndarray) -> np.ndarray:
    """Inverse of a homogeneous transform (fast: no matrix inversion)."""
    R = T[:3, :3]
    p = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


# ---------------------------------------------------------------------------
# Kinematic functions on a DHArm (port of quik::Robot)
# ---------------------------------------------------------------------------

def _arm_dh_array(arm) -> np.ndarray:
    """DH table as (n, 4) array [a, alpha, d, theta_offset] for the arm."""
    return np.array([[p.a, p.alpha, p.d, p.theta_offset] for p in arm.dh_params],
                    dtype=float)


def arm_forward_all(arm, Q: np.ndarray) -> np.ndarray:
    """Stacked FK: (n+1)*4 x 4, rows 0..n-1 per-joint frame, last = tool.

    Mirrors quik::Robot::FK (Tbase then successive Ak, tool applied last).
    Q is in radians, as the DH variable (theta_offset added internally).
    """
    n = arm.num_joints
    DH = _arm_dh_array(arm)
    T = np.zeros(((n + 1) * 4, 4))
    Tk = arm.base_transform.copy()
    for k in range(n):
        a, alpha, d, th_off = DH[k]
        th = Q[k] + th_off  # Qsign = +1 (all Astra joints are +)
        ct, st = np.cos(th), np.sin(th)
        ca, sa = np.cos(alpha), np.sin(alpha)
        Ak = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])
        Tk = Tk @ Ak
        T[4 * k:4 * k + 4] = Tk
    # Tool frame
    T[4 * n:4 * n + 4] = Tk @ arm.tool_transform if hasattr(arm, "tool_transform") else Tk
    return T


def arm_jacobian_from_T(arm, T: np.ndarray, include_tool: bool = True) -> np.ndarray:
    """Geometric Jacobian from the stacked FK (port of quik::Robot::jacobian)."""
    n = arm.num_joints
    J = np.zeros((6, n))
    tool_idx = n - 1 + int(include_tool)
    o_n = T[4 * tool_idx:4 * tool_idx + 3, 3]
    for i in range(n):
        if i > 0:
            z_im1 = T[4 * (i - 1):4 * (i - 1) + 3, 2]
            o_im1 = T[4 * (i - 1):4 * (i - 1) + 3, 3]
        else:
            z_im1 = arm.base_transform[:3, 2]
            o_im1 = arm.base_transform[:3, 3]
        # All Astra joints are revolute; prismatic would need the d branch.
        J[:3, i] = np.cross(z_im1, o_n - o_im1)
        J[3:, i] = z_im1
    return J


def arm_hessian_product(arm, J: np.ndarray, dQ: np.ndarray, A: np.ndarray) -> np.ndarray:
    """Add H*dQ to A in-place (port of quik::Robot::hessianProduct).

    Implements the revolute-only branch (all Astra joints are revolute).
    """
    n = arm.num_joints
    for k in range(n):
        jvk = J[:3, k]
        jwk = J[3:, k]
        Aw_sum = np.zeros(3)
        for i in range(k):
            cp = np.cross(J[3:, i], jvk)
            A[:3, k] += cp * dQ[i]
            A[:3, i] += cp * dQ[k]  # symmetry
            Aw_sum += J[3:, i] * dQ[i]
        A[3:, k] += np.cross(Aw_sum, jwk)
        A[:3, k] += np.cross(jwk, jvk) * dQ[k]
    return A


def characteristic_length(arm) -> float:
    """sum(sqrt(a_i^2 + d_i^2)) — used to scale max linear step."""
    DH = _arm_dh_array(arm)
    return float(np.sqrt(DH[:, 0] ** 2 + DH[:, 2] ** 2).sum())


# ---------------------------------------------------------------------------
# IKSolver (port of quik::IKSolver)
# ---------------------------------------------------------------------------

class IKSolver:
    """Generalized IK solver for a DHArm using QuIK / Newton / BFGS.

    Parameters mirror the original C++ class (see module docstring / IEEE-TRO
    paper).  ``algorithm`` is one of ALGORITHM_QUIK, ALGORITHM_NR,
    ALGORITHM_BFGS.
    """

    def __init__(
        self,
        arm,
        max_iterations: int = 100,
        algorithm: int = ALGORITHM_QUIK,
        exit_tolerance: float = 1e-12,
        minimum_step_size: float = 1e-14,
        relative_improvement_tolerance: float = 0.05,
        max_consecutive_grad_fails: int = 5,
        max_gradient_fails: int = 20,
        lambda_squared: float = 0.0,
        max_linear_step_size: Optional[float] = None,
        max_angular_step_size: float = 1.0,
        armijo_sigma: float = 1e-5,
        armijo_beta: float = 0.5,
    ):
        self.arm = arm
        self.n = arm.num_joints
        self.max_iterations = max_iterations
        self.algorithm = algorithm
        self.exit_tolerance = exit_tolerance
        self.minimum_step_size = minimum_step_size
        self.relative_improvement_tolerance = relative_improvement_tolerance
        self.max_consecutive_grad_fails = max_consecutive_grad_fails
        self.max_gradient_fails = max_gradient_fails
        self.lambda_squared = lambda_squared
        self.max_angular_step_size = max_angular_step_size
        self.armijo_sigma = armijo_sigma
        self.armijo_beta = armijo_beta
        if max_linear_step_size is None or max_linear_step_size <= 0:
            self.max_linear_step_size = 0.33 * characteristic_length(arm)
        else:
            self.max_linear_step_size = max_linear_step_size

    # -- helpers ---------------------------------------------------------

    def clamp_mag(self, e: np.ndarray) -> None:
        """Saturate linear/angular parts of the error (port of clampMag)."""
        if self.algorithm == ALGORITHM_BFGS:
            return
        lin_norm2 = e[:3] @ e[:3]
        ang_norm2 = e[3:] @ e[3:]
        if lin_norm2 > self.max_linear_step_size ** 2:
            e[:3] *= self.max_linear_step_size / np.sqrt(lin_norm2)
        if ang_norm2 > self.max_angular_step_size ** 2:
            e[3:] *= self.max_angular_step_size / np.sqrt(ang_norm2)

    def lsolve(self, A: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Damped linear solve: x = A^T (A A^T + lam^2 I)^-1 b (port of lsolve).

        Guards against the singular-matrix case (the Halley step can drive
        ``A A^T`` singular near a rank-deficient configuration) with a small
        default regularization and a pseudo-inverse fallback.
        """
        Astar = A @ A.T
        lam2 = self.lambda_squared
        if lam2 <= 0:
            lam2 = 1e-10  # tiny default so Astar is always invertible
        Astar = Astar + lam2 * np.eye(Astar.shape[0])
        try:
            return A.T @ np.linalg.solve(Astar, b)
        except np.linalg.LinAlgError:
            return A.T @ np.linalg.pinv(Astar) @ b

    def _pose_error(self, Twt: np.ndarray, Q: np.ndarray, T=None):
        if T is None:
            T = arm_forward_all(self.arm, Q)
        T_tool = T[4 * self.n:4 * self.n + 4]
        return hgt_diff(T_tool, Twt)

    # -- main entry -------------------------------------------------------

    def ik(self, Twt: np.ndarray, Q0, ret_star=False):
        """Solve IK for the target pose ``Twt`` from seed ``Q0``.

        Returns (Q_star, e_star, iter, break_reason).  With ``ret_star=False``
        returns Q_star only.
        """
        n = self.n
        Q = np.asarray(Q0, dtype=float).copy()
        dQ = np.zeros(n)
        e = np.zeros(6)
        iter_ = self.max_iterations
        break_reason = BREAKREASON_MAX_ITER
        grad_fail_counter = 0
        grad_fail_counter_total = 0
        e_prev_norm = 1e10

        # BFGS state
        H_i = None
        grad_i = None
        cost_i = 1e10

        for i in range(self.max_iterations):
            # FK + Jacobian + error (skip for BFGS after first iteration)
            if self.algorithm != ALGORITHM_BFGS or i == 0:
                T = arm_forward_all(self.arm, Q)
                J = arm_jacobian_from_T(self.arm, T, include_tool=True)
                e = hgt_diff(T[4 * n:4 * n + 4], Twt)

            e_norm = float(np.linalg.norm(e))
            if e_norm < self.exit_tolerance:
                break_reason = BREAKREASON_TOLERANCE
                iter_ = i
                break

            error_rel_improvement = (e_prev_norm - e_norm) / e_prev_norm
            if error_rel_improvement < self.relative_improvement_tolerance:
                grad_fail_counter += 1
                grad_fail_counter_total += 1
                if grad_fail_counter > self.max_consecutive_grad_fails:
                    break_reason = BREAKREASON_GRAD_FAILS
                    iter_ = i
                    break
                if grad_fail_counter_total > self.max_gradient_fails:
                    break_reason = BREAKREASON_GRAD_FAILS
                    iter_ = i
                    break
            else:
                grad_fail_counter = 1

            e_prev_norm = e_norm
            self.clamp_mag(e)

            if self.algorithm == ALGORITHM_QUIK:
                # Halley's method (QuIK)
                dQ = self.lsolve(J, e)
                dQ *= -0.5
                A = J.copy()
                arm_hessian_product(self.arm, J, dQ, A)
                dQ = self.lsolve(A, e)
                dQ *= -1.0
            elif self.algorithm == ALGORITHM_NR:
                dQ = self.lsolve(J, e)
                dQ *= -1.0
            elif self.algorithm == ALGORITHM_BFGS:
                if i == 0:
                    H_i = np.eye(n)
                    grad_i = J.T @ e
                    cost_i = 0.5 * float(e @ e)
                s0 = -H_i @ grad_i
                gamma = 1.0
                # line search
                while True:
                    T_ls = arm_forward_all(self.arm, Q + gamma * s0)
                    e_ls = hgt_diff(T_ls[4 * n:4 * n + 4], Twt)
                    cost_ip1 = 0.5 * float(e_ls @ e_ls)
                    if (cost_i - cost_ip1) >= -self.armijo_sigma * (grad_i @ (gamma * s0)):
                        break
                    gamma = self.armijo_beta * gamma
                    if gamma < self.minimum_step_size:
                        break
                if gamma < self.minimum_step_size:
                    break_reason = BREAKREASON_MIN_STEP
                    iter_ = i
                    break
                dQ = gamma * s0
                T = arm_forward_all(self.arm, Q + dQ)
                J = arm_jacobian_from_T(self.arm, T, include_tool=True)
                grad_ip1 = J.T @ e_ls
                y = grad_ip1 - grad_i
                rho = dQ @ y
                delta = y @ (H_i @ y)
                eps = np.finfo(float).eps
                if rho > delta and rho > eps:
                    H_i = H_i + ((1 + delta / rho) * np.outer(dQ, dQ)
                                 - np.outer(dQ, y) @ H_i
                                 - H_i @ np.outer(y, dQ)) / rho
                elif delta > eps and rho > eps:
                    H_i = H_i + np.outer(dQ, dQ) / rho - (H_i @ np.outer(y, y) @ H_i) / delta
                grad_i = grad_ip1
                cost_i = cost_ip1
            else:
                dQ.fill(0.0)

            Q = Q + dQ

            if dQ @ dQ < self.minimum_step_size ** 2:
                break_reason = BREAKREASON_MIN_STEP
                iter_ = i
                break

        if ret_star:
            return Q, e, iter_, break_reason
        return Q


def solve_ik(
    arm,
    target_pos,
    target_orient=None,
    seed=None,
    algorithm: int = ALGORITHM_QUIK,
    max_iterations: int = 100,
    retries: int = 15,
    tol: float = 1e-6,
    verify_tol: float = 1e-4,
    bounds=None,
):
    """Convenience wrapper: solve IK with random-restart + joint-limit clamp.

    Mirrors the behaviour of ``DHArm.compute_ik`` so it can be used as a
    drop-in alternative: it runs the QuIK solver from several seeds, verifies
    each candidate actually reaches the target, and returns the first verified
    joint list (radians) or None.

    Args:
        arm: DHArm.
        target_pos: [x, y, z] target (m).
        target_orient: optional 3x3 target rotation; None = position-only.
        seed: initial joint guess (radians); defaults to zeros.
        algorithm: ALGORITHM_QUIK / ALGORITHM_NR.
        max_iterations: per-seed iterations.
        retries: number of random restarts (in addition to the seed).
        tol: solver exit tolerance.
        verify_tol: how close (m) the verified solution must be to target.
        bounds: optional (lower, upper) arrays for joint clamping.

    Returns:
        Joint list (radians) or None.
    """
    n = arm.num_joints
    solver = IKSolver(arm, max_iterations=max_iterations, algorithm=algorithm,
                      exit_tolerance=tol)
    guesses = []
    if seed is not None:
        guesses.append(np.asarray(seed, dtype=float))
    guesses.append(np.zeros(n))
    rng = np.random.RandomState(0)
    for _ in range(retries):
        guesses.append(rng.uniform(-1.5, 1.5, size=n))
    guesses.append(np.full(n, -2.5))
    guesses.append(np.full(n, 2.5))

    T_wt = np.eye(4)
    T_wt[:3, 3] = np.asarray(target_pos, dtype=float)
    if target_orient is not None:
        T_wt[:3, :3] = np.asarray(target_orient, dtype=float)

    def _verify(q):
        q = np.clip(q, bounds[0], bounds[1]) if bounds else q
        T = arm_forward_all(arm, q)
        err = float(np.linalg.norm(T[4 * n:4 * n + 4, :3, 3] - T_wt[:3, 3]))
        if err < verify_tol:
            return q.tolist()
        return None

    for q0 in guesses:
        q = solver.ik(T_wt, q0)
        verified = _verify(q)
        if verified is not None:
            return verified
    return None
