"""DH Parameters and Forward Kinematics Module.

Provides:
  - DHParameter class for storing a single joint's DH parameters
  - DHArm class for a complete robot kinematic chain
  - Standard and Modified DH conventions
  - Forward kinematics (FK) computation
  - Jacobian computation
  - Utility functions for converting between representations
"""

import numpy as np
from .urdf_parser import URDFModel


def dh_transform(a, alpha, d, theta, convention="standard"):
    """Compute the homogeneous transformation matrix for a single joint.

    Args:
        a:      link length (mm/m)
        alpha:  link twist (radians)
        d:      link offset (mm/m)
        theta:  joint angle (radians)
        convention: "standard" or "modified"

    Returns:
        4x4 numpy array
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    if convention == "standard":
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,       ca,       d],
            [0,        0,        0,       1],
        ])
    else:
        # Modified DH (Craig)
        return np.array([
            [ct,      -st,      0,       a],
            [st * ca,  ct * ca, -sa, -d * sa],
            [st * sa,  ct * sa,  ca,  d * ca],
            [0,         0,       0,      1],
        ])


class DHParameter:
    """DH parameters for a single joint.

    Standard DH:  a_i-1, alpha_i-1, d_i, theta_i
    Modified DH:  a_i-1, alpha_i-1, d_i, theta_i  (same symbols, different transform order)
    """

    def __init__(self, a=0.0, alpha=0.0, d=0.0, theta=0.0, joint_name="",
                 convention="standard", theta_offset=0.0):
        self.a = float(a)           # link length
        self.alpha = float(alpha)   # link twist (radians)
        self.d = float(d)           # link offset
        self.theta = float(theta)   # joint angle (radians) — variable for revolute
        self.joint_name = joint_name
        self.convention = convention
        self.theta_offset = float(theta_offset)  # constant joint offset (radians)

    @property
    def transform(self):
        """4x4 transform for this joint at current theta (+ offset)."""
        return dh_transform(self.a, self.alpha, self.d,
                            self.theta + self.theta_offset, self.convention)

    def copy(self):
        return DHParameter(self.a, self.alpha, self.d, self.theta,
                           self.joint_name, self.convention, self.theta_offset)

    def to_dict(self):
        return {
            "a": self.a,
            "alpha": self.alpha,
            "d": self.d,
            "theta": self.theta,
            "theta_offset": self.theta_offset,
            "joint_name": self.joint_name,
        }

    @classmethod
    def from_dict(cls, data, convention="standard"):
        return cls(
            a=data.get("a", 0),
            alpha=data.get("alpha", 0),
            d=data.get("d", 0),
            theta=data.get("theta", 0),
            joint_name=data.get("joint_name", ""),
            convention=convention,
            theta_offset=data.get("theta_offset", 0),
        )

    def __repr__(self):
        return (f"DH(a={self.a:.4f}, alpha={self.alpha:.4f}, "
                f"d={self.d:.4f}, theta={self.theta:.4f})")


class DHArm:
    """Complete kinematic chain represented by DH parameters.

    Manages a list of DHParameter objects, computes forward kinematics,
    Jacobians, and provides utilities for parameter manipulation.
    """

    def __init__(self, name="robot", convention="standard"):
        self.name = name
        self.convention = convention
        self.dh_params = []          # list of DHParameter
        self.joint_names = []        # parallel list of joint names
        self.joint_limits = {}       # name -> (lower, upper)
        self.gear_ratios = {}        # name -> ratio
        self.home_position = {}      # name -> home angle (radians)
        self.base_transform = np.eye(4)

    @property
    def num_joints(self):
        return len(self.dh_params)

    def add_joint(self, dh, name="", limits=None, gear_ratio=1.0, home=0.0):
        """Add a joint to the kinematic chain.

        Args:
            dh: DHParameter instance
            name: joint name
            limits: (lower, upper) tuple in radians
            gear_ratio: motor-to-joint gear ratio
            home: home position in radians
        """
        self.dh_params.append(dh)
        if name:
            self.joint_names.append(name)
            self.joint_limits[name] = limits or (-np.pi, np.pi)
            self.gear_ratios[name] = gear_ratio
            self.home_position[name] = home

    def remove_joint(self, index):
        """Remove joint at index."""
        if 0 <= index < len(self.dh_params):
            name = self.joint_names[index] if index < len(self.joint_names) else ""
            self.dh_params.pop(index)
            if name:
                self.joint_names.pop(index)
                self.joint_limits.pop(name, None)
                self.gear_ratios.pop(name, None)
                self.home_position.pop(name, None)

    def set_thetas(self, joint_angles, degrees=False):
        """Set joint angles (theta values) from a list or dict.

        Args:
            joint_angles: list of angles or dict of {name: angle}
            degrees: if True, convert from degrees to radians
        """
        if isinstance(joint_angles, dict):
            for i, name in enumerate(self.joint_names):
                if name in joint_angles:
                    val = joint_angles[name]
                    self.dh_params[i].theta = np.radians(val) if degrees else float(val)
        else:
            for i in range(min(len(joint_angles), len(self.dh_params))):
                val = joint_angles[i]
                self.dh_params[i].theta = np.radians(val) if degrees else float(val)

    def get_thetas(self, degrees=False):
        """Get current joint angles as a list.

        Args:
            degrees: if True, convert from radians to degrees
        """
        angles = [p.theta for p in self.dh_params]
        if degrees:
            return [np.degrees(a) for a in angles]
        return angles

    def get_thetas_dict(self, degrees=False):
        """Get current joint angles as a dict keyed by name."""
        angles = self.get_thetas(degrees=degrees)
        return dict(zip(self.joint_names, angles))

    def forward(self, joint_angles=None):
        """Compute forward kinematics.

        Args:
            joint_angles: optional list/dict of joint angles.
                          If None, uses current theta values.

        Returns:
            4x4 homogeneous transform matrix of the end-effector
            in the base frame.
        """
        if joint_angles is not None:
            self.set_thetas(joint_angles)

        T = self.base_transform.copy()
        for dh in self.dh_params:
            T = T @ dh.transform
        return T

    def forward_all(self, joint_angles=None):
        """Compute FK for each joint and return all intermediate transforms.

        Returns:
            List of 4x4 matrices, one per joint (cumulative from base).
        """
        if joint_angles is not None:
            self.set_thetas(joint_angles)

        transforms = []
        T = self.base_transform.copy()
        for dh in self.dh_params:
            T = T @ dh.transform
            transforms.append(T.copy())
        return transforms

    def position(self, joint_angles=None):
        """Get end-effector position (x, y, z)."""
        T = self.forward(joint_angles)
        return T[:3, 3]

    def orientation(self, joint_angles=None):
        """Get end-effector orientation as 3x3 rotation matrix."""
        T = self.forward(joint_angles)
        return T[:3, :3]

    def jacobian(self, joint_angles=None, target_joints=None):
        """Compute geometric Jacobian.

        Args:
            joint_angles: list of joint angles. Uses current if not provided.
            target_joints: optional list of joint indices to compute Jacobian for.
                          Defaults to all joints.

        Returns:
            6xn numpy array: [linear_velocity; angular_velocity]
        """
        if joint_angles is not None:
            self.set_thetas(joint_angles)

        n = len(self.dh_params)
        if target_joints is None:
            target_joints = list(range(n))

        transforms = self.forward_all()
        T_ee = transforms[-1] if transforms else self.base_transform
        p_ee = T_ee[:3, 3]

        J = np.zeros((6, len(target_joints)))
        T_curr = self.base_transform.copy()

        for j_idx, dh_idx in enumerate(target_joints):
            dh = self.dh_params[dh_idx]
            T_curr = T_curr @ dh.transform if dh_idx > 0 else self.base_transform @ dh.transform

            z_i = T_curr[:3, 2]  # z-axis of joint frame
            p_i = T_curr[:3, 3]  # origin of joint frame

            if self.convention == "standard":
                if dh_idx < len(transforms):
                    z_i = transforms[dh_idx][:3, 2]
                    p_i = transforms[dh_idx][:3, 3]

            # For revolute/continuous joints
            J[:3, j_idx] = np.cross(z_i, p_ee - p_i)
            J[3:, j_idx] = z_i

        return J

    def compute_ik(self, target_pos, target_orient=None, max_iter=300, tol=1e-4,
                   joint_angles=None, retries=30, base_weight=0.0):
        """Inverse kinematics via Levenberg-Marquardt (scipy least_squares).

        Args:
            target_pos: desired [x, y, z] position
            target_orient: desired 3x3 rotation matrix (optional — if None,
                           only the position is solved)
            max_iter: maximum iterations
            tol: convergence tolerance
            joint_angles: initial guess (uses current if None)
            retries: number of random-restart attempts
            base_weight: regularizer that penalizes base-joint (J1/J2/J3)
                         deviation from the seed. Set > 0 to keep the arm
                         base orientation fixed and let the wrist (J4/J5/J6)
                         absorb reorientation — the "orientation lock" that
                         keeps the tool frame constant while jogging a single
                         Cartesian axis.

        Returns:
            List of joint angles (radians) or None if failed.
        """
        try:
            from scipy.optimize import least_squares
        except ImportError:
            # Fall back to the damped least-squares solver.
            return self._compute_ik_dls(target_pos, target_orient,
                                        max_iter, tol, joint_angles, retries)

        target_pos = np.array(target_pos, dtype=float)
        n = len(self.dh_params)

        use_orient = target_orient is not None

        # Warm start for the base regularizer.
        warm = None
        if joint_angles is not None:
            warm = np.array(joint_angles, dtype=float)

        # bounds from joint limits (soften the edge slightly so the solver
        # can approach the boundary)
        lower, upper = [], []
        for name in self.joint_names:
            if name in self.joint_limits:
                lo, hi = self.joint_limits[name]
                lower.append(lo - 0.05)
                upper.append(hi + 0.05)
            else:
                lower.append(-np.pi * 2)
                upper.append(np.pi * 2)

        def residual(q):
            self.set_thetas(q)
            T = self.forward()
            pos_err = target_pos - T[:3, 3]
            if not use_orient:
                # With the base regularizer, keep base joints near the seed.
                if base_weight > 0.0 and warm is not None:
                    reg = base_weight * (q[:3] - warm[:3])
                    return np.concatenate([pos_err, reg])
                return pos_err
            R_cur = T[:3, :3]
            R_err = target_orient @ R_cur.T
            orient_err = 0.5 * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ])
            # Scale orientation so it converges alongside position: a 1mm pos
            # error should weigh about the same as a 0.001 rad orient error.
            # (position is in meters, orientation in radians — both ~1e-3 for
            # the precision we need, so a modest weight keeps them balanced.)
            err = np.concatenate([pos_err, orient_err * 10.0])
            if base_weight > 0.0 and warm is not None:
                reg = base_weight * (q[:3] - warm[:3])
                err = np.concatenate([err, reg])
            return err

        # ── Phase 1: warm-start + moderate local restarts ───────────
        # (covers the common case: solution near the current/caller pose)
        guesses = []
        if joint_angles is not None:
            guesses.append(np.array(joint_angles, dtype=float))
        guesses.append(np.zeros(n))
        for _ in range(retries):
            guesses.append(np.random.uniform(-1.5, 1.5, size=n))
        guesses.append(np.full(n, -2.5))
        guesses.append(np.full(n, 2.5))

        candidates = []

        def _verify(x):
            """Clamp to hard limits and check the result reaches the target.
            Returns the clamped joint list, or None if it misses."""
            final = []
            for i, name in enumerate(self.joint_names):
                if name in self.joint_limits:
                    lo, hi = self.joint_limits[name]
                    final.append(float(np.clip(x[i], lo, hi)))
                else:
                    final.append(float(x[i]))
            self.set_thetas(final)
            T = self.forward()
            pe = np.linalg.norm(target_pos - T[:3, 3])
            if use_orient:
                R_err = target_orient @ T[:3, :3].T
                oe = 0.5 * np.linalg.norm(np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]))
            else:
                oe = 0.0
            if pe < 0.003 and oe < 0.01:
                return final
            return None

        def _run(guesses, iters, early_exit=False):
            """Run least-squares from each guess; optionally return the first
            verified candidate immediately (fast path for the common case where
            the warm start converges in one solve)."""
            for q0 in guesses:
                try:
                    res = least_squares(
                        residual, q0,
                        bounds=(lower, upper),
                        method="trf",
                        max_nfev=iters,
                        xtol=tol,
                        ftol=tol,
                        gtol=tol,
                    )
                except Exception:
                    continue
                candidates.append(res.x)
                if early_exit:
                    verified = _verify(res.x)
                    if verified is not None:
                        return verified
            return None

        # ── Phase 1: warm-start + moderate local restarts ───────────
        # Fast path (jog): only when the warm start is a *nearby* pose — its
        # FK pose must be close to the target. The world jog always passes
        # the previous pose (a few mm from the target), so this hits the
        # fast path: single least_squares solve → ~3 ms. A cold/fake warm
        # start (e.g. zeros for a random full-range pose) is far from the
        # target, so it keeps the full multi-restart behavior and still
        # converges to the best solution.
        if joint_angles is not None:
            warm_fk = self.forward(np.array(joint_angles, dtype=float))
            warm_pos_err = float(np.linalg.norm(target_pos - warm_fk[:3, 3]))
            near_target = warm_pos_err < 0.05  # 50mm — generous for jog steps
            if near_target:
                fast = _run(guesses[:1], max_iter, early_exit=True)
                if fast is not None:
                    return fast
        # Full phase 1: moderate local restarts, verify every candidate
        # (the lowest-cost one can be a bad local minimum while a
        # higher-cost candidate is the real solution). Prefer the verified
        # solution closest to the warm start so the jog stays on one IK
        # branch instead of hopping to a far alternate wrist configuration
        # between steps.
        _run(guesses, max_iter)
        warm = np.array(joint_angles, dtype=float) if joint_angles is not None else None
        best_verified = None
        best_dist = float("inf")
        for c in candidates:
            verified = _verify(c)
            if verified is None:
                continue
            if warm is not None:
                d = np.linalg.norm(np.array(verified) - warm)
            else:
                d = 0.0
            if d < best_dist:
                best_dist = d
                best_verified = verified
        if best_verified is not None:
            return best_verified

        # ── Phase 2: exhaustive search (boundary / alternate-branch) ──
        # Runs whenever no phase-1 candidate verified — not only when the
        # lowest cost was None (a bad local minimum can have the lowest
        # cost yet miss the target; blocking phase 2 then made results
        # depend on how many random restarts happened to be tried).
        # Full-range guesses reach alternate wrist branches (e.g. J6 at
        # -2.35) that the moderate [-1.5,1.5] restarts never sample.
        # Prefer the verified solution closest to the warm start (branch
        # stability) for the same reason as phase 1.
        for _ in range(retries):
            g = [np.random.uniform(lo, hi) for lo, hi in zip(lower, upper)]
            try:
                res = least_squares(
                    residual, np.array(g, dtype=float),
                    bounds=(lower, upper),
                    method="trf",
                    max_nfev=max_iter,
                    xtol=tol,
                    ftol=tol,
                    gtol=tol,
                )
            except Exception:
                continue
            verified = _verify(res.x)
            if verified is None:
                continue
            if warm is not None:
                d = np.linalg.norm(np.array(verified) - warm)
            else:
                d = 0.0
            if d < best_dist:
                best_dist = d
                best_verified = verified
        return best_verified

    def _compute_ik_dls(self, target_pos, target_orient=None, max_iter=300,
                        tol=1e-4, joint_angles=None, retries=6):
        """Damped least-squares IK fallback (used if scipy is missing)."""
        target_pos = np.array(target_pos, dtype=float)
        n = len(self.dh_params)
        guesses = []
        if joint_angles is not None:
            guesses.append(np.array(joint_angles, dtype=float))
        guesses.append(np.zeros(n))
        for _ in range(retries):
            guesses.append(np.random.uniform(-1.0, 1.0, size=n))

        use_orient = target_orient is not None
        best_q = None
        best_err = float("inf")

        for q in guesses:
            q = q.copy()
            for _ in range(max_iter):
                self.set_thetas(q)
                T = self.forward()
                pos_err = target_pos - T[:3, 3]
                if use_orient:
                    R_cur = T[:3, :3]
                    R_err = target_orient @ R_cur.T
                    orient_err = 0.5 * np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ])
                    err = np.concatenate([pos_err, orient_err])
                else:
                    err = pos_err
                cur_err = np.linalg.norm(err)
                if cur_err < best_err:
                    best_err = cur_err
                    best_q = q.copy()
                if np.linalg.norm(pos_err) < tol and (not use_orient or np.linalg.norm(err[3:]) < tol):
                    return [float(np.clip(q[i],
                                          self.joint_limits[self.joint_names[i]][0] if self.joint_names[i] in self.joint_limits else -np.pi*2,
                                          self.joint_limits[self.joint_names[i]][1] if self.joint_names[i] in self.joint_limits else np.pi*2))
                            for i in range(n)]
                J = self.jacobian()
                if J.shape[1] == 0:
                    break
                J_use = J if use_orient else J[:3, :]
                Jt = J_use.T
                lam = 0.01 + 0.2 * (cur_err / (1.0 + cur_err))
                try:
                    A = J_use @ Jt + lam * np.eye(J_use.shape[0])
                    dq = Jt @ np.linalg.solve(A, err)
                except np.linalg.LinAlgError:
                    break
                q = q + dq
                for i, name in enumerate(self.joint_names):
                    if name in self.joint_limits:
                        lo, hi = self.joint_limits[name]
                        if q[i] < lo or q[i] > hi:
                            q[i] = np.clip(q[i], lo - 0.5, hi + 0.5)

        if best_q is not None and best_err < 0.05:
            return [float(np.clip(best_q[i],
                                  self.joint_limits[self.joint_names[i]][0] if self.joint_names[i] in self.joint_limits else -np.pi*2,
                                  self.joint_limits[self.joint_names[i]][1] if self.joint_names[i] in self.joint_limits else np.pi*2))
                    for i in range(n)]
        return None

    @classmethod
    def from_urdf(cls, urdf_path, convention="standard"):
        """Build a DHArm by estimating DH parameters from a URDF file.

        The estimation projects each joint's origin and axis into the
        parent frame to determine a, alpha, d parameters.
        """
        model = URDFModel.load(urdf_path)
        arm = cls(model.name, convention)
        arm.base_transform = np.eye(4)

        for jname in model._joint_order:
            joint = model.joints[jname]
            if not joint.is_movable:
                continue

            # Extract DH parameters from joint origin and axis
            # In URDF, the joint origin defines the transform from
            # parent link to child link.
            T_joint = joint.origin
            axis = np.array(joint.axis)

            # For DH: a = distance along x from parent z to current z
            #         d = distance along parent z
            #         alpha = angle from parent z to current z about x
            #         theta = rotation about z (joint variable)

            # The URDF convention differs, so we estimate:
            a = np.sqrt(T_joint[0, 3]**2 + T_joint[1, 3]**2)
            d = T_joint[2, 3]

            # Extract rotation between parent frame and joint frame
            R = T_joint[:3, :3]
            # z-axis of joint
            z_j = R[:, 2]
            # alpha = angle between z_parent (0,0,1) and z_j
            z_p = np.array([0, 0, 1])
            cos_alpha = np.dot(z_p, z_j)
            alpha = np.arccos(np.clip(cos_alpha, -1, 1))
            if np.cross(z_p, z_j)[0] < 0:
                alpha = -alpha

            dh = DHParameter(
                a=a,
                alpha=alpha,
                d=d,
                theta=0.0,  # joint variable
                joint_name=jname,
                convention=convention,
            )

            limits = (joint.lower_limit, joint.upper_limit)
            arm.add_joint(dh, name=jname, limits=limits, home=0.0)

        return arm

    def to_dict(self):
        return {
            "name": self.name,
            "convention": self.convention,
            "num_joints": self.num_joints,
            "dh_params": [p.to_dict() for p in self.dh_params],
            "joint_names": list(self.joint_names),
            "joint_limits": {n: list(v) for n, v in self.joint_limits.items()},
            "gear_ratios": dict(self.gear_ratios),
            "home_position": {n: float(v) for n, v in self.home_position.items()},
        }

    def save(self, path):
        """Save DH configuration to a JSON file."""
        import json
        data = self.to_dict()
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    @classmethod
    def load_from_dict(cls, data):
        """Load DH configuration from a dictionary (from to_dict())."""
        arm = cls(data.get("name", "robot"), data.get("convention", "standard"))
        for i in range(len(data.get("dh_params", []))):
            dp = data["dh_params"][i]
            name = data["joint_names"][i] if i < len(data.get("joint_names", [])) else ""
            dh = DHParameter.from_dict(dp, arm.convention)
            limits = tuple(data.get("joint_limits", {}).get(name, (-np.pi, np.pi)))
            gear = data.get("gear_ratios", {}).get(name, 1.0)
            home = data.get("home_position", {}).get(name, 0.0)
            arm.add_joint(dh, name=name, limits=limits, gear_ratio=gear, home=home)
        return arm

    @classmethod
    def load(cls, path):
        """Load DH configuration from a JSON file."""
        import json
        with open(path) as f:
            data = json.load(f)
        return cls.load_from_dict(data)


# ── Preset DH Configurations ──────────────────────────────────────────

def create_astra_dh(convention="standard"):
    """Create a DHArm for the NITE 369 / Astra 6-DOF robot arm.

    DH table is the active nominal table from the NITE 369 v1.0 RoboDK model
    (Fanuc/Motoman standard convention). Units are meters:
      J1: a=0,     alpha=0,    d=0.290, theta_off=0
      J2: a=0.080, alpha=-90°, d=0,     theta_off=-90°
      J3: a=0.350, alpha=0,    d=0,     theta_off=0
      J4: a=0.045, alpha=-90°, d=0.335, theta_off=0
      J5: a=-0.004,alpha=+90°, d=0,     theta_off=0
      J6: a=0,     alpha=-90°, d=0.044, theta_off=180°
    Home (all zeros) = tool at [459, 0, 685] mm, matching RoboDK.
    """
    arm = DHArm("Astra 6-DOF", convention)

    params = [
        # (a, alpha_deg, d, theta_offset_deg, name, limits_rad, gear)
        (0,      0,   0.290, 0,    "j1", (-np.pi, np.pi),            22.8),
        (0.080, -90,  0,    -90,  "j2", (-np.pi * 200 / 180, np.pi * 200 / 180), 45.96),
        (0.350,  0,   0,     0,   "j3", (-np.pi * 200 / 180, np.pi * 200 / 180), 40.78125),
        (0.045, -90,  0.335, 0,   "j4", (-np.pi * 200 / 180, np.pi * 200 / 180), 24.75),
        (-0.004, 90,  0,     0,   "j5", (-np.pi * 200 / 180, np.pi * 200 / 180), 25.0),
        (0,     -90,  0.044, 180, "j6", (-np.pi * 200 / 180, np.pi * 200 / 180), 25.0),
    ]

    for a, alpha_deg, d, theta_off_deg, name, limits, gear in params:
        dh = DHParameter(a=a, alpha=np.radians(alpha_deg), d=d,
                         theta=0.0, joint_name=name, convention=convention,
                         theta_offset=np.radians(theta_off_deg))
        arm.add_joint(dh, name=name, limits=limits, gear_ratio=gear, home=0.0)

    return arm


def create_kuka_kr6_dh(convention="standard"):
    """Create a DHArm for a KUKA KR6 R900 robot."""
    arm = DHArm("KUKA KR6 R900", convention)

    params = [
        (0.025,  np.pi/2,  0.400, 0,  "j1", (-185, 185), 1.0),
        (0.315,  0,        0,     0,  "j2", (-135, 35),  1.0),
        (0.035,  np.pi/2,  0,     0,  "j3", (-135, 135), 1.0),
        (0,     -np.pi/2,  0.450, 0,  "j4", (-350, 350), 1.0),
        (0,      np.pi/2,  0,     0,  "j5", (-115, 115), 1.0),
        (0,      0,        0.080, 0,  "j6", (-350, 350), 1.0),
    ]

    for a, alpha, d, theta_off, name, limits, gear in params:
        dh = DHParameter(a=a, alpha=alpha, d=d, theta=theta_off,
                         joint_name=name, convention=convention)
        arm.add_joint(dh, name=name, limits=limits, gear_ratio=gear, home=0.0)
        arm.joint_limits[name] = (np.radians(lower), np.radians(upper)) if False else limits
    return arm


def radians(deg):
    return np.radians(deg)


def degrees(rad):
    return np.degrees(rad)
