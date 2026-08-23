# Astra (NITE 369) Kinematics — README

**Location:** `D:\projects\test\astra_studio\astra_studio\core\KINEMATICS_README.md`

This document explains **how every kinematics quantity is calculated** for the
Astra 6-DOF robot arm, the **exact data** required, how FK/IK are computed and
verified against **RoboDK**, how trajectory interpolation works, and — honestly —
**what was found wrong / missed** during the unification work.

All code this README describes lives in **new files only** (the originals were
never modified):

| File | Purpose |
|---|---|
| `core/kinematics.py` *(original, read-only)* | Canonical **standard-DH** table + `DHArm` (FK, Jacobian, scipy IK) |
| `core/astra_kinematics.py` *(new)* | Single source of truth: DH → PoE converters, JS constant generator, `verify_models` |
| `core/poe_kinematics.py` *(new)* | Python **Product-of-Exponentials** FK/Jacobian/IK (mirrors the JS viewer math) |
| `core/dual_quaternion.py` *(new)* | Independent **dual-quaternion** FK cross-check |
| `core/quik_ik.py` *(new, AGPL-3.0)* | **Halley's-method** (second-order) IK solver, ported from steffanlloyd/quik |
| `tools/build_kinematics_assets.py` *(new)* | Generates `stl_embed_v2/astra_kinematics.js` + `nite369.urdf` from the DH table |
| `gui/stl_embed_v2/` *(new)* | Upgraded Three.js viewer (joint-frame viz, consistency readout) |
| `gui/threejs_viewport_v2.py` *(new)* | Parallel bridge serving `stl_embed_v2/` |
| `tests/test_kinematics_unified.py` *(new)* | Cross-model equivalence + IK round-trip + JS round-trip tests |

---

## 1. The canonical data (everything required for the calculation)

The **standard DH table** is the single source of truth. Units: **meters** for
lengths, **radians** for angles.

| Joint | a (m) | α (deg) | d (m) | θ_off (deg) | joint limits (deg) | gear ratio |
|---|---|---|---|---|---|---|
| j1 | 0 | 0 | 0.290 | 0 | ±180 | 22.8 |
| j2 | 0.080 | −90 | 0 | −90 | ±200 | 45.96 |
| j3 | 0.350 | 0 | 0 | 0 | ±200 | 40.78125 |
| j4 | 0.045 | −90 | 0.335 | 0 | ±200 | 24.75 |
| j5 | −0.004 | +90 | 0 | 0 | ±200 | 25.0 |
| j6 | 0 | −90 | 0.044 | 180 | ±200 | 25.0 |

Home (θ = 0 for all joints) tool pose:

```
position = [379, -471, 290] mm
rotation = R = [[0,-1,0],
                [1, 0,0],
                [0, 0,1]]
```

> ⚠️ **Correction found:** the comment in the original `create_astra_dh()` claims
> home = `[459, 0, 685] mm`. That is **stale and wrong** — the actual DH table
> produces `[379, -471, 290] mm`. The RoboDK 100/100 validation validates the
> *table's* FK/IK round-trip, not that stale home-point claim.

---

## 2. How FK (forward kinematics) is calculated

### 2a. Standard DH transform (per joint)

```
T_i(a, alpha, d, theta) =
  [ cosθ,   -sinθ·cosα,  sinθ·sinα,  a·cosθ ]
  [ sinθ,    cosθ·cosα, -cosθ·sinα,  a·sinθ ]
  [   0,          sinα,       cosα,       d  ]
  [   0,            0,          0,       1  ]
```
where `θ = θ_joint + θ_offset` (the offset is constant and baked in).

**FK:** multiply all six joint transforms with the base transform:
```
T_tool(θ) = T_base · T_1 · T_2 · T_3 · T_4 · T_5 · T_6
```
Implementation: `DHArm.forward()` / `DHArm.forward_all()` in `core/kinematics.py`.

### 2b. Product-of-Exponentials (PoE) form — derived from the DH table

The PoE model expresses the same FK using **screw axes in the base frame**:

```
T(θ) = e^[S1]θ1 · e^[S2]θ2 · … · e^[S6]θ6 · M
```

Derivation from DH (`astra_kinematics.dh_to_poe`):

- `M` = DH FK at θ = 0 (the home 4×4 above).
- For each joint i, take the DH frame **before** joint i at home:
  - **pivot** `q_i` = origin of that frame (meters)
  - **axis** `ω_i` = that frame's z-axis expressed in base frame

Exact values (also emitted into `stl_embed_v2/astra_kinematics.js`):

```
POE_PIVOTS (m):
  [0,       0,       0.29 ]
  [0,      -0.08,    0.29 ]
  [0,      -0.43,    0.29 ]
  [0.335,  -0.475,   0.29 ]
  [0.335,  -0.471,   0.29 ]

POE_AXES:
  [0,0,1]  [0,0,1]  [1,0,0]  [1,0,0]  [0,0,-1]  [1,0,0]

POE_HOME_ROT = [[0,-1,0],[1,0,0],[0,0,1]]
POE_HOME_POS = [0.379, -0.471, 0.290]
```

The screw exponential (Rodrigues) for a unit screw `S = (ω, v)`:

```
e^[S]θ = [ e^[ω]θ ,  (I − e^[ω]θ)(ω×v) + ω(ω·v)θ ]
         [   0    ,                1                ]
```
with `v = −ω × q` for a revolute joint at pivot q.

**Why PoE matters:** the same constants are **generated from Python** into the
JS viewer, so the Python and JavaScript FK can never drift apart again.

### 2c. Dual-quaternion FK (independent cross-check)

Same FK computed through **unit dual quaternions** — a genuinely different
algebraic route, so a shared sign/order bug in DH and PoE would still be caught:

```
q̂ = q_real + ε·q_dual          (ε² = 0)
q_dual = ½·t·q_real            (t = translation as pure quaternion)
T(θ) = q̂_1 · q̂_2 · … · q̂_6    (dual-quaternion product)
```

Convention used (verified self-consistent):

```
_matrix_to_dq:  qd = 0.5 * qmul(tq, qr)         (t premultiplied by r)
_dq_to_matrix:  t  = 2 * qmul(qd, qconj(qr))
```

---

## 3. How IK (inverse kinematics) is calculated

### 3a. Existing solver — `DHArm.compute_ik` (scipy Levenberg–Marquardt)

1. Build residual: position error + orientation error (axis-angle, weighted ×10).
2. Solve with `scipy.optimize.least_squares(..., method="trf", bounds=joint_limits)`.
3. Three phases:
   - **Fast path:** warm-start close to target → single solve.
   - **Phase 1:** warm start + moderate random restarts (verify each candidate).
   - **Phase 2:** exhaustive random restarts when phase 1 fails.
4. Clamp to joint limits, verify the result actually reaches the target.

### 3b. PoE IK — `PoEModel.ik` (damped least squares, dual form)

```
dθ = Jᵀ (J·Jᵀ + (λ·σ)² I)⁻¹ err        err = target − FK(θ)
```

- **Jacobian:** analytic *chain* Jacobian, column `i` = `z_{i−1} × (p_ee − p_{i−1})`
  (frame **before** joint i at the current config). Verified against finite
  differences to ~1e-10.
- **Adaptive damping:** `λ·σ` where `σ = sqrt(trace(J·Jᵀ)/3)` — behaves the same
  across poses.
- **Step clamp:** max 0.05 rad/iter + line search (Armijo-style halving).

Used for the world-jog X/Y/Z moves: `jog_world(dx, dy, dz)` translates the tool
along a world axis by exactly the requested mm.

### 3c. QuIK — Halley's method (`core/quik_ik.py`, AGPL-3.0)

Port of [steffanlloyd/quik](https://github.com/steffanlloyd/quik) (IEEE-TRO 2022,
"Fast and Robust Inverse Kinematics for Serial Robots using Halley's Method").

Uses the **first and second derivatives** (Jacobian + Hessian–vector product) in
a Halley (third-order Newton) update:

```
dQ = −lsolve(A, e)      with A = J + H·(−½·lsolve(J, e))
```

Tuning knobs (mirror the C++): `exit_tolerance`, `minimum_step_size`,
`relative_improvement_tolerance`, `max_consecutive_grad_fails`,
`max_gradient_fails`, `lambda_squared`, `max_linear_step_size`,
`max_angular_step_size`, Armijo line-search parameters.

> ⚠️ **Status (known issue):** QuIK currently breaks early on `BREAKREASON_GRAD_FAILS`
> because the 5% `relative_improvement_tolerance` is too aggressive for this arm,
> leaving residual error (pos ~1.6 mm, orient ~0.18 rad). The solver *does*
> converge in a few iterations; the break logic needs relaxing. This is the one
> remaining open item in `tests/test_kinematics_unified.py`.

---

## 4. Verification against RoboDK

RoboDK is an **optional calibration/reference oracle** — not a runtime dependency.
The bridge (`robodk_bridge.py` + `robodk_cli.py`) shells out to RoboDK's embedded
Python over a temp-file JSON protocol:

| op | RoboDK call |
|---|---|
| `fk` | `robot.SolveFK(joints)` |
| `ik` | `robot.SolveIK(pose, guess)` |
| `get_dh` | `robot.Links()` |
| `move_joints` | `robot.MoveJ(...)` |

### The 100/100 test (`tests/test_ik_vs_robodk.py`)

- **100 random full-range poses:** random joint angles → DH FK → IK back (no warm
  start) → compare FK(IK(FK)) to the target. Threshold: **≤5 mm position,
  ≤1e-3 rad orientation**.
- **100 jog-pattern steps:** warm-start IK with ≤10 mm deltas. Threshold:
  **≤3 mm / 1e-2 rad**.
- Result: **100/100 passed** — this is the DH table's claim to fame, and it is
  what `create_astra_dh` is validated against.

### New unified validation (`tests/test_kinematics_unified.py`)

- **DH ≡ PoE ≡ dual-quaternion FK** over 100 random poses → max error ~1e-15.
- PoE `ik` round-trips (reachable targets) → <1e-5.
- World-jog deltas (≤10 mm) move the tool by exactly the requested delta → <1e-3.
- **JS round-trip:** the generated `astra_kinematics.js` is run under **Node**;
  its `fkPoE` matches the Python FK to 1e-9.
- Generator `--check` self-test: home pose + JS constants + cross-model agreement.

---

## 5. Interpolation of the data (trajectories)

`core/path_planning.py` provides the interpolation layer that sits on top of FK/IK:

| Function | What it does |
|---|---|
| `generate_joint_cubic` | Cubic polynomial per joint between two waypoints |
| `generate_joint_quintic` | Quintic (smooth accel) per joint |
| `generate_joint_trapezoidal` | Trapezoidal velocity profile |
| `generate_cartesian_linear` | Linear Cartesian path — interpolate tool pose, **solve IK per point** |
| `slerp` | Spherical linear interpolation of orientation quaternions |
| `generate_multi_waypoint_trajectory` | Chain several waypoints |
| `resample_trajectory` | Re-sample to a fixed time step |

**How they connect:** you interpolate **in joint space** (cheap, always reachable
between adjacent poses) or **in Cartesian space** (interpolate position linearly +
orientation with `slerp`, then call IK at every sample to get joint commands).
The PoE/QuIK solvers are the engines that make Cartesian interpolation possible.

---

## 6. What we missed / corrections made during unification

| # | Issue found | Fix |
|---|---|---|
| 1 | **Stale home pose:** `create_astra_dh` comment claims home = [459, 0, 685] mm; the table actually gives [379, −471, 290] mm | Documented; all constants now derive from the actual table |
| 2 | **JS constants hand-duplicated** in `stl_embed/index.html`, could drift from DH | `poe_to_js()` generates them from DH into `stl_embed_v2/astra_kinematics.js` |
| 3 | **Dual-quaternion translation convention** was inconsistent (`_matrix_to_dq` vs `_dq_to_matrix` vs `_mul`) → DQ FK diverged from DH by 1.37 | Standardized on `qd = ½·t·qr`, `t = 2·qd·qr*` → all models agree ~1e-15 |
| 4 | **PoE Jacobian formula wrong** (naive base-frame screw transport disagreed with finite-diff by 0.45) | Replaced with the chain Jacobian `z_{i−1} × (p_ee − p_{i−1})` → matches to 1e-10 |
| 5 | **PoE IK solved `JᵀJ + λI`** with Gaussian elimination — `JᵀJ` is singular (3×6 → rank ≤ 3) | Switched to dual form `Jᵀ(JJᵀ + λ²I)⁻¹` with adaptive damping + step clamp |
| 6 | **QuIK `lsolve` crashed** on singular `AAᵀ` in the Halley step | Added default regularization + pinv fallback |
| 7 | **QuIK breaks early on `GRAD_FAILS`** (5% improvement tolerance too strict) | **Still open** — next fix: relax `relative_improvement_tolerance` / `max_consecutive_grad_fails` |
| 8 | `create_kuka_kr6_dh` has a dead bug (`lower`/`upper` undefined at line 690) | Not touched (original file); noted only |

---

## 7. Quick reference — commands

```bash
# Regenerate JS constants + URDF into stl_embed_v2/ (self-validates)
python astra_studio/tools/build_kinematics_assets.py --check

# Run the unified validation suite
python -m pytest tests/test_kinematics_unified.py -q

# Run the RoboDK 100/100 round-trip test
python -m pytest tests/test_ik_vs_robodk.py -q
```
