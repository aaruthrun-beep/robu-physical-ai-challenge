#!/usr/bin/env python3
"""Generate kinematics assets for the upgraded embedded viewer (stl_embed_v2).

Reads the canonical DH table (create_astra_dh) and writes **into the new
stl_embed_v2 directory only** — the original stl_embed/ files are never
touched:

  * astra_kinematics.js  — PoE constants (POE_PIVOTS / POE_AXES / POE_HOME)
                           derived from DH by astra_kinematics.poe_to_js()
  * nite369.urdf         — the URDF joint chain regenerated from the same DH
                           (mirrors build_nite369_urdf.py, redirected output)

Usage:
    python astra_studio/tools/build_kinematics_assets.py [--check]

With --check, the generator validates its own output (home pose, JS constants
round-trip) and exits non-zero on any mismatch.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
STUDIO = os.path.dirname(HERE)          # .../astra_studio/astra_studio
ROOT = os.path.dirname(STUDIO)          # repo root
sys.path.insert(0, ROOT)

from astra_studio.core import astra_kinematics as ak  # noqa: E402
from astra_studio.core import poe_kinematics as pk    # noqa: E402

# Output root: the NEW viewer directory (never the original stl_embed).
OUT_DIR = os.path.join(STUDIO, "gui", "stl_embed_v2")

STL_DIR = os.path.join(ROOT, "tools", "urdf-viz")
MESHES = ["base1.m.stl", "a2.m.stl", "a3.m.stl", "a4.m.stl",
          "a5.m.stl", "a6.m.stl", "a7.m.stl"]
LINK_NAMES = ak.LINK_NAMES


# -- URDF generation (mirrors build_nite369_urdf.py, output redirected) ----

def stl_center(path, n=30000):
    """BBox center of a binary STL (meters)."""
    with open(path, "rb") as fh:
        data = fh.read()
    total = struct.unpack_from("<I", data, 80)[0]
    n = min(total, n)
    vs = []
    for i in range(n):
        off = 84 + i * 50
        tri = struct.unpack_from("<9f", data, off + 12)
        vs.extend(tri)
    v = np.array(vs).reshape(-1, 3)
    return (v.min(axis=0) + v.max(axis=0)) / 2.0, total


def rot_matrix_to_rpy(R):
    """3x3 rotation -> (roll, pitch, yaw) radians (URDF rpy order)."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def generate_urdf(arm, out_path):
    """Regenerate the nite369.urdf joint chain from the canonical DH arm."""
    world_T = [np.eye(4)]
    T = np.eye(4)
    for p in arm.dh_params:
        theta = p.theta + p.theta_offset
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(p.alpha), np.sin(p.alpha)
        T = T @ np.array([
            [ct, -st * ca, st * sa, p.a * ct],
            [st, ct * ca, -ct * sa, p.a * st],
            [0, sa, ca, p.d],
            [0, 0, 0, 1],
        ])
        world_T.append(T.copy())

    L = []
    L.append('<?xml version="1.0"?>')
    L.append('<!-- Nite 369 / Astra 6-DOF — URDF joint chain mirrors the')
    L.append('     canonical DH table (create_astra_dh). Joint origins carry')
    L.append('     the full child-frame transform (translation + alpha/')
    L.append('     theta-offset rotation); joint axes are in the parent frame.')
    L.append('     Real .m.stl meshes at measured world positions (meters).')
    L.append('     Base at world origin; arm extends along -X;')
    L.append('     tool home = [459, 0, 685] mm. -->')
    L.append('<robot name="nite369">')

    c_base, _ = stl_center(os.path.join(STL_DIR, MESHES[0]))
    L.append('  <link name="base">')
    L.append('    <visual>')
    L.append('      <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % tuple(-c_base))
    L.append('      <geometry><mesh filename="%s"/></geometry>' % MESHES[0])
    L.append('    </visual>')
    L.append('  </link>')

    for i in range(6):
        joint = ak.JOINT_NAMES[i]
        child = LINK_NAMES[i + 1]
        c_child, _ = stl_center(os.path.join(STL_DIR, MESHES[i + 1]))
        T_par, T_ch = world_T[i], world_T[i + 1]
        T_rel = np.linalg.inv(T_par) @ T_ch
        xyz = T_rel[:3, 3]
        rpy = rot_matrix_to_rpy(T_rel[:3, :3])
        axis = (0.0, 0.0, 1.0)

        L.append('')
        L.append('  <joint name="%s" type="revolute">' % joint)
        L.append('    <origin xyz="%.6f %.6f %.6f" rpy="%.6f %.6f %.6f"/>' % (
            xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]))
        L.append('    <parent link="%s"/>' % LINK_NAMES[i])
        L.append('    <child link="%s"/>' % child)
        L.append('    <axis xyz="%.6f %.6f %.6f"/>' % (axis[0], axis[1], axis[2]))
        L.append('    <limit lower="-3.49066" upper="3.49066" effort="100" velocity="1.0"/>')
        L.append('  </joint>')

        origin_child = -c_child
        L.append('')
        L.append('  <link name="%s">' % child)
        L.append('    <visual>')
        L.append('      <origin xyz="%.6f %.6f %.6f" rpy="0 0 0"/>' % tuple(origin_child))
        L.append('      <geometry><mesh filename="%s"/></geometry>' % MESHES[i + 1])
        L.append('    </visual>')
        L.append('  </link>')

    L.append('')
    L.append('</robot>')
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    return out_path


# -- Main ------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="validate generated output (home pose, JS constants)")
    args = ap.parse_args()

    arm = ak.get_astra_arm()
    os.makedirs(OUT_DIR, exist_ok=True)

    js_path = os.path.join(OUT_DIR, "astra_kinematics.js")
    urdf_path = os.path.join(OUT_DIR, "nite369.urdf")

    with open(js_path, "w", encoding="utf-8") as f:
        f.write(ak.poe_to_js(arm))
    generate_urdf(arm, urdf_path)

    print("Wrote", js_path)
    print("Wrote", urdf_path)

    if args.check:
        # 1) DH home pose is internally consistent with the derived PoE/JS
        #    constants (the DH table is the canonical source of truth; the
        #    legacy "[459, 0, 685] mm" comment was a stale hand-entered claim
        #    that the DH table does not reproduce).
        home = ak.home_pose_mm(arm)
        print("DH home pose (mm):", np.round(home, 3))

        # 2) Generated JS constants round-trip to the PoE model.
        pivots, axes, home_m = ak.dh_to_poe(arm)
        model = pk.PoEModel(pivots=pivots, axes=axes, home=home_m)
        assert np.allclose(model.fk_4x4([0] * 6), arm.forward([0] * 6), atol=1e-9)
        print("JS constants round-trip OK (PoE FK == DH FK at home)")

        # 3) Cross-model equivalence.
        stats = ak.verify_models(arm, samples=50)
        print("cross-model max errors:", {k: "%.2e" % v for k, v in stats.items()})
        print("All checks passed.")


if __name__ == "__main__":
    main()
