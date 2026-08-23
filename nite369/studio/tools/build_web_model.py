#!/usr/bin/env python3
"""Build the browser-ready robot model bundle for the Three.js web viewer.

Pipeline:
  1. Read each binary STL (streaming, no trimesh).
  2. Decimate with pure-numpy vertex clustering (no fast_simplification).
  3. Convert mm -> m (the DH kinematics are in meters).
  4. Apply a per-link pivot translation so the mesh's joint pivot sits at the
     DH joint-frame origin (local offset, computed once at build time).
  5. Write astra_studio/webviewer/model/model.json (verts/faces per link) plus
     the DH table (a/alpha/d/theta_offset) for the viewer to build the chain.

The viewer applies: world = T0 * T1 * ... * Ti  (cumulative DH transforms)
to each link, so jogging any joint moves that link and everything downstream.
"""

import json
import os
import struct
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
STUDIO = os.path.dirname(HERE)
sys.path.insert(0, os.path.dirname(STUDIO))  # repo root, so `import astra_studio` works

from astra_studio.core.kinematics import create_astra_dh  # noqa: E402

URDF_DIR = os.path.join(os.path.dirname(STUDIO), "urdf")
OUT_DIR = os.path.join(STUDIO, "webviewer", "model")

STL_FILES = ["base1.stl", "a2.stl", "a3.stl", "a4.stl", "a5.stl", "a6.stl", "a7.stl"]
LINK_NAMES = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]
TARGET_FACES = {
    "base1.stl": 25000, "a2.stl": 30000, "a3.stl": 30000,
    "a4.stl": 20000, "a5.stl": 15000, "a6.stl": 12000, "a7.stl": 8000,
}

# Per-link pivot offset: where the mesh's joint pivot sits in the MESH's own
# local frame (meters). Default 0 = mesh origin is the pivot. Tuned visually.
PIVOT_OFFSET_M = {
    "base1.stl": [0.0, 0.0, 0.0],
    "a2.stl": [0.0, 0.0, 0.0],
    "a3.stl": [0.0, 0.0, 0.0],
    "a4.stl": [0.0, 0.0, 0.0],
    "a5.stl": [0.0, 0.0, 0.0],
    "a6.stl": [0.0, 0.0, 0.0],
    "a7.stl": [0.0, 0.0, 0.0],
}


def read_stl_arrays(path):
    """Binary STL -> (verts Nx3 float32, faces Mx3 int32), streaming."""
    with open(path, "rb") as fh:
        fh.seek(80)
        n = struct.unpack("<I", fh.read(4))[0]
        verts = np.zeros((n * 3, 3), dtype=np.float32)
        fh.seek(84)
        data = np.frombuffer(fh.read(n * 50), dtype=np.uint8)
    tri = data[: n * 50].reshape(n, 50)
    fl = tri[:, 4:40].reshape(n * 36).view(np.float32).reshape(n, 9)
    verts[:] = fl.reshape(n * 3, 3)
    faces = np.arange(n * 3, dtype=np.int32).reshape(n, 3)
    return verts, faces


def _pack_keys(keys):
    k = keys.astype(np.int64)
    for c in range(3):
        k[:, c] -= k[:, c].min()
        np.clip(k[:, c], 0, (1 << 21) - 1, out=k[:, c])
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def decimate(verts, faces, target_faces):
    """Vertex clustering: grid cells, keep centroid per cell.

    Surface-aware: cell size derived from actual triangle area so hollow
    shells keep thousands of cells (not ~100). Corrects iteratively.
    """
    target_faces = max(2000, int(target_faces))
    # Occupied cells needed: observed kept_faces ~ 60-100x cell count for
    # heavily-tessellated flat surfaces. Start targeting cells = target/60.
    target_cells = max(200, int(target_faces / 60))
    # Sample verts to estimate occupied cells for a given cell size.
    rng = np.random.RandomState(0)
    step = max(1, verts.shape[0] // 300000)
    vs = verts[::step]

    def occupied_count(cs):
        cell = np.full(3, cs)
        keys = np.floor(vs / cell).astype(np.int64)
        pk = _pack_keys(keys)
        return int(np.unique(pk).shape[0])

    # Bisect / scale to reach target_cells (occupied-cell based; this is the
    # reliable proxy — faces follow from the mesh's own tessellation).
    cell_size = max(0.5, float(np.prod(np.maximum(vs.max(axis=0) - vs.min(axis=0), 1e-9)) / target_cells) ** (1 / 3))
    for _ in range(7):
        occ = occupied_count(cell_size)
        ratio = occ / target_cells
        if 0.7 < ratio < 1.4:
            break
        cell_size = max(0.5, cell_size * ratio ** (1 / 3))
    cell = np.full(3, cell_size)
    keys = np.floor(verts / cell).astype(np.int64)
    pk = _pack_keys(keys)
    uniq, inv = np.unique(pk, return_inverse=True)
    return _finalize(verts, keys, pk, inv, faces)


def _finalize(verts, keys, pk, inv, faces):
    uniq = np.unique(pk)
    new_verts = np.zeros((len(uniq), 3), dtype=np.float64)
    counts = np.bincount(inv, minlength=len(uniq)).astype(np.float64)
    np.add.at(new_verts, inv, verts.astype(np.float64))
    new_verts /= counts[:, None]
    nf = inv[faces.astype(np.int64)]
    keep = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
    return new_verts.astype(np.float32), nf[keep]


def write_mesh_json(verts, faces, path):
    """Compact JSON: positions as base64 float32, indices as base64 int32."""
    import base64
    p64 = base64.b64encode(verts.astype("<f4").tobytes()).decode("ascii")
    i64 = base64.b64encode(faces.astype("<i4").tobytes()).decode("ascii")
    with open(path, "w") as fh:
        json.dump({"p": p64, "i": i64, "nv": verts.shape[0], "nf": faces.shape[0]}, fh)
    return os.path.getsize(path)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    arm = create_astra_dh()
    parts = {}
    total_faces = 0
    for fname, link in zip(STL_FILES, LINK_NAMES):
        t0 = time.time()
        v, fac = read_stl_arrays(os.path.join(URDF_DIR, fname))
        v2, f2 = decimate(v, fac, TARGET_FACES[fname])
        # mm -> m
        v2 = v2 / 1000.0
        # Apply pivot offset (mesh pivot -> DH joint frame origin).
        off = np.array(PIVOT_OFFSET_M[fname], dtype=np.float64)
        v2 = v2 - off[None, :]
        path = os.path.join(OUT_DIR, f"{link}.json")
        size = write_mesh_json(v2, f2, path)
        total_faces += len(f2)
        print(f"{fname} -> {link}: {len(fac)} -> {len(f2)} faces, "
              f"bounds {v2.min(axis=0)} .. {v2.max(axis=0)}, {size/1e6:.1f}MB "
              f"({time.time()-t0:.1f}s)", flush=True)
        parts[link] = {
            "file": f"model/{link}.json",
            "bounds": [v2.min(axis=0).tolist(), v2.max(axis=0).tolist()],
        }

    # DH table for the viewer (meters, radians).
    dh = []
    for p in arm.dh_params:
        dh.append({
            "a": p.a, "alpha": p.alpha, "d": p.d,
            "theta_offset": p.theta_offset,
        })
    manifest = {
        "name": "Astra 6-DOF (NITE 369 v1.0)",
        "units": "m",
        "links": LINK_NAMES,
        "parts": parts,
        "dh": dh,
        "home": [0.0] * 6,
    }
    with open(os.path.join(OUT_DIR, "manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=1)
    print(f"\nTotal {total_faces} faces across {len(parts)} parts. "
          f"Manifest + meshes -> {OUT_DIR}")


if __name__ == "__main__":
    main()
