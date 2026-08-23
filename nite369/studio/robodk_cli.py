#!/usr/bin/env python3
"""RoboDK CLI helper — runs under RoboDK's embedded Python.

The studio (system Python) cannot import robodk because RoboDK's embedded
site-packages bundles an old `enum` package that breaks Python 3.10's stdlib
enum. Instead, this script runs under RoboDK's own Python and exposes a tiny
JSON line protocol on stdin/stdout:

    {"op": "fk", "joints": [deg x6]}   -> {"ok": true, "pose": [[4x4]]}
    {"op": "ik", "pose": [[4x4]], "initial": [deg x6]} -> {"ok": true, "joints": [..]}
    {"op": "move", "joints": [deg x6]} -> {"ok": true}
    {"op": "get_joints"}               -> {"ok": true, "joints": [..]}
    {"op": "ping"}                     -> {"ok": true}

Usage:
    python robodk_cli.py [port]
"""

import sys
import json

try:
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
except Exception as e:
    print(json.dumps({"ok": False, "error": f"robodk import failed: {e}"}))
    sys.exit(1)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else None
    rdk = None
    if port:
        try:
            rdk = Robolink(port=port)
        except Exception:
            rdk = None
    if rdk is None:
        rdk = Robolink()

    robot = None
    for item in rdk.ItemList():
        if item.Type() == ITEM_TYPE_ROBOT:
            robot = item
            break

    def pose_to_list(pose):
        # RoboDK Mat is column-major; the studio wants row-major (position
        # in the last column). np.asarray(Mat) yields column-major, so
        # transpose to row-major.
        try:
            import numpy as np
            return np.asarray(pose, dtype=float).T.tolist()
        except Exception:
            pass
        if hasattr(pose, "tolist"):
            mat = pose.tolist()
        else:
            mat = pose
        # Some versions return a flat 16-list; reshape to 4x4 then transpose.
        if len(mat) == 16 and all(isinstance(v, (int, float)) for v in mat):
            mat = [list(mat[i * 4:(i + 1) * 4]) for i in range(4)]
        return [list(row) for row in zip(*mat)]

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except Exception as e:
            print(json.dumps({"ok": False, "error": str(e)}))
            continue

        op = req.get("op")
        try:
            if op == "ping":
                print(json.dumps({"ok": True, "connected": rdk is not None,
                                  "robot": robot.Name() if robot else None}))
            elif op == "fk":
                if robot is None:
                    print(json.dumps({"ok": False, "error": "no robot"}))
                    continue
                pose = robot.SolveFK([float(j) for j in req["joints"]])
                print(json.dumps({"ok": True, "pose": pose_to_list(pose)}))
            elif op == "ik":
                if robot is None:
                    print(json.dumps({"ok": False, "error": "no robot"}))
                    continue
                pose = req["pose"]
                # The studio sends a row-major 4x4; fromNumpy reads it in
                # directly (no transpose — Mat stores values in column order
                # internally but fromNumpy handles the row-major input).
                import numpy as np
                from robodk import robomath
                mat = robomath.Mat.fromNumpy(np.asarray(pose, dtype=float))
                guess = req.get("initial") or [0.0] * 6
                sol = robot.SolveIK(mat, [float(g) for g in guess])
                if sol is None:
                    print(json.dumps({"ok": False, "error": "no solution"}))
                else:
                    vals = sol.list() if hasattr(sol, "list") else sol
                    print(json.dumps({"ok": True, "joints": [float(v) for v in vals[:6]]}))
            elif op == "move":
                if robot is None:
                    print(json.dumps({"ok": False, "error": "no robot"}))
                    continue
                robot.setJoints([float(j) for j in req["joints"]])
                print(json.dumps({"ok": True}))
            elif op == "get_joints":
                if robot is None:
                    print(json.dumps({"ok": False, "error": "no robot"}))
                    continue
                j = robot.Joints()
                vals = j.list() if hasattr(j, "list") else j
                print(json.dumps({"ok": True, "joints": [float(v) for v in vals[:6]]}))
            elif op == "get_dh":
                # Dump the robot's link/DH model. Try multiple RoboDK APIs.
                if robot is None:
                    print(json.dumps({"ok": False, "error": "no robot"}))
                    continue
                try:
                    links = robot.Links()
                    table = []
                    for L in links:
                        entry = {}
                        for attr in ("a", "alpha", "d", "theta", "offset",
                                     "type", "joint", "q", "name", "frame"):
                            try:
                                v = getattr(L, attr)
                                if callable(v):
                                    v = v()
                                entry[attr] = v
                            except Exception:
                                entry[attr] = None
                        table.append(entry)
                    if not table:
                        # Fallback: probe Link(i) by index
                        for i in range(10):
                            try:
                                L = robot.Link(i)
                            except Exception:
                                break
                            if L is None:
                                continue
                            entry = {}
                            for attr in ("a", "alpha", "d", "theta", "offset",
                                         "type", "joint", "q"):
                                try:
                                    v = getattr(L, attr)
                                    if callable(v):
                                        v = v()
                                    entry[attr] = v
                                except Exception:
                                    entry[attr] = None
                            table.append(entry)
                    print(json.dumps({"ok": True, "robot": robot.Name(),
                                      "dh": table}))
                except Exception as e:
                    print(json.dumps({"ok": False, "error": f"get_dh failed: {e}"}))
                except Exception as e:
                    print(json.dumps({"ok": False, "error": f"get_dh failed: {e}"}))
            else:
                print(json.dumps({"ok": False, "error": f"unknown op {op}"}))
        except Exception as e:
            print(json.dumps({"ok": False, "error": str(e)}))


if __name__ == "__main__":
    main()
