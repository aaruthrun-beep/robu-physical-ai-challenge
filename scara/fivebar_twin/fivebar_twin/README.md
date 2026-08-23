# 5-Bar Parallel Robot — Digital Twin (Windows desktop app)

A local desktop app that simulates a planar **5-bar parallel-linkage robot**, checks
every move for **collision** before accepting it, and can drive the **real robot** over
a **USB-to-CAN** adapter while mirroring live encoder feedback in the twin.

The collision checker is the single source of truth: the *same* check guards both the
simulator and real hardware. No command is ever sent (to sim or robot) without passing it.

## What's in the box

```
fivebar_twin/
├─ app.py                 # the desktop app (Tkinter) — run this
├─ make_workspace_map.py  # renders workspace_map.png for the current geometry
├─ run.bat                # double-click launcher (Windows)
├─ requirements.txt
├─ fivebar/
│  ├─ config.py           # ALL robot parameters (edit these to match your robot)
│  ├─ kinematics.py       # forward + inverse kinematics, both elbow branches
│  ├─ collision.py        # 3D capsule collision checker (links, hubs, obstacles)
│  ├─ workspace.py        # safe / forbidden / unreachable classifier + PNG map
│  ├─ planner.py          # joint-space path with CONTINUOUS collision checking
│  ├─ command.py          # unified validate-then-send command manager
│  └─ backends.py         # SimBackend + CanBackend + CAN message format
└─ tests/                 # unit tests (kinematics both branches, collision, workspace)
```

## Requirements

- **Python 3.10+** (from python.org — make sure "Add Python to PATH" is ticked).
- Tkinter is included with the standard Windows Python, so the app runs with no extra install.
- Optional: `pip install matplotlib` (for the workspace PNG) and `pip install python-can` (for real hardware).

## Run — simulation first

```
python app.py
```
or double-click **run.bat**.

- **Click anywhere** in the view to set a target; the twin animates to it.
- **Green** = safe reachable, **red** = forbidden (IK solves but the pose collides),
  **dark** = unreachable.
- The **MOVE / SEND** button is *disabled* whenever the target is in collision or
  unreachable — you cannot send a bad command. The reason appears in the rejection log.
- **Jog** each motor by a step; a step that would collide is refused at the last safe angle.
- Left panel edits geometry live; **Apply geometry** recomputes the workspace.

To regenerate the workspace picture after changing geometry:
```
python make_workspace_map.py
```

## Switch to live hardware (phase 2)

1. Wire your USB-CAN adapter and note the interface/port.
2. In `fivebar/config.py` set `can_interface`, `can_channel`, `can_bitrate`, and the
   two motor CAN IDs. On Windows a CANable/slcan adapter is typically
   `can_interface="slcan", can_channel="COM3"`; PCAN is `"pcan","PCAN_USBBUS1"`.
   (On Linux use `"socketcan","can0"`.)
3. Adjust the **CAN message format** in `backends.py` (`pack_command`/`unpack_feedback`)
   to match your motor drivers — it's documented at the top of the file.
4. In the app, switch **Mode → Live (CAN)**. The first live command asks for confirmation.
   The twin then shows *actual* encoder feedback, so you can see following error/stalls.

The collision checker runs before every live command too — it is never bypassed.

## Tests

```
python -m pytest -q
```
Covers IK/FK round-trip on both elbow branches, safe/near/colliding collision poses,
and workspace classification.

## Parameters you should confirm for YOUR robot (in config.py)

- Link lengths `L1a,L1b,L2a,L2b`, base separation `d`, vertical offset `dh`.
- Physical radii: `link_radius`, `hub_radius`, `hub_height`, `ee_radius`.
- `margin` (mm) — collision safety clearance.
- `max_vel`, `max_acc` — real motor limits.
- CAN bitrate, IDs, and the payload format in `backends.py`.
- `theta_min/theta_max` — only if you have hard-stops beyond the collision limits
  (leave `None` for full 0–360°; the real constraint is the computed forbidden zones).
