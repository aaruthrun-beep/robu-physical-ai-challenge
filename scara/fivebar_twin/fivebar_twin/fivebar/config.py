"""
Robot configuration — all tunable parameters in one place.
Edit these to match your physical robot, or load from a JSON file.
"""
from dataclasses import dataclass, field, asdict
from typing import List, Optional
import json, math


@dataclass
class Obstacle:
    """A static workcell obstacle, modelled as a vertical cylinder (top-down circle)."""
    x: float
    y: float
    radius: float
    z_lo: float = -50.0
    z_hi: float = 10.0
    name: str = "obstacle"


@dataclass
class RobotConfig:
    # ---- link lengths (mm, pivot-to-pivot) --------------------------------
    L1a: float = 120.0     # left  proximal (upper) link
    L1b: float = 120.0     # right proximal (upper) link
    L2a: float = 200.0     # left  distal  (lower) link
    L2b: float = 200.0     # right distal  (lower) link
    d:   float = 120.0     # base motor separation (shaft-to-shaft, along X)

    # ---- out-of-plane geometry (mm) --------------------------------------
    dh:  float = 20.0      # vertical offset between the two proximal-link planes
                           # (left plane z=0, right plane z=dh). This is why the
                           # arm can cross itself; collision uses the real z.

    # ---- physical radii for collision (mm) -------------------------------
    link_radius: float = 9.5     # half-thickness of every link (capsule radius)
    hub_radius:  float = 39.25   # RS01 body is 78.5 mm square -> ~39 mm radius
    hub_height:  float = 40.0    # motor body height below its link plane
    ee_radius:   float = 12.0    # end-effector joint hardware radius

    # ---- safety / limits --------------------------------------------------
    margin: float = 4.0                      # collision safety margin (mm)
    theta_min: Optional[float] = None        # extra hard-stop limits (rad), or None
    theta_max: Optional[float] = None        # for full 0..360 leave as None

    # ---- motion limits (for the planner / animation) ---------------------
    max_vel: float = 3.0        # rad/s per motor
    max_acc: float = 8.0        # rad/s^2 per motor
    path_check_steps: int = 40  # collision samples along a joint-space move

    # ---- static obstacles -------------------------------------------------
    obstacles: List[Obstacle] = field(default_factory=list)

    # ---- CAN & RobStride Motor Parameters ---------------------------------
    motor_model:   str = "rs-01"   # RobStride motor model: rs-01, rs-02, rs-03, etc.
    run_mode:      int = 1         # 1 = PP (Profile Position, recommended for discrete moves), 5 = CSP (Cyclic Synchronous Position)
    can_interface: str = "slcan"   # python-can interface: slcan/pcan/kvaser/socketcan...
    can_channel:   str = "COM18"    # Windows: COM port for CANable/slcan; Linux: can0
    can_bitrate:   int = 1000000
    can_id_left:   int = 1
    can_id_right:  int = 2

    # ---- convenience ------------------------------------------------------
    def bases(self):
        """Motor pivot XY positions (both on the base axis, y=0)."""
        return (-self.d / 2.0, 0.0), (self.d / 2.0, 0.0)

    def save(self, path):
        data = asdict(self)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        obs = [Obstacle(**o) for o in data.pop("obstacles", [])]
        cfg = cls(**data)
        cfg.obstacles = obs
        return cfg


def default_config() -> RobotConfig:
    """The locked geometry from the build, with two example obstacles."""
    cfg = RobotConfig()
    # example fixtures (edit / clear to match your workcell)
    cfg.obstacles = [
        Obstacle(x=0.0,  y=-95.0, radius=20.0, name="rear_post"),
    ]
    return cfg
