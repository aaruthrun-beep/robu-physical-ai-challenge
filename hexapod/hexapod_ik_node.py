#!/usr/bin/env python3
"""
hexapod_ik_node.py
------------------
Pure inverse kinematics calculator for M9X6 hexapod.

Geometry confirmed from M9X6.xacro (uploaded 2026-04-19):
  - 6 legs x 3 revolute joints = 18 joints
  - All hip axes: Z (straight up)
  - femur Z offset: +0.079924m (goes UPWARD from coxa — inverted geometry)
  - Coxa XY length: 0.04823m
  - Tibia length: 0.1465m (real foot-tip collision box bottom; was 0.120m — too short by 2.65cm)

  Hip mount yaws (atan2 of hip origin XY):
    Leg1=0°, Leg2=-60°, Leg3=-120°, Leg4=180°, Leg5=120°, Leg6=60°

Subscribe : /hexapod/leg_targets [geometry_msgs/PoseArray] — from gait node
Publish   : /joint_states        [sensor_msgs/JointState]  — to RSP + hardware
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState

# ── Joint names (must match URDF exactly) ─────────────────────────────────────
JOINT_NAMES = [
    "hip_joint1",  "femur_joint1",  "foot_joint1",
    "hip_joint2",  "femur_joint2",  "foot_joint2",
    "hip_joint3",  "femur_joint3",  "foot_joint3",
    "hip_joint4",  "femur_joint4",  "foot_joint4",
    "hip_joint5",  "femur_joint5",  "foot_joint5",
    "hip_joint6",  "femur_joint6",  "foot_joint6",
]

# ── Robot geometry (exact from xacro) ─────────────────────────────────────────
COXA_LENGTH  = 0.04823   # XY distance of femur_joint origin from hip_joint
FEMUR_LENGTH = 0.079924  # Z offset of foot_joint from femur_joint (+Z = UP)
# TIBIA_LENGTH = 0.120   # OLD — estimated, 2.65cm too SHORT → feet plowed
#                          through the ground → robot floated / "climbed"
TIBIA_LENGTH = 0.1465    # real foot contact = foot-tip collision box BOTTOM
#                          (box at z=-0.1365 in tibia frame, sz=0.02 → bottom -0.1465)

# Hip pivot positions in base_link frame [x, y, z]
HIP_ORIGINS = np.array([
    [ 0.117445, -0.000015,  0.0306],   # leg 1  yaw =   0°
    [ 0.058710, -0.101718,  0.0306],   # leg 2  yaw = -60°
    [-0.058735, -0.101703,  0.0306],   # leg 3  yaw = -120°
    [-0.117445,  0.000015,  0.0306],   # leg 4  yaw = 180°
    [-0.058710,  0.101718,  0.0306],   # leg 5  yaw = 120°
    [ 0.058735,  0.101703,  0.0306],   # leg 6  yaw =  60°
], dtype=float)

# Nominal outward yaw of each leg [rad] — atan2(hip_y, hip_x)
HIP_MOUNT_YAW = np.array([
    math.atan2(-0.000015,  0.117445),   # leg 1 ≈  0.000 rad
    math.atan2(-0.101718,  0.058710),   # leg 2 ≈ -1.047 rad
    math.atan2(-0.101703, -0.058735),   # leg 3 ≈ -2.094 rad
    math.atan2( 0.000015, -0.117445),   # leg 4 ≈ ±3.142 rad
    math.atan2( 0.101718, -0.058710),   # leg 5 ≈  2.094 rad
    math.atan2( 0.101703,  0.058735),   # leg 6 ≈  1.047 rad
], dtype=float)

# ── Joint limits from URDF ────────────────────────────────────────────────────
HIP_LIM   = (-1.570796,  1.570796)   # ±90°
FEMUR_LIM = (-2.617994,  0.523599)   # -150° to +30°
FOOT_LIM  = (-0.087266,  3.054326)   #  -5°  to +175°

# ── Neutral stance parameters ─────────────────────────────────────────────────
# Confirmed good values from RViz tuning session
REACH_MULT  = 0.9     # r = COXA + FEMUR * 0.9 — wider stance for stability
CLEARANCE   = 0.10    # foot Z below hip (reduced so feet don't clip through ground)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def solve_leg_ik(foot_x, foot_y, foot_z, hip_origin, hip_mount_yaw):
    """
    Analytical IK for one leg. Returns (q_hip, q_femur, q_foot) in radians.

    Coordinate convention (from xacro):
      - Hip rotates around Z axis
      - Femur pivot is at coxa tip, offset -Z (down) from hip by 0.033988m
      - Femur arm goes +Z (upward) when at 0° — INVERTED geometry
      - foot_joint is 0.079924m above femur pivot at femur=0°

    IK approach:
      1. Hip yaw = atan2(dy, dx) - mount_yaw
      2. Radial reach from hip to foot in XY = R_total
         Effective reach for 2-link chain = R_total - COXA_LENGTH
      3. Vertical = dz (foot Z relative to hip pivot)
      4. Law of cosines for femur + tibia
      5. Negative sign on femur because femur goes UP at positive angle
         (robot's inverted geometry — confirmed during tuning)

    Returns (0, 0, 0) if target is out of workspace.
    """
    dx = foot_x - hip_origin[0]
    dy = foot_y - hip_origin[1]
    dz = foot_z - hip_origin[2]

    # ── 1. Hip yaw ────────────────────────────────────────────────────────
    q_hip = math.atan2(dy, dx) - hip_mount_yaw
    q_hip = (q_hip + math.pi) % (2 * math.pi) - math.pi
    q_hip = _clamp(q_hip, *HIP_LIM)

    # ── 2. Project to sagittal plane ──────────────────────────────────────
    R_eff = math.hypot(dx, dy) - COXA_LENGTH
    Z_eff = dz

    # ── 3. 2-link IK (femur + tibia) ─────────────────────────────────────
    L1, L2 = FEMUR_LENGTH, TIBIA_LENGTH
    D2 = R_eff**2 + Z_eff**2
    D  = math.sqrt(D2)

    if D > (L1 + L2) or D < abs(L1 - L2) or D < 1e-9:
        return (0.0, 0.0, 0.0)

    # Foot (tibia) angle — law of cosines
    cos_foot = _clamp((D2 - L1**2 - L2**2) / (2.0 * L1 * L2), -1.0, 1.0)
    q_foot   = math.acos(cos_foot)

    # Femur angle — negative because femur goes UP at positive joint angle
    alpha   = math.atan2(Z_eff, R_eff)
    cos_b   = _clamp((D2 + L1**2 - L2**2) / (2.0 * D * L1), -1.0, 1.0)
    q_femur = -(alpha + math.acos(cos_b))

    # ── 4. Clamp to limits ────────────────────────────────────────────────
    q_hip   = _clamp(q_hip,   *HIP_LIM)
    q_femur = _clamp(q_femur, *FEMUR_LIM)
    q_foot  = _clamp(q_foot,  *FOOT_LIM)

    return (q_hip, q_femur, q_foot)


class HexapodIKNode(Node):
    """
    Pure IK calculator node.
    Receives foot targets from gait_controller_node → solves 18 joint angles
    → publishes /joint_states for robot_state_publisher and hardware.
    """

    def __init__(self):
        super().__init__('hexapod_ik_node')

        self.declare_parameter('publish_rate', 50.0)
        rate_hz = float(self.get_parameter('publish_rate').value)

        # Start at neutral stance so RViz has a valid pose on boot
        self._foot_targets   = self._neutral_targets()
        self._joint_angles   = [0.0] * 18
        self._targets_recv   = False
        self._solve_all()

        self._js_pub    = self.create_publisher(JointState, '/joint_states',    10)
        self._cmd_pub   = self.create_publisher(JointState, '/joint_commands',  10)  # for hardware bridge
        self._sub       = self.create_subscription(
            PoseArray, '/hexapod/leg_targets', self._on_targets, 10)
        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info('hexapod_ik_node ready (pure IK calculator)')
        self.get_logger().info(f'  publish_rate = {rate_hz} Hz')
        self.get_logger().info(f'  COXA={COXA_LENGTH}m  FEMUR={FEMUR_LENGTH}m  TIBIA={TIBIA_LENGTH}m')
        self.get_logger().info(f'  Neutral reach = COXA + FEMUR*{REACH_MULT} = {COXA_LENGTH + FEMUR_LENGTH*REACH_MULT:.4f}m')

    def _neutral_targets(self):
        """Safe standing position — used on startup before gait node connects."""
        targets = np.zeros((6, 3), dtype=float)
        r = COXA_LENGTH + FEMUR_LENGTH * REACH_MULT
        for i in range(6):
            yaw = HIP_MOUNT_YAW[i]
            ox, oy, oz = HIP_ORIGINS[i]
            targets[i] = [
                ox + r * math.cos(yaw),
                oy + r * math.sin(yaw),
                oz - CLEARANCE,
            ]
        return targets

    def _solve_all(self):
        angles = []
        for i in range(6):
            fx, fy, fz = self._foot_targets[i]
            qh, qf, qt = solve_leg_ik(
                fx, fy, fz, HIP_ORIGINS[i], HIP_MOUNT_YAW[i])
            angles.extend([qh, qf, qt])
        self._joint_angles = angles

    def _on_targets(self, msg: PoseArray):
        if len(msg.poses) != 6:
            self.get_logger().warn(
                f'Expected 6 poses, got {len(msg.poses)}. Ignoring.')
            return
        for i, pose in enumerate(msg.poses):
            self._foot_targets[i, 0] = pose.position.x
            self._foot_targets[i, 1] = pose.position.y
            self._foot_targets[i, 2] = pose.position.z
        if not self._targets_recv:
            self.get_logger().info('First target received from gait_controller_node.')
            self._targets_recv = True
        self._solve_all()

    def _publish(self):
        msg = JointState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name            = JOINT_NAMES
        msg.position        = self._joint_angles
        msg.velocity        = [0.0] * 18
        msg.effort          = [0.0] * 18
        self._js_pub.publish(msg)
        self._cmd_pub.publish(msg)    # same message → hardware bridge


def main(args=None):
    rclpy.init(args=args)
    node = HexapodIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('hexapod_ik_node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
