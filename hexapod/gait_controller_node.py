#!/usr/bin/env python3
"""
gait_controller_node.py
-----------------------
FSM-based gait controller for M9X6 hexapod AMR.

Geometry (from M9X6.xacro, confirmed 2026-04-19):
  Femur goes UPWARD (+Z) at joint=0° — inverted geometry.
  Stride is applied in world XY frame directly (not leg-local frame)
  to prevent the rotation-instead-of-translation bug.

FSM: IDLE → STAND → TRIPOD / WAVE / RIPPLE → ROTATE → SIT → ESTOP

Subscribe:
  /cmd_vel       [Twist]   — linear.x/y, angular.z
  /imu/data      [Imu]     — roll/pitch body compensation
  /gait/mode     [String]  — TRIPOD / WAVE / RIPPLE
  /estop         [Bool]    — latch emergency stop
  /estop_reset   [Bool]    — clear estop

Publish:
  /hexapod/leg_targets  [PoseArray] — 6 foot XYZ → hexapod_ik_node
  /gait/state           [String]    — current FSM state
  /gait/diagnostics     [String]    — JSON diagnostics
"""

import math
import json
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String, Bool


# ── Robot constants (exact from xacro) ────────────────────────────────────────
COXA_LENGTH  = 0.04823
FEMUR_LENGTH = 0.079924
TIBIA_LENGTH = 0.1465  # must match hexapod_ik_node (real foot-tip collision bottom)

HIP_ORIGINS = np.array([
    [ 0.117445, -0.000015,  0.0306],
    [ 0.058710, -0.101718,  0.0306],
    [-0.058735, -0.101703,  0.0306],
    [-0.117445,  0.000015,  0.0306],
    [-0.058710,  0.101718,  0.0306],
    [ 0.058735,  0.101703,  0.0306],
], dtype=float)

HIP_MOUNT_YAW = np.array([
    math.atan2(-0.000015,  0.117445),
    math.atan2(-0.101718,  0.058710),
    math.atan2(-0.101703, -0.058735),
    math.atan2( 0.000015, -0.117445),
    math.atan2( 0.101718, -0.058710),
    math.atan2( 0.101703,  0.058735),
], dtype=float)

# ── Gait groups ───────────────────────────────────────────────────────────────
TRIPOD_GROUPS  = [[0, 2, 4], [1, 3, 5]]
WAVE_SEQUENCE  = [0, 1, 2, 3, 4, 5]
RIPPLE_GROUPS  = [[0, 3], [1, 4], [2, 5]]

SWING_END  = math.pi
STANCE_END = 2.0 * math.pi

# ── Servo-space stand poses (per joint: coxa, femur, tibia) ────────────────
# The physical robot's calibrated neutral is commanded in SERVO degrees
# directly (bypassing the sim-tuned IK, whose foot targets fold the knee
# past the real tibia range → tucked legs). These match the Pico firmware
# pose tables. Published on /gait/servo_stand for pico_serial_bridge.
STAND_SERVO = [90, 70, 100]   # home — calibrated standing pose (tibia tuned to 100°)
SIT_SERVO   = [90, 70, 160]   # sit — folded, body low

STAND_SERVO_18 = STAND_SERVO * 6   # [90,70,100] x 6 legs
SIT_SERVO_18   = SIT_SERVO   * 6


# ── FSM ───────────────────────────────────────────────────────────────────────
class GaitState(Enum):
    IDLE   = auto()
    STAND  = auto()
    TRIPOD = auto()
    WAVE   = auto()
    RIPPLE = auto()
    ROTATE = auto()
    SIT    = auto()
    ESTOP  = auto()


# ── Foot trajectory ───────────────────────────────────────────────────────────
def compute_foot_target(phase, hip_origin, mount_yaw,
                        vx, vy, wz,
                        stride, lift, clearance,
                        body_roll, body_pitch, comp_gain,
                        reach_mult):
    """
    Compute foot XYZ in base_link frame for one leg at a given phase.

    Swing  [0 → π] : foot lifts and moves forward (direction of travel)
    Stance [π → 2π]: foot on ground, moves backward (pushes body forward)

    Stride direction is in WORLD XY (not leg-local) to prevent rotation bug.
    Rotational contribution adds tangential velocity at each hip pivot.
    """
    ox, oy, oz = hip_origin

    # Neutral foot reach
    r = (COXA_LENGTH + FEMUR_LENGTH * reach_mult)

    # Leg forward/lateral unit vectors
    cx =  math.cos(mount_yaw)
    cy =  math.sin(mount_yaw)

    # World-frame stride velocity for this leg
    # Rotational: tangential velocity at hip pivot = wz × r_hip
    total_vx = vx + (-wz * oy)
    total_vy = vy + ( wz * ox)

    speed = math.hypot(total_vx, total_vy)

    if speed < 1e-4:
        # No motion — hold neutral
        z_comp = comp_gain * (body_pitch * ox - body_roll * oy)
        return np.array([
            ox + r * cx,
            oy + r * cy,
            oz - clearance + z_comp,
        ])

    # Stride direction unit vector in world XY
    sdx = total_vx / speed
    sdy = total_vy / speed

    # Scale stride by speed, capped at stride param
    stride_s = min(stride, stride * speed / 0.05)

    # Phase-based position
    if phase < SWING_END:
        t = phase / SWING_END                  # 0→1
        s = stride_s * (2.0 * t - 1.0)        # −stride→+stride
        z_lift = lift * math.sin(t * math.pi)  # parabolic lift
        z_foot = oz - clearance + z_lift
    else:
        t = (phase - SWING_END) / (STANCE_END - SWING_END)  # 0→1
        s = stride_s * (1.0 - 2.0 * t)        # +stride→−stride
        z_foot = oz - clearance

    # IMU body compensation
    z_comp = comp_gain * (body_pitch * ox - body_roll * oy)

    return np.array([
        ox + r * cx + s * sdx,
        oy + r * cy + s * sdy,
        z_foot + z_comp,
    ])


# ── Node ──────────────────────────────────────────────────────────────────────
class GaitControllerNode(Node):

    def __init__(self):
        super().__init__('gait_controller_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('publish_rate',   50.0)
        self.declare_parameter('gait_freq',       0.5)
        self.declare_parameter('stride_length',   0.030)
        self.declare_parameter('step_height',     0.020)
        self.declare_parameter('body_clearance',  0.04)
        self.declare_parameter('stance_width',    1.0)
        self.declare_parameter('body_comp_gain',  0.4)
        self.declare_parameter('max_vx',          0.10)
        self.declare_parameter('max_vy',          0.06)
        self.declare_parameter('max_wz',          0.80)
        self.declare_parameter('leg_amp',         [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('servo_walk',  True)     # servo-space gait (hardware)
        self.declare_parameter('coxa_swing',  20.0)     # ±deg coxa sweep around stand (step length)
        self.declare_parameter('coxa_sign',   1.0)      # +1 forward / -1 backward (test)
        self.declare_parameter('femur_lift',      0.0)  # deg femur lift during return stroke (step height)
        self.declare_parameter('femur_lift_sign', 1.0)  # +1 = foot rises / -1 = foot digs (flip if wrong)

        self._rate      = float(self.get_parameter('publish_rate').value)
        self._gait_freq = float(self.get_parameter('gait_freq').value)
        self._stride    = float(self.get_parameter('stride_length').value)
        self._lift      = float(self.get_parameter('step_height').value)
        self._clear     = float(self.get_parameter('body_clearance').value)
        self._width     = float(self.get_parameter('stance_width').value)
        self._comp      = float(self.get_parameter('body_comp_gain').value)
        self._max_vx    = float(self.get_parameter('max_vx').value)
        self._max_vy    = float(self.get_parameter('max_vy').value)
        self._max_wz    = float(self.get_parameter('max_wz').value)
        # Per-leg movement amplitude (0.0 = frozen at neutral, 1.0 = full).
        # Leg 1 (index 0) is reduced on the real robot: its three PCA1
        # channels are electrically cross-coupled (damaged driver from the
        # earlier shorted motors), so full stride makes the other two
        # twitch along. The leg still holds neutral stance — it just takes
        # a smaller stride/step when walking.
        self._leg_amp = [float(a) for a in self.get_parameter('leg_amp').value]
        # Servo-space gait: the hardware bridge commands these angles
        # directly — the sim IK cannot reproduce the real robot's stance.
        self._servo_walk = bool(self.get_parameter('servo_walk').value)
        self._coxa_swing = float(self.get_parameter('coxa_swing').value)
        self._coxa_sign  = float(self.get_parameter('coxa_sign').value)
        self._femur_lift       = float(self.get_parameter('femur_lift').value)
        self._femur_lift_sign  = float(self.get_parameter('femur_lift_sign').value)
        self._walk_ramp  = 0.0   # 0..1 amplitude ramp on walking start

        # ── FSM ───────────────────────────────────────────────────────────
        self._state      = GaitState.STAND    # start standing still — no sudden phase jump
        self._gait_mode  = GaitState.TRIPOD
        self._prev_state = GaitState.IDLE

        # ── Velocity ──────────────────────────────────────────────────────
        self._vx      = 0.0   # ramped velocity (used for foot targets)
        self._vx_cmd  = 0.0   # commanded velocity (from /cmd_vel, un-ramped)
        self._vy      = 0.0
        self._wz      = 0.0
        self._last_cmd_t = self.get_clock().now().nanoseconds * 1e-9
        self._tripod_ramp_start = None  # timestamp when TRIPOD started (for vx ramp)

        # ── IMU ───────────────────────────────────────────────────────────
        self._roll  = 0.0
        self._pitch = 0.0

        # ── Leg phases [0, 2π) ────────────────────────────────────────────
        # Tripod: A(1,3,5)=0, B(2,4,6)=π — always anti-phase
        self._phases = np.array(
            [0.0, math.pi, 0.0, math.pi, 0.0, math.pi], dtype=float)

        # Wave / ripple counters
        self._wave_idx   = 0
        self._ripple_idx = 0

        # ── Stand / sit transition ────────────────────────────────────────
        self._stand_prog = 0.0
        self._sit_prog   = 0.0

        # ── Foot targets ──────────────────────────────────────────────────
        self._targets = self._neutral_targets()

        self._last_t = self.get_clock().now().nanoseconds * 1e-9

        # ── Publishers ────────────────────────────────────────────────────
        self._tgt_pub   = self.create_publisher(PoseArray, '/hexapod/leg_targets', 10)
        self._state_pub = self.create_publisher(String,    '/gait/state',          10)
        self._diag_pub  = self.create_publisher(String,    '/gait/diagnostics',    10)
        # Servo-space stand pose (hardware: pico_serial_bridge uses this
        # to hold the calibrated home stance without the sim IK).
        self._stand_pub = self.create_publisher(JointState, '/gait/servo_stand', 10)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Twist,  '/cmd_vel',     self._on_cmd_vel,   10)
        self.create_subscription(Imu,    '/imu/data',    self._on_imu,       10)
        self.create_subscription(String, '/gait/mode',   self._on_mode,      10)
        self.create_subscription(Bool,   '/estop',       self._on_estop,     10)
        self.create_subscription(Bool,   '/estop_reset', self._on_estop_rst, 10)

        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info('gait_controller_node ready')
        self.get_logger().info(f'  Initial state : {self._state.name}')
        self.get_logger().info(f'  Default gait  : {self._gait_mode.name}')
        self.get_logger().info(f'  Rate          : {self._rate} Hz')
        self.get_logger().info(f'  Gait freq     : {self._gait_freq} Hz')
        self.get_logger().info(f'  Stride        : {self._stride} m')
        self.get_logger().info(f'  Step height   : {self._lift} m')
        self.get_logger().info(f'  Body clearance: {self._clear} m')
        self.get_logger().info(f'  Stance width  : {self._width}x')
        self.get_logger().info(f'  Leg amplitude : {self._leg_amp}')
        self.get_logger().info(
            f'  Servo gait    : {self._servo_walk} (coxa swing ±{self._coxa_swing}°, '
            f'sign {self._coxa_sign}, femur lift {self._femur_lift}° '
            f'sign {self._femur_lift_sign})')

    # ── Neutral / sit targets ─────────────────────────────────────────────────

    def _neutral_targets(self):
        t = np.zeros((6, 3), dtype=float)
        r = COXA_LENGTH + FEMUR_LENGTH * 0.9
        for i in range(6):
            yaw = HIP_MOUNT_YAW[i]
            ox, oy, oz = HIP_ORIGINS[i]
            t[i] = [ox + r*math.cos(yaw), oy + r*math.sin(yaw), oz - self._clear]
        return t

    def _sit_targets(self):
        t = np.zeros((6, 3), dtype=float)
        r = COXA_LENGTH + FEMUR_LENGTH * 0.9
        for i in range(6):
            yaw = HIP_MOUNT_YAW[i]
            ox, oy, oz = HIP_ORIGINS[i]
            t[i] = [ox + r*math.cos(yaw), oy + r*math.sin(yaw), oz - self._clear*0.3]
        return t

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        self._vx_cmd = float(np.clip(msg.linear.x,  -self._max_vx, self._max_vx))
        self._vy     = float(np.clip(msg.linear.y,  -self._max_vy, self._max_vy))
        self._wz     = float(np.clip(msg.angular.z, -self._max_wz, self._max_wz))
        self._last_cmd_t = self.get_clock().now().nanoseconds * 1e-9

        if self._state == GaitState.ESTOP:
            return

        moving = abs(self._vx_cmd) > 1e-3 or abs(self._vy) > 1e-3 or abs(self._wz) > 1e-3

        if moving and self._state in (GaitState.IDLE, GaitState.STAND):
            next_state = GaitState.ROTATE if (
                abs(self._wz) > 1e-3 and abs(self._vx_cmd) < 1e-3 and abs(self._vy) < 1e-3
            ) else self._gait_mode
            self._transition(next_state)
        elif not moving and self._state not in (
                GaitState.IDLE, GaitState.STAND, GaitState.SIT):
            self._transition(GaitState.STAND)

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        sinr = 2.0 * (q.w*q.x + q.y*q.z)
        cosr = 1.0 - 2.0*(q.x*q.x + q.y*q.y)
        self._roll  = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0*(q.w*q.y - q.z*q.x)))
        self._pitch = math.asin(sinp)

    def _on_mode(self, msg: String):
        m = {'TRIPOD': GaitState.TRIPOD,
             'WAVE':   GaitState.WAVE,
             'RIPPLE': GaitState.RIPPLE}.get(msg.data.upper())
        if m is None:
            self.get_logger().warn(f'Unknown mode: {msg.data}')
            return
        self._gait_mode = m
        if self._state in (GaitState.TRIPOD, GaitState.WAVE,
                           GaitState.RIPPLE, GaitState.ROTATE):
            self._transition(m)

    def _on_estop(self, msg: Bool):
        if msg.data:
            self.get_logger().error('ESTOP — freezing all joints.')
            self._transition(GaitState.ESTOP)

    def _on_estop_rst(self, msg: Bool):
        if msg.data and self._state == GaitState.ESTOP:
            self.get_logger().warn('ESTOP cleared → STAND')
            self._transition(GaitState.STAND)

    # ── FSM transition ────────────────────────────────────────────────────────

    def _transition(self, new: GaitState):
        if new == self._state:
            return
        self.get_logger().info(f'FSM  {self._state.name} → {new.name}')
        self._prev_state = self._state
        self._state      = new

        if new == GaitState.STAND:
            self._stand_prog = 0.0
        if new == GaitState.SIT:
            self._sit_prog = 0.0
        if new in (GaitState.TRIPOD, GaitState.WAVE,
                   GaitState.RIPPLE, GaitState.ROTATE):
            self._wave_idx   = 0
            self._ripple_idx = 0
            self._reset_phases(new)
            # Start vx ramp from 0 (prevent snap/discontinuity)
            self._tripod_ramp_start = self.get_clock().now().nanoseconds * 1e-9
            self._vx = 0.0

    def _reset_phases(self, mode: GaitState):
        if mode in (GaitState.TRIPOD, GaitState.ROTATE):
            self._phases = np.array(
                [0.0, math.pi, 0.0, math.pi, 0.0, math.pi], dtype=float)
        elif mode == GaitState.WAVE:
            self._phases = np.array(
                [i * (2*math.pi/6) for i in range(6)], dtype=float)
        elif mode == GaitState.RIPPLE:
            self._phases = np.array(
                [0.0, 2.094, 4.189, 0.0, 2.094, 4.189], dtype=float)

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = now - self._last_t
        self._last_t = now

        # Velocity watchdog — stop after 5s silence on /cmd_vel
        if self._state not in (GaitState.IDLE, GaitState.STAND,
                               GaitState.SIT,  GaitState.ESTOP):
            if (now - self._last_cmd_t) > 5.0:
                self._vx_cmd = 0.0
                self._vy     = 0.0
                self._wz     = 0.0
                self._transition(GaitState.STAND)

        # ── VX ramp: smoothly accelerate from 0→target over 1s ──────────
        # Prevents the discontinuous snap when transitioning STAND→TRIPOD
        if self._state in (GaitState.TRIPOD, GaitState.WAVE,
                           GaitState.RIPPLE, GaitState.ROTATE):
            if self._tripod_ramp_start is not None:
                elapsed = now - self._tripod_ramp_start
                ramp = min(1.0, elapsed / 1.0)
                self._vx = self._vx_cmd * ramp
                self._walk_ramp = ramp
                if ramp >= 1.0:
                    self._tripod_ramp_start = None  # ramp complete
            else:
                self._vx = self._vx_cmd
                self._walk_ramp = 1.0
        else:
            self._vx = 0.0
            self._walk_ramp = 0.0

        # Dispatch
        if   self._state == GaitState.IDLE:   self._do_idle()
        elif self._state == GaitState.STAND:  self._do_stand(dt)
        elif self._state == GaitState.TRIPOD: self._do_tripod(dt)
        elif self._state == GaitState.WAVE:   self._do_wave(dt)
        elif self._state == GaitState.RIPPLE: self._do_ripple(dt)
        elif self._state == GaitState.ROTATE: self._do_rotate(dt)
        elif self._state == GaitState.SIT:    self._do_sit(dt)
        # ESTOP: hold last targets, no update

        self._publish_targets()
        self._publish_servo_stand()
        self._publish_state()
        self._publish_diag(now)

    # ── FSM actions ───────────────────────────────────────────────────────────

    def _do_idle(self):
        r = COXA_LENGTH
        for i in range(6):
            yaw = HIP_MOUNT_YAW[i]
            ox, oy, oz = HIP_ORIGINS[i]
            self._targets[i] = [ox + r*math.cos(yaw), oy + r*math.sin(yaw), oz]

    def _do_stand(self, dt):
        self._stand_prog = min(1.0, self._stand_prog + dt / 0.5)
        neutral = self._neutral_targets()
        p = self._stand_prog
        for i in range(6):
            self._targets[i] = (1-p)*self._targets[i] + p*neutral[i]

    def _do_sit(self, dt):
        self._sit_prog = min(1.0, self._sit_prog + dt / 0.8)
        sit = self._sit_targets()
        p = self._sit_prog
        for i in range(6):
            self._targets[i] = (1-p)*self._targets[i] + p*sit[i]

    def _advance_phases(self, dt):
        omega = 2.0 * math.pi * self._gait_freq
        self._phases = (self._phases + omega * dt) % (2.0 * math.pi)

    def _foot(self, i):
        return compute_foot_target(
            self._phases[i], HIP_ORIGINS[i], HIP_MOUNT_YAW[i],
            self._vx, self._vy, self._wz,
            self._stride, self._lift, self._clear,
            self._roll, self._pitch, self._comp,
            0.9,   # reach_mult (must match IK's REACH_MULT)
        )

    def _foot_scaled(self, i):
        """Foot target for leg i with per-leg amplitude applied.

        For legs with amp < 1.0 the target is blended toward that leg's
        neutral stance position, so it still supports the body but walks
        with a smaller stride and lower step height.
        """
        a = max(0.0, self._leg_amp[i])  # clamp — never invert leg motion
        if a >= 1.0:
            return self._foot(i)
        return self._neutral_targets()[i] + a * (self._foot(i) - self._neutral_targets()[i])

    def _do_tripod(self, dt):
        self._advance_phases(dt)
        for i in range(6):
            self._targets[i] = self._foot_scaled(i)

    def _do_wave(self, dt):
        omega = 2.0 * math.pi * self._gait_freq
        delta = omega * dt
        for i in range(6):
            if i == self._wave_idx:
                prev = self._phases[i]
                self._phases[i] = (self._phases[i] + delta) % (2*math.pi)
                if self._phases[i] >= math.pi > prev:
                    self._wave_idx = (self._wave_idx + 1) % 6
            else:
                self._phases[i] = (self._phases[i] + delta/5.0) % (2*math.pi)
                if self._phases[i] < math.pi:
                    self._phases[i] = math.pi
            self._targets[i] = self._foot_scaled(i)

    def _do_ripple(self, dt):
        omega = 2.0 * math.pi * self._gait_freq
        delta = omega * dt
        active = RIPPLE_GROUPS[self._ripple_idx]
        for i in range(6):
            if i in active:
                prev = self._phases[i]
                self._phases[i] = (self._phases[i] + delta) % (2*math.pi)
                if self._phases[i] >= math.pi > prev:
                    self._ripple_idx = (self._ripple_idx + 1) % 3
            else:
                self._phases[i] = (self._phases[i] + delta/2.0) % (2*math.pi)
                if self._phases[i] < math.pi:
                    self._phases[i] = math.pi
            self._targets[i] = self._foot_scaled(i)

    def _do_rotate(self, dt):
        if abs(self._vx) > 1e-3 or abs(self._vy) > 1e-3:
            self._transition(self._gait_mode)
            return
        self._advance_phases(dt)
        for i in range(6):
            self._targets[i] = self._foot_scaled(i)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_targets(self):
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for i in range(6):
            p = Pose()
            p.position.x = float(self._targets[i, 0])
            p.position.y = float(self._targets[i, 1])
            p.position.z = float(self._targets[i, 2])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self._tgt_pub.publish(msg)

    def _publish_servo_stand(self):
        """Publish the servo-space stand pose for pico_serial_bridge.

        STAND / IDLE → calibrated home [90, 70, 100] x 6 (the pose the
        real robot was homed into). SIT → [90, 70, 160] x 6. Walking
        states → the servo-space walk pose (coxa sweep + femur lift).
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._state == GaitState.ESTOP:
            # Freeze: don't publish — the bridge keeps its last cached
            # 18-value pose, so the legs hold exactly where they are.
            # (Publishing walk angles here would snap coxas to 90.)
            return
        if self._state in (GaitState.STAND, GaitState.IDLE):
            msg.position = list(STAND_SERVO_18)
        elif self._state == GaitState.SIT:
            msg.position = list(SIT_SERVO_18)
        elif self._servo_walk:
            msg.position = self._servo_walk_angles()
        else:
            msg.position = []
        self._stand_pub.publish(msg)

    def _servo_walk_angles(self):
        """Servo-space walking pose for pico_serial_bridge.

        Stage 2 (coxa sweep + femur lift): coxa sweeps ±coxa_swing°
        around the stand value with tripod anti-phasing (legs 1,3,5 vs
        2,4,6 via the existing per-leg phases) — that's the step LENGTH.
        Femur lifts during the return stroke so the foot clears the
        ground (step HEIGHT) instead of dragging: the lift window peaks
        at phase 0 (mid return stroke, foot moving forward relative to
        body) and is zero at ±90° (plant / lift-off). Tibia holds the
        calibrated stand pose. All amplitudes ramp in with the velocity
        ramp and scale per-leg by leg_amp (leg 1 moves less — damaged
        driver).
        """
        coxa_base, fem_base, tib = STAND_SERVO
        amp  = self._coxa_swing * self._coxa_sign * self._walk_ramp
        lift = self._femur_lift * self._femur_lift_sign * self._walk_ramp
        out = []
        for i in range(6):
            ph = self._phases[i]
            c = coxa_base + amp * self._leg_amp[i] * math.sin(ph)
            # Smooth lift bump: max(0, cos(phase))² peaks at phase 0
            # (mid return stroke), 0 at phase ±π/2 (plant & lift-off).
            lift_env = max(0.0, math.cos(ph)) ** 2
            f = fem_base + lift * self._leg_amp[i] * lift_env
            out += [float(c), float(f), float(tib)]
        return out

    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)

    def _publish_diag(self, now):
        msg = String()
        msg.data = json.dumps({
            'state'      : self._state.name,
            'gait_mode'  : self._gait_mode.name,
            'vx'         : round(self._vx, 4),
            'vy'         : round(self._vy, 4),
            'wz'         : round(self._wz, 4),
            'roll_deg'   : round(math.degrees(self._roll),  2),
            'pitch_deg'  : round(math.degrees(self._pitch), 2),
            'phases_deg' : [round(math.degrees(p), 1) for p in self._phases],
            'wave_idx'   : self._wave_idx,
            'ripple_idx' : self._ripple_idx,
            'time'       : round(now, 3),
        })
        self._diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('gait_controller_node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
