"""Path Planning — trajectory generation, collision avoidance, waypoint interpolation.

Provides:
  - Joint-space trajectory generation (cubic, quintic, trapezoidal)
  - Cartesian-space linear trajectory with orientation interpolation
  - Waypoint management and path interpolation
  - Collision detection (sphere, capsule, bounding box)
  - Collision-free path planning via waypoint insertion

All angles in radians, positions in meters.
"""

import numpy as np
import math
from enum import Enum


class TrajectoryType(Enum):
    """Types of trajectory generation."""
    JOINT_PTP = "joint_ptp"           # Joint-space point-to-point (cubic)
    JOINT_CUBIC = "joint_cubic"       # Joint-space cubic polynomial
    JOINT_QUINTIC = "joint_quintic"   # Joint-space quintic polynomial
    JOINT_TRAPEZOID = "joint_trap"    # Joint-space trapezoidal velocity
    CARTESIAN_LIN = "cartesian_lin"   # Cartesian linear with orientation Slerp
    CARTESIAN_CIRCULAR = "cartesian_circ"  # Cartesian circular arc


class Waypoint:
    """A single waypoint with joint angles and optional Cartesian pose."""

    def __init__(self, joint_angles=None, position=None, orientation=None,
                 speed=50.0, name=""):
        self.joint_angles = list(joint_angles) if joint_angles else []
        self.position = list(position) if position else None       # [x, y, z]
        self.orientation = list(orientation) if orientation else None  # quaternion [x, y, z, w]
        self.speed = speed          # 1-100%
        self.name = name

    def to_dict(self):
        return {
            "joint_angles": list(self.joint_angles) if self.joint_angles else [],
            "position": list(self.position) if self.position else None,
            "orientation": list(self.orientation) if self.orientation else None,
            "speed": self.speed,
            "name": self.name,
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            joint_angles=data.get("joint_angles", []),
            position=data.get("position"),
            orientation=data.get("orientation"),
            speed=data.get("speed", 50.0),
            name=data.get("name", ""),
        )


class TrajectoryPoint:
    """A single point along a generated trajectory."""

    def __init__(self, time, positions, velocities=None, accelerations=None,
                 position_cartesian=None, orientation_cartesian=None):
        self.time = time
        self.positions = np.array(positions, dtype=float)           # joint angles
        self.velocities = np.array(velocities, dtype=float) if velocities is not None else None
        self.accelerations = np.array(accelerations, dtype=float) if accelerations is not None else None
        self.position_cartesian = position_cartesian                # [x, y, z] or None
        self.orientation_cartesian = orientation_cartesian          # quat or None

    def to_dict(self):
        return {
            "time": self.time,
            "positions": list(self.positions),
            "velocities": list(self.velocities) if self.velocities is not None else None,
            "accelerations": list(self.accelerations) if self.accelerations is not None else None,
        }


class Trajectory:
    """A complete time-indexed trajectory."""

    def __init__(self, traj_type=TrajectoryType.JOINT_PTP):
        self.traj_type = traj_type
        self.points = []            # list of TrajectoryPoint
        self.total_time = 0.0
        self.num_joints = 0
        self.num_points = 0
        self._joint_names = []

    def add_point(self, point):
        self.points.append(point)
        self.num_points = len(self.points)
        if self.num_joints == 0 and point.positions is not None:
            self.num_joints = len(point.positions)

    def get_position_matrix(self):
        """Get joint positions as (num_points x num_joints) array."""
        if not self.points:
            return np.zeros((0, 0))
        return np.array([p.positions for p in self.points])

    def get_time_array(self):
        """Get time values as 1D array."""
        return np.array([p.time for p in self.points])

    def get_velocity_matrix(self):
        """Get velocities as (num_points x num_joints) array or None."""
        if not self.points or self.points[0].velocities is None:
            return None
        return np.array([p.velocities for p in self.points])

    def sample_at(self, time, degrees=False):
        """Sample trajectory at a given time via linear interpolation.
        
        Returns: interpolated joint angles as list.
        """
        if not self.points:
            return []
        times = self.get_time_array()
        if time <= times[0]:
            angles = self.points[0].positions
        elif time >= times[-1]:
            angles = self.points[-1].positions
        else:
            # Linear interpolation between bracketing points
            idx = np.searchsorted(times, time) - 1
            idx = max(0, min(idx, len(self.points) - 2))
            t0, t1 = times[idx], times[idx + 1]
            if t1 - t0 < 1e-10:
                angles = self.points[idx].positions
            else:
                frac = (time - t0) / (t1 - t0)
                q0 = self.points[idx].positions
                q1 = self.points[idx + 1].positions
                angles = q0 + frac * (q1 - q0)
        if degrees:
            return list(np.degrees(angles))
        return list(angles)

    def to_dict(self):
        return {
            "traj_type": self.traj_type.value,
            "total_time": self.total_time,
            "num_joints": self.num_joints,
            "points": [p.to_dict() for p in self.points],
        }


# ═══════════════════════════════════════════════════════════════════════
# Trajectory Generators
# ═══════════════════════════════════════════════════════════════════════

def generate_joint_cubic(start_q, end_q, duration, num_points=100):
    """Generate a joint-space cubic polynomial trajectory.
    
    q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    with zero velocity at start and end.
    
    Args:
        start_q: starting joint angles (list)
        end_q: ending joint angles (list)
        duration: total time in seconds
        num_points: number of trajectory points
    
    Returns:
        Trajectory with positions, velocities, accelerations
    """
    start_q = np.array(start_q, dtype=float)
    end_q = np.array(end_q, dtype=float)
    n_joints = len(start_q)
    
    # Cubic coefficients for zero-velocity boundary conditions
    a0 = start_q
    a1 = np.zeros(n_joints)
    a2 = (3 / duration**2) * (end_q - start_q)
    a3 = (-2 / duration**3) * (end_q - start_q)
    
    times = np.linspace(0, duration, num_points)
    dt = duration / (num_points - 1) if num_points > 1 else duration
    
    traj = Trajectory(TrajectoryType.JOINT_CUBIC)
    traj.num_joints = n_joints
    traj.total_time = duration
    
    for t in times:
        positions = a0 + a1 * t + a2 * t**2 + a3 * t**3
        velocities = a1 + 2 * a2 * t + 3 * a3 * t**2
        accelerations = 2 * a2 + 6 * a3 * t
        traj.add_point(TrajectoryPoint(
            time=t,
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
        ))
    
    return traj


def generate_joint_quintic(start_q, end_q, duration, num_points=100,
                            start_vel=None, end_vel=None):
    """Generate a joint-space quintic polynomial trajectory.
    
    q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    with zero velocity and acceleration at start and end.
    
    Args:
        start_q: starting joint angles (list)
        end_q: ending joint angles (list)
        duration: total time in seconds
        num_points: number of trajectory points
        start_vel: optional starting velocities (None = zeros)
        end_vel: optional ending velocities (None = zeros)
    
    Returns:
        Trajectory
    """
    start_q = np.array(start_q, dtype=float)
    end_q = np.array(end_q, dtype=float)
    n_joints = len(start_q)
    
    v0 = np.zeros(n_joints) if start_vel is None else np.array(start_vel)
    vf = np.zeros(n_joints) if end_vel is None else np.array(end_vel)
    a0_acc = np.zeros(n_joints)  # zero accel at start
    af = np.zeros(n_joints)      # zero accel at end
    
    T = duration
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    
    # Quintic coefficients
    q0 = start_q
    q1 = v0
    q2 = 0.5 * a0_acc
    
    q3 = (20 * end_q - 20 * start_q - (8 * vf + 12 * v0) * T - (3 * af - a0_acc) * T2) / (2 * T3)
    q4 = (30 * start_q - 30 * end_q + (14 * vf + 16 * v0) * T + (3 * af - 2 * a0_acc) * T2) / (2 * T4)
    q5 = (12 * end_q - 12 * start_q - (6 * vf + 6 * v0) * T - (af - a0_acc) * T2) / (2 * T5)
    
    times = np.linspace(0, duration, num_points)
    
    traj = Trajectory(TrajectoryType.JOINT_QUINTIC)
    traj.num_joints = n_joints
    traj.total_time = duration
    
    for t in times:
        positions = q0 + q1 * t + q2 * t**2 + q3 * t**3 + q4 * t**4 + q5 * t**5
        velocities = q1 + 2 * q2 * t + 3 * q3 * t**2 + 4 * q4 * t**3 + 5 * q5 * t**4
        accelerations = 2 * q2 + 6 * q3 * t + 12 * q4 * t**2 + 20 * q5 * t**3
        traj.add_point(TrajectoryPoint(
            time=t,
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
        ))
    
    return traj


def generate_joint_trapezoidal(start_q, end_q, duration, num_points=100,
                                max_velocity=None, acceleration_time=0.2):
    """Generate a joint-space trapezoidal velocity profile trajectory.
    
    Three phases: constant acceleration, constant velocity, constant deceleration.
    
    Args:
        start_q: starting joint angles (list)
        end_q: ending joint angles (list)
        duration: total time in seconds
        num_points: number of trajectory points
        max_velocity: optional max velocity (computed if None)
        acceleration_time: fraction of duration for accel/decel phases
    
    Returns:
        Trajectory
    """
    start_q = np.array(start_q, dtype=float)
    end_q = np.array(end_q, dtype=float)
    n_joints = len(start_q)
    
    T = duration
    Ta = T * min(acceleration_time, 0.49)  # acceleration phase
    Tv = T - 2 * Ta  # constant velocity phase
    Dq = end_q - start_q
    
    # Compute velocities for each joint
    if max_velocity is not None:
        v_max = np.full(n_joints, max_velocity)
    else:
        v_max = Dq / (Ta + Tv + Ta)  # average velocity
        # Scale down if needed
        v_required = Dq / (Ta + Tv + Ta)
        v_max = np.maximum(np.abs(v_required), 0.01) * np.sign(Dq)
    
    times = np.linspace(0, T, num_points)
    
    traj = Trajectory(TrajectoryType.JOINT_TRAPEZOID)
    traj.num_joints = n_joints
    traj.total_time = T
    
    for t in times:
        if t <= Ta:
            # Acceleration phase
            frac = t / Ta
            positions = start_q + 0.5 * v_max * Ta * frac**2
            velocities = v_max * frac
            accelerations = v_max / Ta
        elif t <= T - Ta:
            # Constant velocity phase
            tc = t - Ta
            positions = start_q + v_max * (Ta / 2 + tc)
            velocities = v_max
            accelerations = np.zeros(n_joints)
        else:
            # Deceleration phase
            td = t - (T - Ta)
            positions = end_q - 0.5 * v_max * Ta * (1 - td / Ta)**2
            velocities = v_max * (1 - td / Ta)
            accelerations = -v_max / Ta
        
        traj.add_point(TrajectoryPoint(
            time=t,
            positions=positions,
            velocities=velocities,
            accelerations=accelerations,
        ))
    
    return traj


# ═══════════════════════════════════════════════════════════════════════
# Multi-Waypoint Trajectory
# ═══════════════════════════════════════════════════════════════════════

def generate_multi_waypoint_trajectory(waypoints, duration_per_segment=None,
                                        num_points_per_segment=50,
                                        traj_type=TrajectoryType.JOINT_CUBIC):
    """Generate a trajectory through multiple waypoints.
    
    Args:
        waypoints: list of Waypoint objects
        duration_per_segment: time per segment, or None for equal division
        num_points_per_segment: trajectory points per segment
        traj_type: trajectory type for each segment
    
    Returns:
        Trajectory concatenated from all segments
    """
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints")
    
    n_joints = len(waypoints[0].joint_angles)
    num_segments = len(waypoints) - 1
    
    if duration_per_segment is None:
        duration_per_segment = 2.0
    
    total_duration = duration_per_segment * num_segments
    
    traj = Trajectory(traj_type)
    traj.num_joints = n_joints
    traj.total_time = total_duration
    
    current_time = 0.0
    
    for i in range(num_segments):
        start = waypoints[i]
        end = waypoints[i + 1]
        seg_duration = duration_per_segment
        
        # Generate segment trajectory
        if traj_type in (TrajectoryType.JOINT_CUBIC, TrajectoryType.JOINT_PTP):
            seg_traj = generate_joint_cubic(
                start.joint_angles, end.joint_angles,
                seg_duration, num_points_per_segment
            )
        elif traj_type == TrajectoryType.JOINT_QUINTIC:
            seg_traj = generate_joint_quintic(
                start.joint_angles, end.joint_angles,
                seg_duration, num_points_per_segment
            )
        elif traj_type == TrajectoryType.JOINT_TRAPEZOID:
            seg_traj = generate_joint_trapezoidal(
                start.joint_angles, end.joint_angles,
                seg_duration, num_points_per_segment
            )
        else:
            seg_traj = generate_joint_cubic(
                start.joint_angles, end.joint_angles,
                seg_duration, num_points_per_segment
            )
        
        # Append points, offsetting time
        for pt in seg_traj.points:
            traj.add_point(TrajectoryPoint(
                time=pt.time + current_time,
                positions=pt.positions,
                velocities=pt.velocities,
                accelerations=pt.accelerations,
            ))
        
        current_time += seg_duration
    
    return traj


# ═══════════════════════════════════════════════════════════════════════
# Cartesian Trajectory Generation
# ═══════════════════════════════════════════════════════════════════════

def slerp(q1, q2, t):
    """Spherical linear interpolation between two quaternions.
    
    Args:
        q1, q2: quaternions as [x, y, z, w]
        t: interpolation parameter (0 to 1)
    
    Returns:
        Interpolated quaternion [x, y, z, w]
    """
    q1 = np.array(q1)
    q2 = np.array(q2)
    
    dot = np.dot(q1, q2)
    
    # If the dot product is negative, negate one quaternion
    if dot < 0:
        q2 = -q2
        dot = -dot
    
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Linear interpolation for small angles
        result = q1 + t * (q2 - q1)
        result = result / np.linalg.norm(result)
        return result
    
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = sin_theta / sin_theta_0
    
    result = s0 * q1 + s1 * q2
    return result / np.linalg.norm(result)


def generate_cartesian_linear(start_pos, end_pos, duration,
                               start_orient=None, end_orient=None,
                               num_points=100):
    """Generate a Cartesian linear trajectory with orientation interpolation.
    
    Args:
        start_pos: starting [x, y, z]
        end_pos: ending [x, y, z]
        duration: total time in seconds
        start_orient: starting quaternion [x, y, z, w]
        end_orient: ending quaternion [x, y, z, w]
        num_points: number of trajectory points
    
    Returns:
        Trajectory with position_cartesian set for each point
    """
    start_pos = np.array(start_pos, dtype=float)
    end_pos = np.array(end_pos, dtype=float)
    
    times = np.linspace(0, duration, num_points)
    
    traj = Trajectory(TrajectoryType.CARTESIAN_LIN)
    traj.total_time = duration
    
    for t in times:
        frac = t / duration
        # Linear position interpolation
        pos = start_pos + frac * (end_pos - start_pos)
        
        # Orientation (Slerp)
        orient = None
        if start_orient is not None and end_orient is not None:
            orient = slerp(start_orient, end_orient, frac)
        
        # Use cubic velocity profile for smooth motion along the line
        v_frac = 3 * frac**2 - 2 * frac**3  # smooth step
        # Map back to joint positions (requires IK - for now just store cartesian)
        joint_pos = list(pos)  # placeholder - will be converted by IK
        
        traj.add_point(TrajectoryPoint(
            time=t,
            positions=joint_pos,
            position_cartesian=list(pos),
            orientation_cartesian=list(orient) if orient is not None else None,
        ))
    
    return traj


# ═══════════════════════════════════════════════════════════════════════
# Collision Avoidance
# ═══════════════════════════════════════════════════════════════════════

class CollisionSphere:
    """A sphere used for collision detection."""

    def __init__(self, center, radius, name=""):
        self.center = np.array(center, dtype=float)
        self.radius = radius
        self.name = name

    def distance_to(self, other):
        """Distance between this sphere and another sphere."""
        d = np.linalg.norm(self.center - other.center)
        return d - self.radius - other.radius

    def collides_with(self, other):
        """Check if this sphere collides with another sphere."""
        return self.distance_to(other) < 0


class CollisionCapsule:
    """A capsule (cylinder with hemispherical ends) for collision detection."""

    def __init__(self, point_a, point_b, radius, name=""):
        self.point_a = np.array(point_a, dtype=float)
        self.point_b = np.array(point_b, dtype=float)
        self.radius = radius
        self.name = name

    def closest_point_to(self, point):
        """Find closest point on capsule axis to given point."""
        p = np.array(point, dtype=float)
        ab = self.point_b - self.point_a
        ab_len_sq = np.dot(ab, ab)
        if ab_len_sq < 1e-10:
            return self.point_a.copy()
        t = np.clip(np.dot(p - self.point_a, ab) / ab_len_sq, 0, 1)
        return self.point_a + t * ab

    def distance_to_sphere(self, sphere):
        """Distance between this capsule and a sphere."""
        closest = self.closest_point_to(sphere.center)
        d = np.linalg.norm(closest - sphere.center)
        return d - self.radius - sphere.radius

    def collides_with_sphere(self, sphere):
        return self.distance_to_sphere(sphere) < 0


class CollisionScene:
    """Manages collision objects and provides collision checking along a trajectory."""

    def __init__(self):
        self.obstacles = []          # list of CollisionSphere (static obstacles)
        self.robot_spheres = []      # list of CollisionSphere (robot body parts)
        self._joint_positions_history = []

    def add_obstacle(self, center, radius, name=""):
        """Add a static spherical obstacle."""
        self.obstacles.append(CollisionSphere(center, radius, name))

    def clear_obstacles(self):
        self.obstacles = []

    def clear_robot_spheres(self):
        self.robot_spheres = []

    def update_robot_spheres(self, joint_angles, dh_arm=None):
        """Update robot collision spheres based on current joint angles.
        
        If dh_arm is provided, uses FK to position spheres along the arm.
        Otherwise, stores joint angles for later evaluation.
        """
        if dh_arm is not None:
            transforms = dh_arm.forward_all(joint_angles)
            self.robot_spheres = []
            # Place a sphere at each joint and along links
            for i, T in enumerate(transforms):
                pos = T[:3, 3]
                self.robot_spheres.append(CollisionSphere(
                    pos, 0.05, f"joint_{i}"
                ))
                # Mid-point sphere for longer links
                if i > 0:
                    prev_pos = transforms[i - 1][:3, 3]
                    mid = (pos + prev_pos) / 2
                    self.robot_spheres.append(CollisionSphere(
                        mid, 0.04, f"link_{i}"
                    ))
        else:
            self._joint_positions_history.append(list(joint_angles))

    def check_collisions(self):
        """Check all robot spheres against all obstacles.
        
        Returns:
            List of (robot_sphere, obstacle) collision pairs, or empty if clear.
        """
        collisions = []
        for r_idx, r_sphere in enumerate(self.robot_spheres):
            for o_idx, obstacle in enumerate(self.obstacles):
                if r_sphere.collides_with(obstacle):
                    collisions.append({
                        "robot_sphere": r_sphere,
                        "robot_idx": r_idx,
                        "obstacle": obstacle,
                        "obstacle_idx": o_idx,
                        "distance": r_sphere.distance_to(obstacle),
                    })
        return collisions

    def is_safe(self):
        """Returns True if no collisions detected."""
        return len(self.check_collisions()) == 0

    def check_trajectory_safety(self, trajectory, dh_arm=None, stride=5):
        """Check an entire trajectory for collisions.
        
        Args:
            trajectory: Trajectory object
            dh_arm: DHArm for computing FK
            stride: check every Nth point
        
        Returns:
            List of (time, collision_pairs) for colliding points, or [] if safe.
        """
        collisions_at_times = []
        for i, pt in enumerate(trajectory.points):
            if i % stride != 0:
                continue
            if dh_arm is not None:
                self.update_robot_spheres(pt.positions, dh_arm)
            else:
                self.update_robot_spheres(pt.positions)
            col = self.check_collisions()
            if col:
                collisions_at_times.append((pt.time, col))
        return collisions_at_times


# ═══════════════════════════════════════════════════════════════════════
# Path Utilities
# ═══════════════════════════════════════════════════════════════════════

def compute_path_length(trajectory):
    """Compute total Cartesian path length of a trajectory.
    
    Works for both joint-space (approximate via FK) and Cartesian trajectories.
    """
    if not trajectory.points:
        return 0.0
    
    # If Cartesian positions available, use them
    if trajectory.points[0].position_cartesian is not None:
        total = 0.0
        for i in range(1, len(trajectory.points)):
            p0 = np.array(trajectory.points[i - 1].position_cartesian)
            p1 = np.array(trajectory.points[i].position_cartesian)
            total += np.linalg.norm(p1 - p0)
        return total
    
    # Otherwise estimate from joint-space
    total = 0.0
    for i in range(1, len(trajectory.points)):
        q0 = trajectory.points[i - 1].positions
        q1 = trajectory.points[i].positions
        total += np.linalg.norm(q1 - q0)
    return total


def smooth_trajectory(trajectory, window_size=5):
    """Apply moving average smoothing to a trajectory.
    
    Args:
        trajectory: Trajectory object
        window_size: number of points in smoothing window
    
    Returns:
        New smoothed Trajectory
    """
    if len(trajectory.points) < window_size:
        return trajectory
    
    smoothed = Trajectory(trajectory.traj_type)
    smoothed.num_joints = trajectory.num_joints
    smoothed.total_time = trajectory.total_time
    
    positions = trajectory.get_position_matrix()
    half_w = window_size // 2
    
    for i in range(len(trajectory.points)):
        start = max(0, i - half_w)
        end = min(len(positions), i + half_w + 1)
        smoothed_pos = np.mean(positions[start:end], axis=0)
        
        smoothed.add_point(TrajectoryPoint(
            time=trajectory.points[i].time,
            positions=smoothed_pos,
        ))
    
    return smoothed


def resample_trajectory(trajectory, num_points=200):
    """Resample a trajectory to have exactly num_points evenly spaced in time.
    
    Args:
        trajectory: Trajectory object
        num_points: desired number of points
    
    Returns:
        New resampled Trajectory
    """
    if len(trajectory.points) < 2:
        return trajectory
    
    times = trajectory.get_time_array()
    new_times = np.linspace(times[0], times[-1], num_points)
    
    resampled = Trajectory(trajectory.traj_type)
    resampled.num_joints = trajectory.num_joints
    resampled.total_time = trajectory.total_time
    
    for t in new_times:
        angles = trajectory.sample_at(t)
        resampled.add_point(TrajectoryPoint(
            time=t,
            positions=angles,
        ))
    
    return resampled
