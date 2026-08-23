"""Unified motion controller for Astra Studio Pro.

Bridges simulation and hardware, providing smooth interpolated motion
with optional real-time command streaming to the physical robot.

Note: ``move_joints`` runs a short blocking interpolation loop. It is
designed to be called from a worker thread (e.g. the program runner's
QThread) so it never freezes the UI.
"""

import time
import math
import logging
import numpy as np

log = logging.getLogger("astra_studio.controller")


class RobotController:
    """Unified motion controller — bridges simulation and hardware."""

    def __init__(self, sim_engine=None, connection_manager=None):
        self.sim = sim_engine
        self.connection = connection_manager
        self.current_joints = [0.0] * 6
        self.speed = 50.0
        self.feed_rate = 600
        self._joint_names = ["X", "Y", "Z", "A", "B", "C"]

    def move_joints(self, targets, speed=None, transition_time=1.5, send_to_robot=False):
        """Move robot to target joint positions with smooth interpolation."""
        speed = speed or self.speed
        targets = list(targets[:6])

        if len(targets) < 6:
            targets.extend([0.0] * (6 - len(targets)))

        steps = max(10, int(transition_time * 30))
        start = list(self.current_joints)

        for step in range(1, steps + 1):
            t = step / steps
            eased_t = t * t * (3 - 2 * t)

            interpolated = [
                start[i] + (targets[i] - start[i]) * eased_t
                for i in range(6)
            ]

            if self.sim and "astra" in self.sim.robots:
                self.sim.set_joint_positions("astra", interpolated)

            if send_to_robot and self.connection and self.connection.is_connected:
                self.connection.move_joints(interpolated, speed=speed)

            time.sleep(1.0 / 30)

        self.current_joints = list(targets)
        log.debug("Moved joints to %s", [f"{v:.2f}" for v in targets])

    def move_joints_immediate(self, targets, send_to_robot=False):
        """Move robot immediately without interpolation."""
        targets = list(targets[:6])
        if len(targets) < 6:
            targets.extend([0.0] * (6 - len(targets)))

        if self.sim and "astra" in self.sim.robots:
            self.sim.set_joint_positions("astra", targets)

        if send_to_robot and self.connection and self.connection.is_connected:
            self.connection.move_joints(targets, speed=self.speed)

        self.current_joints = list(targets)

    def home(self, send_to_robot=False):
        """Move to home position."""
        self.move_joints([0.0] * 6, send_to_robot=send_to_robot)
        if send_to_robot and self.connection and self.connection.is_connected:
            self.connection.home()

    def stop(self, send_to_robot=False):
        """Emergency stop."""
        if send_to_robot and self.connection and self.connection.is_connected:
            self.connection.stop()

    def unlock(self, send_to_robot=False):
        """Unlock robot (clear alarm state)."""
        if send_to_robot and self.connection and self.connection.is_connected:
            self.connection.unlock()

    def set_gripper(self, position, send_to_robot=False):
        """Set gripper position (0.0=open, 1.0=closed)."""
        position = max(0.0, min(1.0, position))
        if send_to_robot and self.connection and self.connection.is_connected:
            self.connection.set_gripper(position)

    def get_current_joints(self):
        """Get current joint positions."""
        if self.sim and "astra" in self.sim.robots:
            return list(self.sim.get_joint_positions("astra"))
        return list(self.current_joints)

    def get_end_effector_pose(self):
        """Get end-effector position and orientation."""
        if self.sim and "astra" in self.sim.robots:
            return self.sim.get_endeffector_pose("astra")
        return None

    def calculate_ik(self, target_pos, target_orient=None):
        """Calculate inverse kinematics for a target position."""
        if self.sim and "astra" in self.sim.robots:
            return self.sim.calculate_ik("astra", target_pos, target_orient)
        return None

    def set_speed(self, speed):
        """Set global speed (1-100%)."""
        self.speed = max(1, min(100, speed))

    def set_feed_rate(self, rate):
        """Set feed rate in mm/min."""
        self.feed_rate = max(1, min(10000, rate))
