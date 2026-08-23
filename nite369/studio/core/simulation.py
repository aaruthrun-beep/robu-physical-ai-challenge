import pybullet as p
import pybullet_data
import numpy as np
import os
import json
import time
import logging

log = logging.getLogger("astra_studio.simulation")

# Small delay so pybullet can fully disconnect before reconnecting.
RESET_DELAY = 0.5


class SimulationEngine:
    def __init__(self, gui=True):
        self.gui = gui
        self.client_id = None
        self.robots = {}
        self.bodies = {}
        self.grippers = {}
        self.timestep = 1.0 / 240.0
        self.gravity = [0, 0, -9.81]
        self.running = False
        self.sim_time = 0.0
        self._joint_name_to_index = {}
        self._saved_poses = {}

    def start(self):
        mode = p.GUI if self.gui else p.DIRECT
        try:
            self.client_id = p.connect(mode)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(*self.gravity, physicsClientId=self.client_id)
            p.setTimeStep(self.timestep, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self.client_id)
            self._setup_ground()
            self.running = True
            log.info("Simulation engine started")
        except Exception as e:
            log.error("Couldn't start the simulation engine: %s", e)
            raise RuntimeError(f"Couldn't start the simulation engine: {e}") from e
        return self

    def stop(self):
        self.running = False
        if self.client_id is not None:
            p.disconnect(physicsClientId=self.client_id)
            self.client_id = None

    def _setup_ground(self):
        """Create a ground plane WITHOUT a URDF file.

        The old code loaded ``plane.urdf`` from pybullet_data, which is
        missing in the frozen EXE (the search path doesn't resolve to the
        bundled data dir) — the app crashed at startup with
        "Cannot load URDF file". A programmatic plane body is dependency-free
        and works in both source and frozen builds.
        """
        ground_shape = p.createCollisionShape(
            p.GEOM_PLANE, planeNormal=[0, 0, 1], physicsClientId=self.client_id
        )
        ground_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[50, 50, 0.01], rgbaColor=[0.3, 0.3, 0.35, 1.0],
            physicsClientId=self.client_id,
        )
        plane = p.createMultiBody(
            0, ground_shape, ground_visual, basePosition=[0, 0, -0.01],
            physicsClientId=self.client_id,
        )
        self.bodies["ground"] = plane

    def load_robot(self, name, urdf_path, base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1],
                   scale=1.0, fixed_base=True):
        if not os.path.exists(urdf_path):
            log.error("Robot model file not found: %s", urdf_path)
            raise FileNotFoundError(f"Couldn't find the robot model file at {urdf_path}")
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            baseOrientation=base_orientation,
            useFixedBase=fixed_base,
            globalScaling=scale,
            physicsClientId=self.client_id,
        )
        self.robots[name] = robot_id
        self._build_joint_map(name)
        return robot_id

    def _build_joint_map(self, robot_name):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return
        self._joint_name_to_index[robot_name] = {}
        for i in range(p.getNumJoints(robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(robot_id, i, physicsClientId=self.client_id)
            name = info[1].decode("utf-8")
            self._joint_name_to_index[robot_name][name] = i

    def get_joint_info(self, robot_name):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return []
        info = []
        for i in range(p.getNumJoints(robot_id, physicsClientId=self.client_id)):
            joint = p.getJointInfo(robot_id, i, physicsClientId=self.client_id)
            info.append({
                "index": i,
                "name": joint[1].decode("utf-8"),
                "type": joint[2],
                "lower_limit": joint[8],
                "upper_limit": joint[9],
                "max_force": joint[10],
                "max_velocity": joint[11],
                "joint_axis": list(joint[13]),
                "parent_index": joint[16],
            })
        return info

    def get_num_joints(self, robot_name):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return 0
        return p.getNumJoints(robot_id, physicsClientId=self.client_id)

    def get_revolute_joints(self, robot_name):
        info = self.get_joint_info(robot_name)
        return [j for j in info if j["type"] == p.JOINT_REVOLUTE]

    def set_joint_positions(self, robot_name, positions, joint_indices=None):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return
        num_joints = p.getNumJoints(robot_id, physicsClientId=self.client_id)
        if joint_indices is None:
            revolute = self.get_revolute_joints(robot_name)
            joint_indices = [j["index"] for j in revolute]
        # Apply instantly via resetJointState so reads are immediate (the
        # POSITION_CONTROL path lags until stepSimulation runs, which made
        # Record/Update capture stale zeros).
        for i, idx in enumerate(joint_indices):
            if i < len(positions) and idx < num_joints:
                try:
                    p.resetJointState(robot_id, idx, float(positions[i]),
                                      physicsClientId=self.client_id)
                except Exception:
                    p.setJointMotorControl2(
                        robot_id, idx, p.POSITION_CONTROL,
                        targetPosition=positions[i],
                        force=500.0, maxVelocity=10.0,
                        physicsClientId=self.client_id,
                    )

    def get_joint_positions(self, robot_name):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return []
        revolute = self.get_revolute_joints(robot_name)
        if revolute:
            idxs = [j["index"] for j in revolute]
            states = p.getJointStates(robot_id, idxs, physicsClientId=self.client_id)
            return [s[0] for s in states]
        num = p.getNumJoints(robot_id, physicsClientId=self.client_id)
        states = p.getJointStates(robot_id, range(num), physicsClientId=self.client_id)
        return [s[0] for s in states]

    def get_endeffector_pose(self, robot_name, link_index=-1):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return None
        num_joints = p.getNumJoints(robot_id, physicsClientId=self.client_id)
        if link_index < 0:
            link_index = num_joints - 1
        state = p.getLinkState(robot_id, link_index, physicsClientId=self.client_id)
        return {"position": list(state[4]), "orientation": list(state[5])}

    def calculate_ik(self, robot_name, target_pos, target_orient=None, link_index=-1):
        robot_id = self.robots.get(robot_name)
        if robot_id is None:
            return []
        num_joints = p.getNumJoints(robot_id, physicsClientId=self.client_id)
        if link_index < 0:
            link_index = num_joints - 1
        if target_orient is None:
            target_orient = [0, 0, 0, 1]
        joint_angles = p.calculateInverseKinematics(
            robot_id, link_index, target_pos, target_orient,
            physicsClientId=self.client_id,
        )
        revolute = self.get_revolute_joints(robot_name)
        return [joint_angles[j["index"]] for j in revolute]

    def add_box(self, name, size, position, color=None, mass=0.1, orientation=[0, 0, 0, 1]):
        vs = p.createVisualShape(p.GEOM_BOX, halfExtents=size, physicsClientId=self.client_id)
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=size, physicsClientId=self.client_id)
        if color:
            vs = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=color,
                                     physicsClientId=self.client_id)
        body = p.createMultiBody(
            baseMass=mass, baseVisualShapeIndex=vs, baseCollisionShapeIndex=cs,
            basePosition=position, baseOrientation=orientation,
            physicsClientId=self.client_id,
        )
        self.bodies[name] = body
        return body

    def add_cylinder(self, name, radius, height, position, color=None, mass=0.1):
        vs = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height,
                                 physicsClientId=self.client_id)
        cs = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height,
                                    physicsClientId=self.client_id)
        if color:
            vs = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height,
                                     rgbaColor=color, physicsClientId=self.client_id)
        body = p.createMultiBody(
            baseMass=mass, baseVisualShapeIndex=vs, baseCollisionShapeIndex=cs,
            basePosition=position, physicsClientId=self.client_id,
        )
        self.bodies[name] = body
        return body

    def add_sphere(self, name, radius, position, color=None, mass=0.1):
        vs = p.createVisualShape(p.GEOM_SPHERE, radius=radius, physicsClientId=self.client_id)
        cs = p.createCollisionShape(p.GEOM_SPHERE, radius=radius, physicsClientId=self.client_id)
        if color:
            vs = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color,
                                     physicsClientId=self.client_id)
        body = p.createMultiBody(
            baseMass=mass, baseVisualShapeIndex=vs, baseCollisionShapeIndex=cs,
            basePosition=position, physicsClientId=self.client_id,
        )
        self.bodies[name] = body
        return body

    def add_workbench(self, name="workbench", position=[0.5, 0, 0]):
        wb = self.add_box(name, [0.3, 0.2, 0.02], position, [0.6, 0.4, 0.2, 1.0], mass=0)
        leg_positions = [
            [position[0] - 0.25, position[1] - 0.15, position[2] - 0.15],
            [position[0] + 0.25, position[1] - 0.15, position[2] - 0.15],
            [position[0] - 0.25, position[1] + 0.15, position[2] - 0.15],
            [position[0] + 0.25, position[1] + 0.15, position[2] - 0.15],
        ]
        for i, lp in enumerate(leg_positions):
            self.add_cylinder(f"{name}_leg_{i}", 0.02, 0.3, lp, [0.4, 0.3, 0.15, 1.0], mass=0)
        return wb

    def add_conveyor(self, name="conveyor", position=[0.5, -0.3, 0]):
        belt = self.add_box(f"{name}_belt", [0.25, 0.08, 0.02], position, [0.3, 0.3, 0.35, 1.0], mass=0)
        left_roll = self.add_cylinder(f"{name}_roll_l", 0.03, 0.08,
                                       [position[0] - 0.2, position[1], position[2] + 0.04],
                                       [0.5, 0.5, 0.5, 1.0], mass=0)
        right_roll = self.add_cylinder(f"{name}_roll_r", 0.03, 0.08,
                                        [position[0] + 0.2, position[1], position[2] + 0.04],
                                        [0.5, 0.5, 0.5, 1.0], mass=0)
        leg1 = self.add_cylinder(f"{name}_leg1", 0.015, 0.2,
                                  [position[0] - 0.15, position[1] - 0.05, position[2] - 0.12],
                                  [0.4, 0.3, 0.15, 1.0], mass=0)
        leg2 = self.add_cylinder(f"{name}_leg2", 0.015, 0.2,
                                  [position[0] + 0.15, position[1] - 0.05, position[2] - 0.12],
                                  [0.4, 0.3, 0.15, 1.0], mass=0)
        return belt

    def remove_body(self, name):
        body = self.bodies.pop(name, None)
        if body is not None:
            p.removeBody(body, physicsClientId=self.client_id)

    def get_body_position(self, name):
        body = self.bodies.get(name)
        if body is None:
            return None
        pos, orient = p.getBasePositionAndOrientation(body, physicsClientId=self.client_id)
        return list(pos)

    def set_body_position(self, name, position):
        body = self.bodies.get(name)
        if body is None:
            return
        _, orient = p.getBasePositionAndOrientation(body, physicsClientId=self.client_id)
        p.resetBasePositionAndOrientation(body, position, orient, physicsClientId=self.client_id)

    def save_pose(self, name, robot_name="astra"):
        joints = self.get_joint_positions(robot_name)
        self._saved_poses[name] = joints
        return joints

    def load_pose(self, name, robot_name="astra"):
        joints = self._saved_poses.get(name)
        if joints is None:
            return False
        self.set_joint_positions(robot_name, joints)
        return True

    def get_saved_poses(self):
        return dict(self._saved_poses)

    def delete_pose(self, name):
        self._saved_poses.pop(name, None)

    def step(self):
        if self.running and self.client_id is not None:
            p.stepSimulation(physicsClientId=self.client_id)
            self.sim_time += self.timestep

    def reset(self):
        log.info("Resetting the simulation…")
        self.stop()
        time.sleep(RESET_DELAY)
        self.start()
        self.sim_time = 0.0

    def set_camera(self, distance=2.0, yaw=45.0, pitch=-30.0, target=[0, 0, 0.5]):
        if self.client_id is not None:
            p.resetDebugVisualizerCamera(distance, yaw, pitch, target,
                                         physicsClientId=self.client_id)

    def get_camera_image(self, width=640, height=480, view_matrix=None, proj_matrix=None):
        if self.client_id is None:
            return None
        if view_matrix is None:
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0, 0.5],
                distance=2.0,
                yaw=45.0,
                pitch=-30.0,
                roll=0,
                upAxisIndex=2,
            )
        if proj_matrix is None:
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=60.0, aspect=width / height, nearVal=0.1, farVal=100.0,
            )
        _, _, rgb, depth, seg = p.getCameraImage(
            width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
            physicsClientId=self.client_id,
        )
        return rgb, depth, seg
