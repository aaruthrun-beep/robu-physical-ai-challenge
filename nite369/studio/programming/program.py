"""Robot program and target models.

Supports both the legacy step-based format and the new target-based
Arctos-style format with backward compatibility.
"""

from enum import Enum
import json
import uuid
from datetime import datetime


class MoveType(Enum):
    JOINT = "joint"
    LINEAR = "linear"
    CIRCULAR = "circular"
    HOME = "home"
    GRIPPER = "gripper"
    DELAY = "delay"
    IO = "io"
    COMMENT = "comment"


class Target:
    """Single robot position target (Arctos-style)."""

    def __init__(self, name="Target", joints=None, gripper=None):
        self.id = str(uuid.uuid4())[:8]
        self.name = name
        self.joints = joints or [0.0] * 6
        self.gripper = gripper
        self.speed = 50.0
        self.transition_time = 1.5
        self.delay = 0.0
        self.comment = ""

    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "joints": self.joints,
            "gripper": self.gripper,
            "speed": self.speed,
            "transition_time": self.transition_time,
            "delay": self.delay,
            "comment": self.comment,
        }

    @classmethod
    def from_dict(cls, data):
        t = cls(
            name=data.get("name", "Target"),
            joints=data.get("joints", data.get("targets", [0.0] * 6)),
            gripper=data.get("gripper"),
        )
        t.id = data.get("id", t.id)
        t.speed = data.get("speed", 50.0)
        t.transition_time = data.get("transition_time", 1.5)
        t.delay = data.get("delay", 0.0)
        t.comment = data.get("comment", "")
        return t


class Program:
    """Robot program — ordered list of targets (Arctos-style)."""

    def __init__(self, name="Untitled"):
        self.name = name
        self.targets = []
        self.created = datetime.now().isoformat()
        self.modified = self.created
        self.params = {
            "speed": 50,
            "delay": 0.5,
            "transition": 1.5,
        }

    def add_target(self, target):
        self.targets.append(target)
        self.modified = datetime.now().isoformat()

    def remove_target(self, index):
        if 0 <= index < len(self.targets):
            self.targets.pop(index)
            self.modified = datetime.now().isoformat()

    def move_target(self, from_idx, to_idx):
        if 0 <= from_idx < len(self.targets) and 0 <= to_idx < len(self.targets):
            t = self.targets.pop(from_idx)
            self.targets.insert(to_idx, t)
            self.modified = datetime.now().isoformat()

    def save(self, path):
        data = {
            "name": self.name,
            "created": self.created,
            "modified": self.modified,
            "params": self.params,
            "targets": [t.to_dict() for t in self.targets],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)

        if "targets" in data:
            prog = cls(data.get("name", "Imported"))
            prog.created = data.get("created", prog.created)
            prog.modified = data.get("modified", prog.modified)
            prog.params = data.get("params", prog.params)
            prog.targets = [Target.from_dict(t) for t in data["targets"]]
            return prog

        if "steps" in data:
            prog = cls(data.get("name", "Imported"))
            for step_data in data["steps"]:
                t = Target(
                    name=step_data.get("label", f"Target {len(prog.targets)+1}"),
                    joints=step_data.get("targets", [0.0] * 6),
                    gripper=step_data.get("gripper"),
                )
                t.speed = step_data.get("speed", 50.0)
                t.delay = step_data.get("delay", 0.0)
                prog.targets.append(t)
            return prog

        return cls(data.get("name", "Imported"))

    @property
    def num_targets(self):
        return len(self.targets)


class ProgramStep:
    """Legacy step-based program format (backward compatibility)."""

    def __init__(self, move_type=MoveType.JOINT, targets=None, speed=50.0, label=""):
        self.id = str(uuid.uuid4())[:8]
        self.type = move_type
        self.targets = targets or [0.0] * 6
        self.speed = speed
        self.acceleration = 20.0
        self.label = label
        self.gripper = None
        self.delay = 0.0
        self.comment = ""
        self.pose_name = ""
        self.io_pin = 0
        self.io_state = False

    def to_dict(self):
        return {
            "id": self.id,
            "type": self.type.value,
            "targets": self.targets,
            "speed": self.speed,
            "acceleration": self.acceleration,
            "label": self.label,
            "gripper": self.gripper,
            "delay": self.delay,
            "comment": self.comment,
            "pose_name": self.pose_name,
            "io_pin": self.io_pin,
            "io_state": self.io_state,
        }

    @classmethod
    def from_dict(cls, data):
        step = cls(
            move_type=MoveType(data.get("type", "joint")),
            targets=data.get("targets", [0.0] * 6),
            speed=data.get("speed", 50.0),
            label=data.get("label", ""),
        )
        step.id = data.get("id", step.id)
        step.acceleration = data.get("acceleration", 20.0)
        step.gripper = data.get("gripper")
        step.delay = data.get("delay", 0.0)
        step.comment = data.get("comment", "")
        step.pose_name = data.get("pose_name", "")
        step.io_pin = data.get("io_pin", 0)
        step.io_state = data.get("io_state", False)
        return step


class RobotProgram:
    """Legacy robot program (step-based, backward compatible)."""

    def __init__(self, name="Untitled"):
        self.name = name
        self.steps = []
        self.author = ""
        self.created = None
        self.modified = None
        self.description = ""
        self.target_robot = "astra"

    def add_step(self, step):
        self.steps.append(step)

    def insert_step(self, index, step):
        self.steps.insert(index, step)

    def remove_step(self, index):
        if 0 <= index < len(self.steps):
            return self.steps.pop(index)

    def reorder(self, from_idx, to_idx):
        if from_idx < 0 or from_idx >= len(self.steps) or to_idx < 0 or to_idx >= len(self.steps):
            return False
        step = self.steps.pop(from_idx)
        self.steps.insert(to_idx, step)
        return True

    def to_dict(self):
        return {
            "name": self.name,
            "author": self.author,
            "description": self.description,
            "target_robot": self.target_robot,
            "steps": [s.to_dict() for s in self.steps],
        }

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        program = cls(data.get("name", "Imported"))
        program.author = data.get("author", "")
        program.description = data.get("description", "")
        program.target_robot = data.get("target_robot", "astra")
        program.steps = [ProgramStep.from_dict(s) for s in data.get("steps", [])]
        return program

    @property
    def num_steps(self):
        return len(self.steps)
