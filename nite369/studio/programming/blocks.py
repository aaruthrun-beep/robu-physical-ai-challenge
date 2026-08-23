from enum import Enum


class BlockType(Enum):
    MOVE_J = "move_j"
    MOVE_L = "move_l"
    MOVE_C = "move_c"
    HOME = "home"
    GRIPPER_OPEN = "gripper_open"
    GRIPPER_CLOSE = "gripper_close"
    DELAY = "delay"
    SET_DO = "set_do"
    WAIT_DI = "wait_di"
    COMMENT = "comment"
    POSE = "pose"
    SPEED = "speed"


BLOCK_TEMPLATES = {
    BlockType.MOVE_J: {
        "label": "Move Joint",
        "color": "#1E88E5",   # brand blue
        "icon": "↗",
        "params": {
            "targets": {"label": "Joint Positions (°)", "type": "joints", "default": [0]*6},
            "speed": {"label": "Speed (%)", "type": "float", "default": 50, "min": 1, "max": 100},
        },
    },
    BlockType.MOVE_L: {
        "label": "Move Linear",
        "color": "#7CB342",   # brand green
        "icon": "→",
        "params": {
            "x": {"label": "X (mm)", "type": "float", "default": 0},
            "y": {"label": "Y (mm)", "type": "float", "default": 0},
            "z": {"label": "Z (mm)", "type": "float", "default": 0},
            "speed": {"label": "Speed (%)", "type": "float", "default": 50},
        },
    },
    BlockType.MOVE_C: {
        "label": "Move Circular",
        "color": "#3a9af5",   # brand blue hover
        "icon": "⌒",
        "params": {
            "via_x": {"label": "Via X", "type": "float", "default": 0},
            "via_y": {"label": "Via Y", "type": "float", "default": 0},
            "via_z": {"label": "Via Z", "type": "float", "default": 0},
            "target_x": {"label": "Target X", "type": "float", "default": 0},
            "target_y": {"label": "Target Y", "type": "float", "default": 0},
            "target_z": {"label": "Target Z", "type": "float", "default": 0},
        },
    },
    BlockType.HOME: {
        "label": "Go Home",
        "color": "#2ECC71",   # brand success green
        "icon": "⌂",
        "params": {},
    },
    BlockType.GRIPPER_OPEN: {
        "label": "Gripper Open",
        "color": "#1E88E5",
        "icon": "◈",
        "params": {},
    },
    BlockType.GRIPPER_CLOSE: {
        "label": "Gripper Close",
        "color": "#3a9af5",
        "icon": "◆",
        "params": {},
    },
    BlockType.DELAY: {
        "label": "Wait",
        "color": "#95A5A6",
        "icon": "⏱",
        "params": {
            "seconds": {"label": "Seconds", "type": "float", "default": 1.0, "min": 0, "max": 60},
        },
    },
    BlockType.SET_DO: {
        "label": "Set Digital Out",
        "color": "#8bc34a",
        "icon": "◉",
        "params": {
            "pin": {"label": "Pin", "type": "int", "default": 1},
            "state": {"label": "State", "type": "bool", "default": True},
        },
    },
    BlockType.COMMENT: {
        "label": "Comment",
        "color": "#7F8C8D",
        "icon": "//",
        "params": {
            "text": {"label": "Comment", "type": "text", "default": ""},
        },
    },
    BlockType.POSE: {
        "label": "Recorded Pose",
        "color": "#4A9BE8",
        "icon": "◎",
        "params": {
            "pose_name": {"label": "Pose Name", "type": "select", "options": [], "default": ""},
        },
    },
    BlockType.SPEED: {
        "label": "Set Speed",
        "color": "#7CB342",
        "icon": "↻",
        "params": {
            "speed": {"label": "Speed (%)", "type": "float", "default": 50, "min": 1, "max": 100},
        },
    },
}


class ProgramBlock:
    def __init__(self, block_type=BlockType.MOVE_J):
        self.type = block_type
        template = BLOCK_TEMPLATES.get(block_type, {})
        self.label = template.get("label", "Unknown")
        self.color = template.get("color", "#000000")
        self.icon = template.get("icon", "?")
        self.params = {}
        for key, cfg in template.get("params", {}).items():
            self.params[key] = cfg["default"]
