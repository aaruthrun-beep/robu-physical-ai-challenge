import json
import os

DEFAULT_PREFS = {
    "theme": "dark",
    "render_backend": "pybullet",
    "language": "en",
    "simulation": {
        "gravity": [0, 0, -9.81],
        "timestep": 1.0 / 240.0,
        "real_time": True,
    },
    "communication": {
        "mode": "open_loop",
        "transport": "serial",
        "protocol": "grbl",
        "serial": {
            "port": "COM1",
            "baud_rate": 115200,
            "data_bits": 8,
            "stop_bits": 1,
            "parity": "none",
        },
        "ethernet": {
            "host": "192.168.1.100",
            "port": 8080,
            "timeout": 5.0,
        },
        "custom_protocol_path": "",
    },
    "control": {
        "server_port": 8765,
        "can_interface": "virtual",
        "can_channel": 0,
        "serial_port": "COM3",
        "serial_baud": 115200,
    },
    "joint_limits": {
        "x": [-180, 180],
        "y": [-90, 90],
        "z": [-135, 135],
        "a": [-180, 180],
        "b": [-120, 120],
        "c": [-180, 180],
    },
    "gripper": {
        "open_pwm": 2500,
        "close_pwm": 500,
    },
    "tmc": {
        "default_preset": "1.0A Low Noise",
        "gconf": 0x00C0,
        "chopconf": 0x00010053,
        "ihold_irun": 0x00040F10,
        "microsteps": 3,  # 1/8
        "spreadcycle": True,
        "interpolation": True,
    },
    "encoder": {
        "poll_interval_ms": 100,
        "auto_poll": True,
        "zero_offsets": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "as5600_resolution": 12,
        "as5600_degrees_per_lsb": 0.0879,
    },
    "nite": {
        "ethernet_host": "192.168.1.100",
        "ethernet_port": 8765,
        "serial_port": "COM3",
        "serial_baud": 115200,
        "timeout": 5.0,
    },
    "viewport": {
        "camera_distance": 2.0,
        "camera_yaw": 45.0,
        "camera_pitch": -30.0,
        "grid_enabled": True,
        "grid_size": 1.0,
        "grid_spacing": 0.1,
    },
    "program": {
        "default_speed": 50,
        "default_delay": 0.5,
        "default_transition": 1.5,
        "default_feed_rate": 600,
    },
}


class Settings:
    def __init__(self, path=None):
        self.path = path or self._default_path()
        self.data = dict(DEFAULT_PREFS)
        self.load()

    @staticmethod
    def _default_path():
        """Writable settings path — user dir when frozen, else package assets."""
        try:
            from main import user_data_dir
            return os.path.join(user_data_dir(), "settings.json")
        except Exception:
            return os.path.join(os.path.dirname(__file__), "..", "assets", "settings.json")

    def load(self):
        if os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    loaded = json.load(f)
                self._deep_merge(self.data, loaded)
            except Exception:
                pass

    def _deep_merge(self, base, override):
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._deep_merge(base[key], value)
            else:
                base[key] = value

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        with open(self.path, "w") as f:
            json.dump(self.data, f, indent=2)

    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)
