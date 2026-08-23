"""Protocol adapters for robot communication.

Provides a pluggable interface for different robot firmware protocols.
Each adapter translates high-level commands (move_joints, home, etc.)
into protocol-specific wire format.
"""

import re
import time
import threading
from enum import Enum
from typing import Optional, Callable

from PyQt5.QtCore import QObject, pyqtSignal, QTimer


class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    HOMING = "homing"
    ERROR = "error"
    ALARM = "alarm"
    UNKNOWN = "unknown"


class ProtocolAdapter(QObject):
    """Abstract base for robot protocol adapters."""

    state_changed = pyqtSignal(str)
    message_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    position_updated = pyqtSignal(list)
    command_sent = pyqtSignal(str)

    def __init__(self, transport, parent=None):
        super().__init__(parent)
        self.transport = transport
        self.state = RobotState.UNKNOWN
        self.current_joints = [0.0] * 6
        self._lock = threading.Lock()

    def move_joints(self, positions: list, speed: float = 50.0) -> bool:
        raise NotImplementedError

    def home(self) -> bool:
        raise NotImplementedError

    def unlock(self) -> bool:
        raise NotImplementedError

    def stop(self) -> bool:
        raise NotImplementedError

    def set_gripper(self, position: float) -> bool:
        raise NotImplementedError

    def get_status(self) -> dict:
        raise NotImplementedError

    def get_joints(self) -> list:
        raise NotImplementedError

    def run_gcode(self, gcode: str) -> bool:
        raise NotImplementedError

    def _send_command(self, cmd: str, wait_ok: bool = True, timeout: float = 5.0) -> str:
        """Send a command and wait for response."""
        if not self.transport.is_connected():
            return ""

        with self._lock:
            self.transport.reset_buffer()
            data = (cmd.strip() + "\n").encode("ascii")
            if not self.transport.send(data):
                return ""

            if not wait_ok:
                return ""

            deadline = time.time() + timeout
            while time.time() < deadline:
                line = self.transport.read_line(timeout=0.1)
                if line:
                    decoded = line.decode("ascii", errors="replace").strip()
                    self.message_received.emit(decoded)
                    if decoded.startswith("ok"):
                        return decoded
                    if decoded.startswith("error"):
                        self.error_occurred.emit(decoded)
                        return decoded
                    if decoded.startswith("<"):
                        return decoded
            return ""


class GRBLAdapter(ProtocolAdapter):
    """Standard GRBL protocol adapter for stepper-based robots.

    Supports:
    - G0/G1 linear moves
    - G28 home
    - $X unlock
    - $H home
    - M3/M5 spindle (gripper)
    - $$ settings query
    - <Idle|...> status reports
    """

    def __init__(self, transport, parent=None):
        super().__init__(transport, parent)
        self.feed_rate = 600
        self.max_speed = 1000
        self._step_per_mm = [200.0] * 6
        self._joint_names = ["X", "Y", "Z", "A", "B", "C"]

    def connect_init(self):
        """Send initialization commands after connection."""
        time.sleep(0.5)
        self._send_command("$X")
        time.sleep(0.1)
        self._send_command("$G")
        self._query_status()

    def move_joints(self, positions: list, speed: float = 50.0) -> bool:
        if len(positions) < 6:
            return False

        self.state = RobotState.MOVING
        self.state_changed.emit(self.state.value)

        fr = max(1, int(self.feed_rate * speed / 100.0))
        gcode = f"G1 F{fr}"
        for i, pos in enumerate(positions[:6]):
            val = pos * self._step_per_mm[i]
            gcode += f" {self._joint_names[i]}{val:.4f}"

        result = self._send_command(gcode, timeout=30.0)
        self.current_joints = list(positions[:6])
        self.position_updated.emit(self.current_joints)

        if result.startswith("error"):
            self.state = RobotState.ERROR
            self.state_changed.emit(self.state.value)
            return False

        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def home(self) -> bool:
        self.state = RobotState.HOMING
        self.state_changed.emit(self.state.value)

        result = self._send_command("$H", timeout=60.0)
        if result.startswith("error"):
            result = self._send_command("G28", timeout=60.0)

        self.current_joints = [0.0] * 6
        self.position_updated.emit(self.current_joints)

        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return not result.startswith("error")

    def unlock(self) -> bool:
        result = self._send_command("$X")
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def stop(self) -> bool:
        self._send_command("M8", wait_ok=False)
        self.transport.send(b"!")
        time.sleep(0.05)
        self.transport.send(b"\x18")
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def set_gripper(self, position: float) -> bool:
        position = max(0.0, min(1.0, position))
        pwm = int(500 + position * 2000)
        gcode = f"M3 S{pwm}"
        result = self._send_command(gcode)
        return not result.startswith("error")

    def get_status(self) -> dict:
        self._query_status()
        return {
            "state": self.state.value,
            "joints": list(self.current_joints),
            "feed_rate": self.feed_rate,
        }

    def get_joints(self) -> list:
        return list(self.current_joints)

    def run_gcode(self, gcode: str) -> bool:
        lines = gcode.strip().split("\n")
        for line in lines:
            line = line.strip()
            if not line or line.startswith(";") or line.startswith("("):
                continue
            result = self._send_command(line, timeout=60.0)
            if result.startswith("error"):
                return False
        return True

    def _query_status(self):
        result = self._send_command("?", timeout=1.0)
        if result.startswith("<"):
            self._parse_status(result)

    def _parse_status(self, status_line: str):
        try:
            match = re.match(r"<(\w+)", status_line)
            if match:
                state_str = match.group(1).lower()
                state_map = {
                    "idle": RobotState.IDLE,
                    "run": RobotState.MOVING,
                    "hold": RobotState.IDLE,
                    "home": RobotState.HOMING,
                    "alarm": RobotState.ALARM,
                    "error": RobotState.ERROR,
                    "jog": RobotState.MOVING,
                }
                self.state = state_map.get(state_str, RobotState.UNKNOWN)
                self.state_changed.emit(self.state.value)

            pos_match = re.search(r"WPos:([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)", status_line)
            if pos_match:
                vals = [float(pos_match.group(i)) for i in range(1, 7)]
                self.current_joints = vals
                self.position_updated.emit(self.current_joints)
        except Exception:
            pass


class CustomFirmwareAdapter(ProtocolAdapter):
    """Template for custom firmware protocols.

    Users: Replace the method bodies with your firmware's specific commands.
    The GRBL adapter above serves as a reference implementation.

    Your protocol documentation should specify:
    1. Command format (G-code, binary, JSON, etc.)
    2. Response format (ok/error, status reports)
    3. Joint addressing (which axis index maps to which joint)
    4. Gripper control method (servo PWM, digital output, etc.)
    """

    def __init__(self, transport, parent=None):
        super().__init__(transport, parent)
        self._joint_names = ["X", "Y", "Z", "A", "B", "C"]

    def move_joints(self, positions: list, speed: float = 50.0) -> bool:
        if len(positions) < 6:
            return False

        self.state = RobotState.MOVING
        self.state_changed.emit(self.state.value)

        # TODO: Replace with your firmware's move command
        # Example formats:
        #   G-code: G1 X10 Y20 Z30 A0 B0 C0 F600
        #   Binary: [0x01] [joint_bytes...] [speed_bytes...]
        #   JSON: {"cmd": "move", "joints": [10,20,30,0,0,0], "speed": 600}

        cmd_parts = []
        for i, pos in enumerate(positions[:6]):
            cmd_parts.append(f"{self._joint_names[i]}{pos:.4f}")
        gcode = f"G1 {' '.join(cmd_parts)} F{int(speed * 12)}"

        result = self._send_command(gcode, timeout=30.0)
        self.current_joints = list(positions[:6])
        self.position_updated.emit(self.current_joints)

        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def home(self) -> bool:
        self.state = RobotState.HOMING
        self.state_changed.emit(self.state.value)

        # TODO: Replace with your firmware's home command
        result = self._send_command("G28", timeout=60.0)

        self.current_joints = [0.0] * 6
        self.position_updated.emit(self.current_joints)

        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return not result.startswith("error")

    def unlock(self) -> bool:
        # TODO: Replace with your firmware's unlock command
        result = self._send_command("$X")
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def stop(self) -> bool:
        # TODO: Replace with your firmware's stop command
        self.transport.send(b"!")
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def set_gripper(self, position: float) -> bool:
        position = max(0.0, min(1.0, position))
        # TODO: Replace with your gripper control method
        pwm = int(500 + position * 2000)
        result = self._send_command(f"M3 S{pwm}")
        return not result.startswith("error")

    def get_status(self) -> dict:
        # TODO: Replace with your firmware's status query
        self._query_status()
        return {
            "state": self.state.value,
            "joints": list(self.current_joints),
            "feed_rate": 600,
        }

    def get_joints(self) -> list:
        return list(self.current_joints)

    def run_gcode(self, gcode: str) -> bool:
        lines = gcode.strip().split("\n")
        for line in lines:
            line = line.strip()
            if not line or line.startswith(";"):
                continue
            result = self._send_command(line, timeout=60.0)
            if result.startswith("error"):
                return False
        return True

    def _query_status(self):
        result = self._send_command("?", timeout=1.0)
        if result.startswith("<"):
            self._parse_status(result)

    def _parse_status(self, status_line: str):
        try:
            match = re.match(r"<(\w+)", status_line)
            if match:
                state_str = match.group(1).lower()
                state_map = {
                    "idle": RobotState.IDLE,
                    "run": RobotState.MOVING,
                    "home": RobotState.HOMING,
                    "alarm": RobotState.ALARM,
                    "error": RobotState.ERROR,
                }
                self.state = state_map.get(state_str, RobotState.UNKNOWN)
                self.state_changed.emit(self.state.value)
        except Exception:
            pass


# ==========================================================================
# Nite 369 Protocol Adapter
# ==========================================================================

class Nite369Protocol(ProtocolAdapter):
    """
    Custom ASCII protocol for Nite 369 distributed firmware.

    Communicates with the Master Pico over TCP or Serial.
    The Master Pico bridges commands to Slave 1 (arm base) and
    Slave 2 (wrist) via SPI, aggregating encoder and TMC data.

    Command Set (newline-terminated ASCII):
        #P          → Poll all: positions + encoders
        #E          → Poll encoders only
        #S          → Get system status
        #TR<addr>,<reg>  → TMC register read
        #TW<addr>,<reg>,<hex> → TMC register write
        #M<j1>,...,<j6>  → Move joints to absolute positions
        #H          → Home all axes
        #EN<mask>   → Enable drivers (bitmask)
        #DI<mask>   → Disable drivers (bitmask)
        #V          → Get firmware version
        #T<addr>    → Read TMC DRV_STATUS

    Response Format:
        >P:<j1>,...,<j6>|<e1>,...,<e6>   — Positions + encoders
        >E:<e1>,...,<e6>                  — Encoder values
        >S:<state>,<flags>                — System status
        >TR:<addr>,<reg>,<hex>            — Register value
        >T:<addr>,<hex>                   — DRV_STATUS
        >OK                               — Command accepted
        >ER:<message>                     — Error
        >V:<version>                      — Firmware version
    """

    # Nite-specific signals
    encoder_updated = pyqtSignal(list)         # [j1..j6] encoder degrees
    positions_updated = pyqtSignal(list)        # [j1..j6] commanded positions
    tmc_status_updated = pyqtSignal(int, dict)   # driver_addr, status_dict
    tmc_reg_read = pyqtSignal(int, int, int)     # driver_addr, reg, value
    system_state_changed = pyqtSignal(str, int)  # state, enabled_mask
    version_received = pyqtSignal(str)

    JOINTS_NAMES = ["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)",
                    "J4 (Forearm Roll)", "J5 (Wrist Pitch)", "J6 (Wrist Roll)"]

    # Joint to encoder mapping
    JOINT_ENCODER_MAP = {
        # Slave 1 (Arm Base): 4 encoders (J1, J2A, J2B, J3)
        # J2 uses dual motors (J2A + J2B) → averaged
        # Slave 2 (Wrist): 3 encoders (J4, J5, J6)
        0: {"slave": 1, "enc_ch": 0},   # J1
        1: {"slave": 1, "enc_ch": [1, 2]},  # J2 (dual, averaged)
        2: {"slave": 1, "enc_ch": 3},   # J3
        3: {"slave": 2, "enc_ch": 0},   # J4
        4: {"slave": 2, "enc_ch": 1},   # J5
        5: {"slave": 2, "enc_ch": 2},   # J6
    }

    # TMC driver mapping (Slave 2 only — TMC2209 on J4, J5, J6, Gripper)
    TMC_DRIVERS = [
        {"addr": 0x00, "name": "J4 (Forearm Roll)"},
        {"addr": 0x01, "name": "J5 (Wrist Pitch)"},
        {"addr": 0x02, "name": "J6 (Wrist Roll)"},
        {"addr": 0x03, "name": "Gripper"},
    ]

    def __init__(self, transport, parent=None):
        super().__init__(transport, parent)
        self._joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        self.encoder_values = [0.0] * 6
        self.encoder_raw = [0] * 6
        self.encoder_zero_offsets = [0.0] * 6
        self.commanded_positions = [0.0] * 6
        self.enabled_mask = 0
        self.firmware_version = ""
        self._home_cfg_cache = {}
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._auto_poll)
        self._poll_interval_ms = 100  # 10 Hz default

    # ── Connection Init ─────────────────────────────────────────────

    def connect_init(self):
        """Send initialization sequence after connection."""
        time.sleep(0.5)
        # Real master firmware (spi_cmd_master_eth.c) command set:
        #   #PING / #V / #P / #MV<j>,<speed>,<steps> / #H / #EN / #DI
        self._send_command("V", wait_ok=True, timeout=3.0)
        self._send_command("PING", wait_ok=True, timeout=3.0)
        # NOTE: no internal auto-poll here — the ConnectionManager's worker
        # already polls status once per second. A second poll timer (the old
        # 100ms default) flooded the master with #P and starved moves.
        # self.start_auto_poll()  # disabled

    # ── Auto Polling (kept for API compat; not started automatically) ──

    def start_auto_poll(self, interval_ms: int = 100):
        """Start automatic encoder/status polling."""
        self._poll_interval_ms = max(20, min(1000, interval_ms))
        self._status_timer.start(self._poll_interval_ms)

    def stop_auto_poll(self):
        self._status_timer.stop()

    def set_poll_interval(self, interval_ms: int):
        self._poll_interval_ms = max(20, min(1000, interval_ms))
        self._status_timer.setInterval(self._poll_interval_ms)

    def _auto_poll(self):
        """Timer callback — DISABLED (open-loop mode, no #P polling)."""
        pass

    # ── Command API ─────────────────────────────────────────────────

    def poll_encoders(self) -> bool:
        """Encoder polling DISABLED (open-loop mode).

        The old implementation sent #P, which reads 7 AS5600 encoders over
        slow bit-bang I2C on the master — flooding the bus and starving move
        commands. The arm is open-loop now; polling does nothing.
        """
        return False

    def poll_all(self) -> bool:
        """Request all positions — DISABLED (open-loop mode, see above)."""
        return False

    def get_status(self) -> dict:
        """Get system status (NO network query — returns cached state).

        The #P query does 7 sequential encoder reads on the master, which is
        expensive and floods the bus. Status is derived from the cached
        protocol state, updated when the user polls or sends commands.
        """
        return {
            "state": self.state.value,
            "joints": list(self.current_joints),
            "encoders": list(self.encoder_values),
            "enabled_mask": self.enabled_mask,
            "firmware": self.firmware_version,
        }

    def get_joints(self) -> list:
        return list(self.current_joints)

    # Joint index (studio 0..5) -> master joint number (1..6)
    JOINT_TO_NUMBER = [1, 2, 3, 4, 5, 6]
    # Fallback steps/deg if the config can't be read (real value comes from
    # #CFG via cfg_read(): steps_per_rev * gear_ratio / 100 / 360).
    FALLBACK_STEPS_PER_DEG = 200.0

    def _steps_per_deg(self, joint_idx):
        """Gear-aware steps/deg for a joint (1-based master joint)."""
        cfg = self.cfg_read(joint_idx)
        if cfg:
            spr = int(cfg.get("steps_per_rev", 3200))
            gr = int(cfg.get("gear_ratio", 100))
            if spr > 0 and gr > 0:
                return spr * gr / 100.0 / 360.0
        return self.FALLBACK_STEPS_PER_DEG

    def move_joints(self, positions: list, speed: float = 50.0) -> bool:
        """Move the real robot.

        The jog panel / controller send ABSOLUTE joint angles for all 6
        joints on every tick. The master firmware moves ONE joint by a
        RELATIVE step count (#MV<joint>,<speed>,<steps>). So we find the
        first joint whose commanded angle changed vs the last one, convert
        the delta to steps (gear-aware), and send a single relative move.

        ``speed`` is a percentage (1-100) of the joint's configured max_speed.
        """
        if len(positions) < 6:
            return False

        # Find the changed joint (first non-zero delta).
        joint_idx = 0
        delta_deg = 0.0
        for i in range(6):
            delta = positions[i] - self.commanded_positions[i]
            if abs(delta) > 1e-6:
                joint_idx = i
                delta_deg = delta
                break
        if abs(delta_deg) < 1e-6:
            return True  # nothing changed

        self.state = RobotState.MOVING
        self.state_changed.emit(self.state.value)

        joint_no = self.JOINT_TO_NUMBER[joint_idx]
        spd_per_deg = self._steps_per_deg(joint_no)
        # Speed: percentage (1-100) of the joint's configured max_speed.
        prof = self.cf_read(joint_no) or {}
        max_speed = int(prof.get("max_speed", 2000))
        speed_int = max(1, int(max_speed * max(1, min(100, speed)) / 100.0))
        # Preserve the SIGN of the delta: the master's #MV uses negative
        # steps to reverse direction. abs() here made every move go the
        # same way regardless of slider direction.
        steps = int(delta_deg * spd_per_deg)
        if steps == 0:
            steps = 1 if delta_deg > 0 else -1
        cmd = f"MV{joint_no},{speed_int},{steps}"
        result = self._send_command(cmd, timeout=10.0)

        self.commanded_positions = list(positions[:6])
        self.current_joints = list(positions[:6])
        self.position_updated.emit(self.current_joints)

        if result.startswith("error") or result.startswith("ERR") \
           or result.startswith(">ER"):
            self.state = RobotState.ERROR
            self.state_changed.emit(self.state.value)
            return False

        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def jog_joint(self, joint_no, delta_deg, speed_pct=50.0) -> bool:
        """Jog one joint by a RELATIVE angle via #MV.

        The studio owns the gear ratio and max_speed: it converts the delta
        degrees to raw steps using the gear-aware steps/deg (from #CFG) and
        clamps the commanded speed to the configured max_speed (from #CR).
        The robot just executes the raw move with its accel/decel profile.

        ``speed_pct`` is 1-100 (% of the joint's max_speed).
        """
        if joint_no < 1 or joint_no > 6:
            return False
        if abs(delta_deg) < 1e-9:
            return True

        spd_per_deg = self._steps_per_deg(joint_no)
        prof = self.cf_read(joint_no) or {}
        max_speed = int(prof.get("max_speed", 2000)) or 2000
        speed_int = max(1, int(max_speed * max(1, min(100, speed_pct)) / 100.0))

        steps = int(delta_deg * spd_per_deg)
        if steps == 0:
            steps = 1 if delta_deg > 0 else -1
        # #MV fields: steps is int16 (bounded), speed uint16.
        if abs(steps) > 32767:
            steps = 32767 if steps > 0 else -32767
        if speed_int > 65535:
            speed_int = 65535

        self.state = RobotState.MOVING
        self.state_changed.emit(self.state.value)
        result = self._send_command(f"MV{joint_no},{speed_int},{steps}", timeout=10.0)
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return not (result.startswith("error") or result.startswith("ERR")
                    or result.startswith(">ER"))

    def jog_start(self, joint_no, direction, speed_pct=50.0) -> bool:
        """Start CONTINUOUS jogging of one joint (#JC, hold-to-run).

        The robot moves the joint continuously in ``direction`` (+1/-1) at
        ``speed`` steps/sec until jog_stop() is called. ``direction`` is the
        sign of the motion (+1 = positive, -1 = negative).

        NOTE: we do NOT cf_read() here — every read is ~1s of SPI traffic
        and hammering it before each #JC (joystick cycling) corrupted the
        slave link (config read back zeros, then F0 failures). Use a sane
        fixed speed instead; tune via jog_speed_steps if needed.
        """
        if joint_no < 1 or joint_no > 6 or direction == 0:
            return False
        speed = int(getattr(self, "jog_speed_steps", 2000))
        if speed < 1 or speed > 65535:
            speed = 2000
        self.state = RobotState.MOVING
        self.state_changed.emit(self.state.value)
        result = self._send_command(
            f"JC{joint_no},{1 if direction > 0 else -1},{speed}", timeout=3.0)
        return not (result.startswith("error") or result.startswith("ERR")
                    or result.startswith(">ER"))

    def jog_stop(self) -> bool:
        """Halt all motion (continuous jog / any move)."""
        self._send_command("H", wait_ok=False)
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def coordinated_move(self, delta_deg: list) -> bool:
        """Send all 6 joint deltas in one #M command (time-synchronized).

        The firmware's #M<j1>,<j2>,...,<j6> computes a common duration T
        from the slowest axis and scales each joint's cruise speed so they
        all start and stop together.  This is much smoother than 6 separate
        #MV commands for multi-joint moves (world jog, Cartesian moves).

        Args:
            delta_deg: list of 6 floats (degrees), one per joint.
        """
        if len(delta_deg) < 6:
            return False
        parts = ",".join(f"{float(d):.4f}" for d in delta_deg[:6])
        self.state = RobotState.MOVING
        self.state_changed.emit(self.state.value)
        result = self._send_command(f"M{parts}", timeout=10.0)
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return not (result.startswith(">ER") or result.startswith("error"))

    def home(self) -> bool:
        self.state = RobotState.HOMING
        self.state_changed.emit(self.state.value)

        result = self._send_command("HFF", timeout=10.0)

        self.current_joints = [0.0] * 6
        self.commanded_positions = [0.0] * 6
        self.encoder_values = [0.0] * 6
        self.position_updated.emit(self.current_joints)

        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return not (result.startswith("error") or result.startswith("ERR") or result.startswith(">ER"))

    def unlock(self) -> bool:
        # Master has no dedicated unlock; enable all drivers.
        result = self._send_command("ENFF", timeout=5.0)
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def stop(self) -> bool:
        # #HFF halts all axes.
        self._send_command("HFF", wait_ok=False)
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return True

    def set_gripper(self, position: float) -> bool:
        position = max(0.0, min(1.0, position))
        # Gripper = joint 6 on the master (#MV6). Closed=+steps, open=0.
        steps = int(1000 * position)
        result = self._send_command(f"MV6,200,{steps}", timeout=5.0)
        return not (result.startswith("error") or result.startswith(">ER"))

    # ── TMC Driver Control ─────────────────────────────────────────

    def tmc_read_register(self, driver_addr: int, reg: int) -> Optional[int]:
        """Read a TMC register from a specific driver."""
        cmd = f"#TR{driver_addr:X},{reg:02X}"
        result = self._send_command(cmd, timeout=2.0)

        if result.startswith(">TR:"):
            try:
                parts = result[4:].split(",")
                if len(parts) >= 3:
                    value = int(parts[2], 16)
                    self.tmc_reg_read.emit(driver_addr, reg, value)
                    return value
            except (ValueError, IndexError):
                pass
        return None

    def tmc_write_register(self, driver_addr: int, reg: int, value: int) -> bool:
        """Write a TMC register value."""
        cmd = f"#TW{driver_addr:X},{reg:02X},{value:08X}"
        result = self._send_command(cmd, timeout=2.0)
        return result.startswith(">OK")

    def tmc_read_drv_status(self, driver_addr: int) -> Optional[dict]:
        """Read DRV_STATUS register and parse diagnostics."""
        val = self.tmc_read_register(driver_addr, 0x6F)
        if val is None:
            return None

        status = {
            "raw": val,
            "overtemp_shutdown": bool(val & 0x01),
            "overtemp_warning": bool(val & 0x02),
            "short_to_gnd": bool(val & 0x04),
            "short_to_vs_a": bool(val & 0x08),
            "short_to_vs_b": bool(val & 0x10),
            "open_load_a": bool(val & 0x20),
            "open_load_b": bool(val & 0x40),
            "temp_high": bool(val & 0x80),
            "cs_actual": (val >> 8) & 0x1F,
            "stallguard_result": (val >> 16) & 0xFFF,
            "standstill": bool(val & 0x80000000),
        }
        self.tmc_status_updated.emit(driver_addr, status)
        return status

    def tmc_configure(self, driver_addr: int, gconf: int = None,
                      chopconf: int = None, ihold_irun: int = None) -> bool:
        """Configure a TMC driver with all key registers."""
        success = True

        if gconf is not None:
            success &= self.tmc_write_register(driver_addr, 0x00, gconf)

        if ihold_irun is not None:
            success &= self.tmc_write_register(driver_addr, 0x10, ihold_irun)

        if chopconf is not None:
            success &= self.tmc_write_register(driver_addr, 0x6C, chopconf)

        return success

    def tmc_set_enabled(self, driver_addr: int, enabled: bool) -> bool:
        """Enable or disable a specific driver."""
        mask = 1 << driver_addr if enabled else 0
        cmd = f"#EN{mask:X}" if enabled else f"#DI{mask:X}"
        result = self._send_command(cmd, timeout=1.0)
        return result.startswith(">OK")

    def tmc_set_current(self, driver_addr: int, irun: int, ihold: int,
                        iholddelay: int = 4) -> bool:
        """Set current scaling (IRUN=0-31, IHOLD=0-31)."""
        val = (irun & 0x1F) | ((ihold & 0x1F) << 8) | ((iholddelay & 0x0F) << 16)
        return self.tmc_write_register(driver_addr, 0x10, val)

    def tmc_set_microsteps(self, driver_addr: int, mres: int,
                           interpolation: bool = True) -> bool:
        """Set microstep resolution.
        mres: 0=Full, 1=1/2, 2=1/4, 3=1/8, 4=1/16, 5=1/32, 6=1/64, 7=1/128, 8=1/256
        """
        # Start from default CHOPCONF and modify MRES
        chopconf = 0x00010053  # Default: TOFF=3, HSTRT=5, HEND=2, TBL=1
        chopconf |= (mres & 0x0F) << 24
        if interpolation:
            chopconf |= 1 << 28  # INTPOL
        return self.tmc_write_register(driver_addr, 0x6C, chopconf)

    def tmc_set_mode(self, driver_addr: int, spreadcycle: bool) -> bool:
        """Switch between SpreadCycle (True) and StealthChop (False)."""
        # Read current GCONF, modify EN_SPREADCYCLE bit
        gconf = self.tmc_read_register(driver_addr, 0x00)
        if gconf is None:
            gconf = 0x00C0  # Default: MSTEP_REG_SELECT + PDN_DISABLE

        if spreadcycle:
            gconf |= 0x04   # Set bit 2
        else:
            gconf &= ~0x04  # Clear bit 2

        return self.tmc_write_register(driver_addr, 0x00, gconf)

    # ── Encoder Zero Calibration ───────────────────────────────────

    def calibrate_zero(self, joint_index: int):
        """Set current encoder position as zero offset."""
        if 0 <= joint_index < len(self.encoder_values):
            self.encoder_zero_offsets[joint_index] = self.encoder_values[joint_index]

    def reset_zero_offsets(self):
        self.encoder_zero_offsets = [0.0] * 6

    def get_calibrated_encoder(self, joint_index: int) -> float:
        """Get encoder value with zero offset applied."""
        if 0 <= joint_index < len(self.encoder_values):
            raw = self.encoder_values[joint_index]
            offset = self.encoder_zero_offsets[joint_index]
            return raw - offset
        return 0.0

    # ── GRBL-style Config ($$ / $SECTION.key=value / SAVE / LOAD) ──

    # Axis names per slave — Slave 1 = J1..J3 (+ spare), Slave 2 = J4..J6 + Gripper
    SLAVE_AXIS_NAMES = {
        "slave1": ["J1", "J2", "J3", "Aux"],
        "slave2": ["J4", "J5", "J6", "Gripper"],
    }

    def cfg_read(self, joint) -> Optional[dict]:
        """Read one joint's extended config via #CFG<j>.

        Returns {joint, steps_per_rev, gear_ratio, dir_inverted} or None.
        """
        resp = self._send_command(f"CFG{int(joint)}", timeout=2.0)
        if resp.startswith(">CFG:"):
            parts = resp[5:].split(",")
            if len(parts) >= 4:
                try:
                    return {
                        "joint": int(parts[0]),
                        "steps_per_rev": int(parts[1]),
                        "gear_ratio": int(parts[2]),
                        "dir_inverted": int(parts[3]),
                    }
                except ValueError:
                    return None
        return None

    def cfg_write(self, joint, spr=None, gr=None, di=None) -> bool:
        """Write one joint's extended config via #CFG<j>,<spr>,<gr>,<di>.

        Omitted fields keep their current (read) values.
        """
        # NOTE: do NOT cfg_read() here first — every read is ~1s of SPI
        # (3 retried 0x60 reads) and the Apply&Save loop writes all 6 joints,
        # so pre-reading made joints 2-6 time out (only #CFG1 ever landed).
        # The motion-config UI already holds the values; send them directly.
        if spr is None:
            spr = 3200
        if gr is None:
            gr = 100
        if di is None:
            di = 0
        resp = self._send_command(
            f"CFG{int(joint)},{int(spr)},{int(gr)},{int(di)}", timeout=2.0)
        return resp.startswith(">OK")

    # ── Motion Profile Config (#CF / #CR / #CS) ──────────────────

    def cf_read(self, joint) -> Optional[dict]:
        """Read one joint's motion profile via #CR<j>.

        Returns {joint, max_speed, accel, decel} or None.
        """
        resp = self._send_command(f"CR{int(joint)}", timeout=2.0)
        if resp.startswith(">CR:"):
            parts = resp[4:].split(",")
            if len(parts) >= 4:
                try:
                    return {
                        "joint": int(parts[0]),
                        "max_speed": int(parts[1]),
                        "accel": int(parts[2]),
                        "decel": int(parts[3]),
                    }
                except ValueError:
                    return None
        return None

    def cf_write(self, joint, max_speed=None, accel=None, decel=None,
                 jog_accel=None, jog_decel=None) -> bool:
        """Write one joint's motion profile via #CF<j>,<max>,<accel>,<decel>[,<jog_decel>[,<jog_accel>]].

        Omitted fields keep their current (read) values.
        """
        cur = self.cf_read(joint) or {}
        if max_speed is None:
            max_speed = cur.get("max_speed", 2000)
        if accel is None:
            accel = cur.get("accel", 500)
        if decel is None:
            decel = cur.get("decel", accel)
        # The firmware's #CF takes two optional jog fields:
        #   #CF<j>,<max>,<accel>,<decel>,<jog_decel>,<jog_accel>
        # Only append them when the caller provided them (defaults to keep
        # current values on the robot, per the firmware's -1 sentinel).
        cmd = f"CF{int(joint)},{int(max_speed)},{int(accel)},{int(decel)}"
        if jog_decel is not None:
            cmd += f",{int(jog_decel)}"
            if jog_accel is not None:
                cmd += f",{int(jog_accel)}"
        resp = self._send_command(cmd, timeout=2.0)
        return resp.startswith(">OK")

    def cs_save(self) -> bool:
        """Persist config on both slaves via #CS (0x47 save frame)."""
        resp = self._send_command("CS", timeout=5.0)
        return resp.startswith(">OK")

    # ── Homing & Limits ─────────────────────────────────────────────

    def read_limits(self) -> Optional[list]:
        """Read all 6 limit-switch states via #L.

        Returns a list of 6 booleans (True = triggered) or None on failure.
        """
        resp = self._send_command("L", timeout=2.0)
        if resp.startswith(">L:"):
            try:
                bits = int(resp[3:].strip())
            except ValueError:
                return None
            return [(bits >> i) & 1 == 1 for i in range(6)]
        return None

    def home_joint(self, joint: int) -> bool:
        """Home a single joint (1-6) via #HM<j>. 0 homes all."""
        if joint < 0 or joint > 6:
            return False
        resp = self._send_command(f"HM{int(joint)}", timeout=5.0)
        return resp.startswith(">OK")

    def home_all(self) -> bool:
        """Home all joints via #HM0."""
        return self.home_joint(0)

    def home_status(self, joint: int) -> Optional[dict]:
        """Query homing status for a joint via #HQ<j>.

        Returns {joint, homed, state} or None.
        """
        if joint < 1 or joint > 6:
            return None
        resp = self._send_command(f"HQ{int(joint)}", timeout=2.0)
        if resp.startswith(">HQ:"):
            parts = resp[4:].split(",")
            if len(parts) >= 3:
                try:
                    return {
                        "joint": int(parts[0]),
                        "homed": int(parts[1]) == 1,
                        "state": int(parts[2]),
                    }
                except ValueError:
                    return None
        return None

    def home_set_config(self, joint, search_speed=None, creep_speed=None,
                        backoff_steps=None, home_offset=None,
                        invert_limit=None, invert_dir=None) -> bool:
        """Set homing config for a joint via #HC<j>,<search>,<creep>,<backoff>,<offset>,<invlim>,<invd>.

        Omitted fields keep their current values.
        """
        cur = self._home_cfg_cache.get(int(joint), {})
        search_speed = search_speed if search_speed is not None else cur.get("search_speed", 1000)
        creep_speed = creep_speed if creep_speed is not None else cur.get("creep_speed", 100)
        backoff_steps = backoff_steps if backoff_steps is not None else cur.get("backoff_steps", 200)
        home_offset = home_offset if home_offset is not None else cur.get("home_offset", 0)
        invert_limit = int(invert_limit) if invert_limit is not None else int(cur.get("invert_limit", True))
        invert_dir = int(invert_dir) if invert_dir is not None else int(cur.get("invert_dir", False))
        resp = self._send_command(
            f"HC{int(joint)},{int(search_speed)},{int(creep_speed)},"
            f"{int(backoff_steps)},{int(home_offset)},{invert_limit},{invert_dir}",
            timeout=2.0)
        ok = resp.startswith(">OK")
        if ok:
            self._home_cfg_cache[int(joint)] = {
                "search_speed": int(search_speed),
                "creep_speed": int(creep_speed),
                "backoff_steps": int(backoff_steps),
                "home_offset": int(home_offset),
                "invert_limit": bool(invert_limit),
                "invert_dir": bool(invert_dir),
            }
        return ok

    def home_read_config(self, joint) -> Optional[dict]:
        """Read homing config for a joint via #HG<j> (guarded read).

        Returns a dict or None if the master doesn't support it yet.
        """
        resp = self._send_command(f"HG{int(joint)}", timeout=2.0)
        if resp.startswith(">HG:"):
            parts = resp[4:].split(",")
            if len(parts) >= 7:
                try:
                    cfg = {
                        "joint": int(parts[0]),
                        "search_speed": int(parts[1]),
                        "creep_speed": int(parts[2]),
                        "backoff_steps": int(parts[3]),
                        "home_offset": int(parts[4]),
                        "invert_limit": bool(int(parts[5])),
                        "invert_dir": bool(int(parts[6])),
                    }
                    self._home_cfg_cache[int(joint)] = cfg
                    return cfg
                except ValueError:
                    return None
        return None

    # ── Command Send Override ──────────────────────────────────────

    def _send_command(self, cmd: str, wait_ok: bool = True,
                      timeout: float = 5.0) -> str:
        if not self.transport.is_connected():
            return ""

        with self._lock:
            self.transport.reset_buffer()
            # The master firmware executes commands that start with '#'
            # (e.g. #PING, #P, #MV4,200,1000, #HFF). Add it if missing.
            if not cmd.startswith("#"):
                cmd = "#" + cmd
            data = (cmd.strip() + "\n").encode("ascii")
            if not self.transport.send(data):
                return ""
            self.command_sent.emit(cmd.strip())

            if not wait_ok:
                return ""

            deadline = time.time() + timeout
            full_response = ""
            while time.time() < deadline:
                line = self.transport.read_line(timeout=0.05)
                if line:
                    decoded = line.decode("ascii", errors="replace").strip()
                    self.message_received.emit(decoded)
                    full_response += decoded + "\n"

                    if decoded.startswith("#"):
                        continue  # legacy command echo
                    if decoded.startswith("$"):
                        continue  # config echo / dump lines

                    if decoded.startswith(">"):
                        self._process_response(decoded)

                    # Firmware replies (nite_master / nite_servo)
                    if decoded.startswith("POS "):
                        self._process_response(decoded)
                    if decoded.startswith("MODE "):
                        self.firmware_version = decoded
                    if decoded == "OK" or decoded == "DONE":
                        return decoded
                    if decoded.startswith("ERR"):
                        self.error_occurred.emit(decoded)
                        return decoded

                    # Legacy read responses — any ">XXX:" prefixed line is
                    # a complete reply (e.g. >CFG:, >HG:, >L:, >TR:, >T:).
                    if decoded.startswith(">"):
                        return decoded

            return full_response if full_response else ""

    def _process_response(self, response: str):
        """Parse incoming response lines and emit signals.

        Handles both the legacy ``>P:/>E:/...`` format and the firmware's
        ``POS <j1> <j2> ...`` / ``MODE USB|LAN`` replies.
        """
        try:
            if response.startswith("POS "):
                # Firmware: POS j1 j2 j3 j4 j5 j6 j7 j8  (milli-units)
                vals = response[4:].split()
                nums = [float(v) / 1000.0 for v in vals if v.lstrip("-").replace(".", "").isdigit()]
                if len(nums) >= 6:
                    self.current_joints = nums[:6]
                    self.commanded_positions = nums[:6]
                    self.position_updated.emit(self.current_joints)
                    self.encoder_values = nums[:6]
                    self.encoder_updated.emit(self.encoder_values)
                return

            if response.startswith(">P:"):
                # Positions + Encoders: >P:<j1>,...,<j6>|<e1>,...,<e6>
                data = response[3:]
                if "|" in data:
                    pos_part, enc_part = data.split("|", 1)
                    pos_vals = [float(v) for v in pos_part.split(",") if v]
                    enc_vals = [float(v) for v in enc_part.split(",") if v]
                    if len(pos_vals) >= 6:
                        self.current_joints = pos_vals[:6]
                        self.commanded_positions = pos_vals[:6]
                        self.position_updated.emit(self.current_joints)
                    if len(enc_vals) >= 6:
                        self.encoder_values = enc_vals[:6]
                        self.encoder_updated.emit(self.encoder_values)

            elif response.startswith(">E:"):
                # Encoders only: >E:<e1>,<e2>,...,<e6>
                enc_str = response[3:]
                if enc_str:
                    enc_vals = [float(v) for v in enc_str.split(",") if v]
                    if len(enc_vals) >= 6:
                        self.encoder_values = enc_vals[:6]
                        self.encoder_updated.emit(self.encoder_values)

            elif response.startswith(">S:"):
                # Status: >S:<state>,<flags>
                data = response[3:]
                parts = data.split(",")
                if len(parts) >= 2:
                    state_str = parts[0].strip().lower()
                    flags = int(parts[1].strip(), 16)
                    state_map = {
                        "idle": RobotState.IDLE,
                        "moving": RobotState.MOVING,
                        "homing": RobotState.HOMING,
                        "error": RobotState.ERROR,
                        "alarm": RobotState.ALARM,
                    }
                    self.state = state_map.get(state_str, RobotState.UNKNOWN)
                    self.enabled_mask = flags
                    self.state_changed.emit(self.state.value)
                    self.system_state_changed.emit(self.state.value, flags)

            elif response.startswith(">V:"):
                version = response[3:].strip()
                self.firmware_version = version
                self.version_received.emit(version)

            elif response.startswith(">ER:"):
                error_msg = response[4:].strip()
                self.error_occurred.emit(error_msg)

        except (ValueError, IndexError) as e:
            self.message_received.emit(f"Protocol parse warning: {e} | raw: {response[:60]}")

    def run_gcode(self, gcode: str) -> bool:
        """Run gcode via the Nite bridge.
        Note: Nite firmware must implement the #G command to accept G-code.
        If unimplemented, this will return False.
        """
        lines = gcode.strip().split("\n")
        for line in lines:
            line = line.strip()
            if not line or line.startswith(";"):
                continue
            result = self._send_command(f"#G{line}", timeout=60.0)
            if result.startswith("error") or not result:
                return False
        return True

    # ── G-code commands (Bare G/M — firmware handles them) ──────────

    def gcode_home(self) -> bool:
        """Real homing via G28 (polls both slaves until done)."""
        self.state = RobotState.HOMING
        self.state_changed.emit(self.state.value)
        result = self._send_command("G28", timeout=120.0)
        self.state = RobotState.IDLE
        self.state_changed.emit(self.state.value)
        return not (result.startswith(">ER") or result.startswith("error"))

    def gcode_set_position(self, j1=None, j2=None, j3=None,
                           j4=None, j5=None, j6=None) -> bool:
        """Set position offset via G92 (no move, just resets cmd_pos)."""
        parts = []
        labels = ["X", "Y", "Z", "A", "B", "C"]
        vals = [j1, j2, j3, j4, j5, j6]
        for label, val in zip(labels, vals):
            if val is not None:
                parts.append(f"{label}{float(val):.4f}")
        if not parts:
            return False
        gcode = "G92 " + " ".join(parts)
        result = self._send_command(gcode, timeout=5.0)
        return not (result.startswith(">ER") or result.startswith("error"))

    def gcode_read_encoders(self) -> Optional[list]:
        """Read actual encoder positions via M114.

        Returns [j1..j6] encoder degrees, or None on failure.
        """
        result = self._send_command("M114", timeout=5.0)
        if result.startswith(">M114:"):
            try:
                parts = result[6:].split(",")
                return [float(v) for v in parts[:6]]
            except (ValueError, IndexError):
                return None
        return None

    def gcode_report_config(self) -> str:
        """Report current G-code config via M503."""
        result = self._send_command("M503", timeout=5.0)
        return result

    # ── Real-time Position Streaming ─────────────────────────────────

    def rt_streaming(self, hz: int = 0) -> str:
        """Start/stop real-time position streaming.

        Args:
            hz: 0 = off, 1-200 = auto-send position at N Hz.

        Returns:
            Response string (e.g. >RT:10 or >RT:OFF).
        """
        hz = max(0, min(200, int(hz)))
        result = self._send_command(f"RT{hz}", timeout=3.0)
        return result

    # ── Waypoints ────────────────────────────────────────────────────

    def waypoint_save(self, name: str) -> bool:
        """Save current position as a named waypoint."""
        result = self._send_command(f"WPS{name}", timeout=5.0)
        return result.startswith(">OK")

    def waypoint_move(self, name: str) -> bool:
        """Move to a saved waypoint."""
        result = self._send_command(f"WPM{name}", timeout=30.0)
        return not (result.startswith(">ER") or result.startswith("error"))

    def waypoint_list(self) -> Optional[list]:
        """List all saved waypoints.

        Returns list of dicts [{name, j1..j6}, ...] or None.
        """
        result = self._send_command("WPL", timeout=3.0)
        if result.startswith(">WPL:"):
            waypoints = []
            raw = result[5:].strip()
            if raw:
                for entry in raw.split("|"):
                    parts = entry.split(",")
                    if len(parts) >= 7:
                        try:
                            waypoints.append({
                                "name": parts[0],
                                "j1": float(parts[1]),
                                "j2": float(parts[2]),
                                "j3": float(parts[3]),
                                "j4": float(parts[4]),
                                "j5": float(parts[5]),
                                "j6": float(parts[6]),
                            })
                        except (ValueError, IndexError):
                            pass
            return waypoints
        return None

    def waypoint_delete(self, name: str) -> bool:
        """Delete a saved waypoint."""
        result = self._send_command(f"WPD{name}", timeout=3.0)
        return result.startswith(">OK")

    # ── Macros (Teach-and-Repeat) ────────────────────────────────────

    def macro_record_start(self) -> bool:
        """Start recording a macro."""
        result = self._send_command("MACR", timeout=3.0)
        return result.startswith(">OK")

    def macro_record_stop(self) -> bool:
        """Stop recording a macro."""
        result = self._send_command("MACS", timeout=3.0)
        return result.startswith(">OK")

    def macro_play(self) -> bool:
        """Replay the recorded macro."""
        result = self._send_command("MACP", timeout=60.0)
        return result.startswith(">OK") or "MAC_DONE" in result

    def macro_list(self) -> Optional[list]:
        """List recorded macro steps.

        Returns list of strings (raw commands) or None.
        """
        result = self._send_command("MACL", timeout=3.0)
        if result.startswith(">MACL:"):
            raw = result[6:].strip()
            if raw:
                return raw.split("|")
            return []
        return None

    def macro_clear(self) -> bool:
        """Clear the recorded macro."""
        result = self._send_command("MACD", timeout=3.0)
        return result.startswith(">OK")

    # ── Trajectory Buffer ────────────────────────────────────────────

    def queue_add(self, targets: dict, feed: int = 2000) -> bool:
        """Enqueue an absolute move into the trajectory buffer.

        Args:
            targets: dict of {joint_no: target_deg}, e.g. {1: 100.0, 3: -50.0}
            feed: feed rate (steps/sec)
        """
        gcode_parts = []
        labels = {1: "X", 2: "Y", 3: "Z", 4: "A", 5: "B", 6: "C"}
        for jnt, deg in targets.items():
            label = labels.get(jnt, f"J{jnt}")
            gcode_parts.append(f"{label}{float(deg):.4f}")
        gcode_parts.append(f"F{int(feed)}")
        gcode_line = " ".join(gcode_parts)
        result = self._send_command(f"QA {gcode_line}", timeout=5.0)
        return result.startswith(">OK")

    def queue_execute(self) -> bool:
        """Execute all queued moves."""
        result = self._send_command("QE", timeout=120.0)
        return result.startswith(">OK") or "DONE" in result

    def queue_halt(self) -> bool:
        """Halt trajectory execution."""
        result = self._send_command("QH", timeout=3.0)
        return result.startswith(">OK")

    def queue_clear(self) -> bool:
        """Clear the trajectory buffer."""
        result = self._send_command("QC", timeout=3.0)
        return result.startswith(">OK")

    def queue_status(self) -> Optional[dict]:
        """Query trajectory queue status.

        Returns {count, executing, tail} or None.
        """
        result = self._send_command("QS", timeout=3.0)
        if result.startswith(">QS:"):
            parts = result[4:].split(",")
            if len(parts) >= 3:
                try:
                    return {
                        "count": int(parts[0]),
                        "executing": bool(int(parts[1])),
                        "tail": int(parts[2]),
                    }
                except (ValueError, IndexError):
                    pass
        return None


# Registry for protocol adapters
PROTOCOL_REGISTRY = {
    "grbl": GRBLAdapter,
    "custom": CustomFirmwareAdapter,
    "nite": Nite369Protocol,
}


def create_protocol(name: str, transport) -> Optional[ProtocolAdapter]:
    """Create a protocol adapter by name."""
    cls = PROTOCOL_REGISTRY.get(name)
    if cls is None:
        return None
    return cls(transport)
