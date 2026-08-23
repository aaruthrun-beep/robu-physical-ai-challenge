"""Unified connection manager for robot communication.

Orchestrates transport and protocol layers, providing a single
interface for connecting to, communicating with, and disconnecting
from robot hardware.

All blocking protocol I/O runs on a background worker thread so the Qt
UI never freezes on slow/unresponsive robot replies (the old code polled
status on the UI timer thread, blocking for seconds on every tick).
"""

import time
import logging
from enum import Enum

from typing import Optional
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QTimer

from .transports import SerialTransport, EthernetTransport, TransportState
from .protocols import GRBLAdapter, CustomFirmwareAdapter, Nite369Protocol, RobotState, create_protocol

log = logging.getLogger("astra_studio.connection")


class ConnectionMode(Enum):
    SIMULATION = "simulation"
    OPEN_LOOP = "open_loop"
    CLOSED_LOOP = "closed_loop"


class ProtocolWorker(QThread):
    """Background worker: runs blocking protocol calls off the UI thread.

    The UI posts a job (a callable that does socket I/O) via request(), and
    the worker runs it on its own thread, emitting the result back through
    Qt signals. This keeps the UI responsive even when the robot is slow.
    """

    job_done = pyqtSignal(object)      # result payload
    job_failed = pyqtSignal(str)       # error message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._jobs = []
        self._lock = __import__("threading").Lock()
        self._wake = __import__("threading").Event()
        self._stop = False

    def request(self, fn, *args, **kwargs):
        """Queue a callable to run on the worker thread."""
        with self._lock:
            self._jobs.append((fn, args, kwargs))
        self._wake.set()

    def run(self):
        while not self._stop:
            self._wake.wait(timeout=0.2)
            self._wake.clear()
            while True:
                with self._lock:
                    if not self._jobs:
                        break
                    fn, args, kwargs = self._jobs.pop(0)
                try:
                    result = fn(*args, **kwargs)
                    self.job_done.emit(result)
                except Exception as e:  # noqa: BLE001
                    self.job_failed.emit(str(e))

    def stop(self):
        self._stop = True
        self._wake.set()
        self.wait(3000)


class ConnectionManager(QObject):
    """Unified connection interface — the brain of communication."""

    connectionStateChanged = pyqtSignal(str)
    messageReceived = pyqtSignal(str)
    errorOccurred = pyqtSignal(str)
    statusUpdate = pyqtSignal(dict)
    modeChanged = pyqtSignal(str)
    encoderUpdate = pyqtSignal(list)        # Nite encoder data
    tmcStatusUpdate = pyqtSignal(int, dict)  # Nite TMC status
    tmcRegisterRead = pyqtSignal(int, int, int)  # addr, reg, value
    commandSent = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.transport = None
        self.protocol = None
        self.mode = ConnectionMode.SIMULATION
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._poll_status)
        self._connected_port = ""
        # Background worker for all blocking protocol I/O (keeps UI live).
        self._worker = ProtocolWorker(self)
        self._worker.job_done.connect(self._on_worker_done)
        self._worker.job_failed.connect(self._on_worker_failed)
        self._worker.start()

    @property
    def is_connected(self) -> bool:
        return self.transport is not None and self.transport.is_connected()

    @property
    def nite_protocol(self) -> Optional[Nite369Protocol]:
        """Get the protocol as Nite369Protocol if that's what's active."""
        if isinstance(self.protocol, Nite369Protocol):
            return self.protocol
        return None

    def connect_grbl(self, port: str, baud_rate: int = 115200) -> bool:
        self.disconnect()

        self.transport = SerialTransport(self)
        self._wire_transport_signals()

        self.protocol = GRBLAdapter(self.transport, self)
        self._wire_protocol_signals()

        success = self.transport.connect(port=port, baud_rate=baud_rate)
        if success:
            self.mode = ConnectionMode.OPEN_LOOP
            self._connected_port = port
            self.protocol.connect_init()
            self.connectionStateChanged.emit("connected")
            self.modeChanged.emit(self.mode.value)
            self._start_status_polling()
            self.messageReceived.emit(f"Connected to GRBL on {port}")
        else:
            self.errorOccurred.emit(f"Couldn't open serial port {port} — check the connection and try again")
            self.transport = None
            self.protocol = None
        return success

    def connect_ethernet(self, host: str, port: int, timeout: float = 5.0) -> bool:
        self.disconnect()

        self.transport = EthernetTransport(self)
        self._wire_transport_signals()

        self.protocol = GRBLAdapter(self.transport, self)
        self._wire_protocol_signals()

        success = self.transport.connect(host=host, port=port, timeout=timeout)
        if success:
            self.mode = ConnectionMode.CLOSED_LOOP
            self._connected_port = f"{host}:{port}"
            self.connectionStateChanged.emit("connected")
            self.modeChanged.emit(self.mode.value)
            self._start_status_polling()
            self.messageReceived.emit(f"Connected to {host}:{port}")
        else:
            self.errorOccurred.emit(f"Couldn't reach the robot at {host}:{port} — check the network and try again")
            self.transport = None
            self.protocol = None
        return success

    def connect_nite_ethernet(self, host: str, port: int = 8765, timeout: float = 5.0) -> bool:
        """Connect to Nite 369 firmware via Ethernet."""
        self.disconnect()

        self.transport = EthernetTransport(self)
        self._wire_transport_signals()

        self.protocol = Nite369Protocol(self.transport, self)
        self._wire_protocol_signals()

        success = self.transport.connect(host=host, port=port, timeout=timeout)
        if success:
            self.mode = ConnectionMode.CLOSED_LOOP
            self._connected_port = f"{host}:{port}"
            self.protocol.connect_init()
            self.connectionStateChanged.emit("connected")
            self.modeChanged.emit("nite_ethernet")
            self._start_status_polling()
            self.messageReceived.emit(f"Connected to Nite 369 at {host}:{port}")
        else:
            self.errorOccurred.emit(f"Couldn't reach Nite 369 at {host}:{port} — check the network and try again")
            # Clean up failed transport
            self.protocol = None
            if self.transport:
                self.transport.deleteLater()
            self.transport = None
        return success

    def connect_nite_serial(self, port: str, baud_rate: int = 115200) -> bool:
        """Connect to Nite 369 firmware via Serial."""
        self.disconnect()

        self.transport = SerialTransport(self)
        self._wire_transport_signals()

        self.protocol = Nite369Protocol(self.transport, self)
        self._wire_protocol_signals()

        success = self.transport.connect(port=port, baud_rate=baud_rate)
        if success:
            self.mode = ConnectionMode.CLOSED_LOOP
            self._connected_port = port
            self.protocol.connect_init()
            self.connectionStateChanged.emit("connected")
            self.modeChanged.emit("nite_serial")
            self._start_status_polling()
            self.messageReceived.emit(f"Connected to Nite 369 on {port} @ {baud_rate} baud")
        else:
            self.errorOccurred.emit(f"Couldn't open serial port {port} — check the connection and try again")
            # Clean up failed transport
            self.protocol = None
            if self.transport:
                self.transport.deleteLater()
            self.transport = None
        return success

    def connect_custom(self, transport_class, protocol_name: str, **transport_kwargs) -> bool:
        self.disconnect()

        self.transport = transport_class(self)
        self._wire_transport_signals()

        success = self.transport.connect(**transport_kwargs)
        if success:
            self.protocol = create_protocol(protocol_name, self.transport)
            if self.protocol is None:
                self.protocol = CustomFirmwareAdapter(self.transport, self)
            self._wire_protocol_signals()
            self.mode = ConnectionMode.OPEN_LOOP
            self._connected_port = str(transport_kwargs)
            self.connectionStateChanged.emit("connected")
            self.modeChanged.emit(self.mode.value)
            self._start_status_polling()
            self.messageReceived.emit(f"Connected via {protocol_name} protocol")
        else:
            self.errorOccurred.emit("Couldn't establish the connection — check the settings and try again")
            self.transport = None
            self.protocol = None
        return success

    def disconnect(self):
        self._stop_status_polling()
        if self.protocol:
            self.protocol.deleteLater()
            self.protocol = None
        if self.transport:
            self.transport.disconnect()
            self.transport.deleteLater()
            self.transport = None
        self._connected_port = ""
        self.mode = ConnectionMode.SIMULATION
        self.connectionStateChanged.emit("disconnected")
        self.modeChanged.emit(self.mode.value)

    def send_command(self, cmd: str) -> str:
        if not self.is_connected or not self.protocol:
            return ""
        return self.protocol._send_command(cmd)

    def command(self, cmd: str) -> bool:
        """Send a raw command ASYNC (worker thread) — UI never blocks."""
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        c = str(cmd)
        self._worker.request(proto._send_command, c)
        return True

    # ── Nite 369 Specific Commands ──────────────────────────────────

    def poll_encoders(self) -> list:
        """Poll encoder values from Nite firmware (async on worker)."""
        nite = self.nite_protocol
        if nite:
            self._worker.request(nite.poll_encoders)
        return []

    def tmc_read_register(self, addr: int, reg: int) -> Optional[int]:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_read_register(addr, reg)
        return None

    def tmc_write_register(self, addr: int, reg: int, value: int) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_write_register(addr, reg, value)
        return False

    def tmc_read_drv_status(self, addr: int) -> Optional[dict]:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_read_drv_status(addr)
        return None

    def tmc_configure(self, addr: int, gconf=None, chopconf=None, ihold_irun=None) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_configure(addr, gconf, chopconf, ihold_irun)
        return False

    def tmc_set_current(self, addr: int, irun: int, ihold: int) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_set_current(addr, irun, ihold)
        return False

    def tmc_set_microsteps(self, addr: int, mres: int) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_set_microsteps(addr, mres)
        return False

    def tmc_set_mode(self, addr: int, spreadcycle: bool) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_set_mode(addr, spreadcycle)
        return False

    def tmc_set_enabled(self, addr: int, enabled: bool) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.tmc_set_enabled(addr, enabled)
        return False

    # ── Per-joint #CFG (steps_per_rev / gear_ratio / dir_inverted) ──

    def cfg_read(self, joint) -> Optional[dict]:
        """Read one joint's extended config (#CFG<j>)."""
        nite = self.nite_protocol
        if nite:
            return nite.cfg_read(joint)
        return None

    def cfg_write(self, joint, spr=None, gr=None, di=None) -> bool:
        """Write one joint's extended config (#CFG<j>,<spr>,<gr>,<di>)."""
        nite = self.nite_protocol
        if nite:
            return nite.cfg_write(joint, spr, gr, di)
        return False

    # ── Motion Profile Config (#CF / #CR / #CS) ──────────────────

    def cf_read(self, joint):
        """Read one joint's motion profile (#CR<j>)."""
        nite = self.nite_protocol
        if nite:
            return nite.cf_read(joint)
        return None

    def cf_write(self, joint, max_speed=None, accel=None, decel=None,
                 jog_accel=None, jog_decel=None) -> bool:
        """Write one joint's motion profile (#CF<j>,<max>,<accel>,<decel>[,<jog_decel>[,<jog_accel>]])."""
        nite = self.nite_protocol
        if nite:
            return nite.cf_write(joint, max_speed, accel, decel,
                                 jog_accel=jog_accel, jog_decel=jog_decel)
        return False

    def cs_save(self) -> bool:
        """Persist config on both slaves (#CS)."""
        nite = self.nite_protocol
        if nite:
            return nite.cs_save()
        return False

    # ── Homing & Limits passthroughs ──────────────────────────────

    def read_limits(self):
        """Read all 6 limit-switch states."""
        nite = self.nite_protocol
        if nite:
            return nite.read_limits()
        return None

    def home_joint(self, joint) -> bool:
        """Home a single joint (1-6)."""
        nite = self.nite_protocol
        if nite:
            return nite.home_joint(joint)
        return False

    def home_all(self) -> bool:
        """Home all joints."""
        nite = self.nite_protocol
        if nite:
            return nite.home_all()
        return False

    def home_status(self, joint):
        """Query homing status for a joint."""
        nite = self.nite_protocol
        if nite:
            return nite.home_status(joint)
        return None

    def home_set_config(self, joint, search_speed=None, creep_speed=None,
                        backoff_steps=None, home_offset=None,
                        invert_limit=None, invert_dir=None) -> bool:
        """Set homing config for a joint."""
        nite = self.nite_protocol
        if nite:
            return nite.home_set_config(joint, search_speed, creep_speed,
                                        backoff_steps, home_offset,
                                        invert_limit, invert_dir)
        return False

    def home_read_config(self, joint):
        """Read homing config for a joint."""
        nite = self.nite_protocol
        if nite:
            return nite.home_read_config(joint)
        return None

    def save_robot_config(self) -> bool:
        """Persist config to robot flash via M500."""
        nite = self.nite_protocol
        if nite:
            resp = nite._send_command("M500", timeout=5.0)
            return resp.startswith(">OK")
        return False

    def load_robot_config(self) -> bool:
        """Reload config from robot flash via M501."""
        nite = self.nite_protocol
        if nite:
            resp = nite._send_command("M501", timeout=5.0)
            return resp.startswith(">OK")
        return False

    def set_poll_interval(self, ms: int):
        nite = self.nite_protocol
        if nite:
            nite.set_poll_interval(ms)

    def start_auto_poll(self):
        nite = self.nite_protocol
        if nite:
            nite.start_auto_poll()

    def stop_auto_poll(self):
        nite = self.nite_protocol
        if nite:
            nite.stop_auto_poll()

    # ── G-code commands ──────────────────────────────────────────────

    def gcode_home(self) -> bool:
        """Real homing via G28."""
        nite = self.nite_protocol
        if nite:
            return nite.gcode_home()
        return False

    def gcode_set_position(self, j1=None, j2=None, j3=None,
                           j4=None, j5=None, j6=None) -> bool:
        """Set position offset via G92."""
        nite = self.nite_protocol
        if nite:
            return nite.gcode_set_position(j1, j2, j3, j4, j5, j6)
        return False

    def gcode_read_encoders(self):
        """Read encoder positions via M114."""
        nite = self.nite_protocol
        if nite:
            return nite.gcode_read_encoders()
        return None

    def gcode_report_config(self) -> str:
        """Report config via M503."""
        nite = self.nite_protocol
        if nite:
            return nite.gcode_report_config()
        return ""

    # ── Real-time streaming ──────────────────────────────────────────

    def rt_streaming(self, hz: int = 0) -> str:
        """Start/stop real-time position streaming."""
        nite = self.nite_protocol
        if nite:
            return nite.rt_streaming(hz)
        return ""

    # ── Waypoints ────────────────────────────────────────────────────

    def waypoint_save(self, name: str) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.waypoint_save(name)
        return False

    def waypoint_move(self, name: str) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.waypoint_move(name)
        return False

    def waypoint_list(self):
        nite = self.nite_protocol
        if nite:
            return nite.waypoint_list()
        return None

    def waypoint_delete(self, name: str) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.waypoint_delete(name)
        return False

    # ── Macros ───────────────────────────────────────────────────────

    def macro_record_start(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.macro_record_start()
        return False

    def macro_record_stop(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.macro_record_stop()
        return False

    def macro_play(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.macro_play()
        return False

    def macro_list(self):
        nite = self.nite_protocol
        if nite:
            return nite.macro_list()
        return None

    def macro_clear(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.macro_clear()
        return False

    # ── Trajectory buffer ────────────────────────────────────────────

    def queue_add(self, targets: dict, feed: int = 2000) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.queue_add(targets, feed)
        return False

    def queue_execute(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.queue_execute()
        return False

    def queue_halt(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.queue_halt()
        return False

    def queue_clear(self) -> bool:
        nite = self.nite_protocol
        if nite:
            return nite.queue_clear()
        return False

    def queue_status(self):
        nite = self.nite_protocol
        if nite:
            return nite.queue_status()
        return None

    def move_joints(self, positions: list, speed: float = 50.0) -> bool:
        if not self.is_connected or not self.protocol:
            return False
        # Dispatch to the worker so the UI never blocks on the move I/O.
        proto = self.protocol
        pos = list(positions)
        spd = speed
        self._worker.request(proto.move_joints, pos, spd)
        return True

    def home(self) -> bool:
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        self._worker.request(proto.home)
        return True

    def unlock(self) -> bool:
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        self._worker.request(proto.unlock)
        return True

    def stop(self) -> bool:
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        self._worker.request(proto.stop)
        return True

    def set_gripper(self, position: float) -> bool:
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        pos = float(position)
        self._worker.request(proto.set_gripper, pos)
        return True

    def jog_joint(self, joint_no: int, delta_deg: float, speed_pct: float = 50.0) -> bool:
        """Jog one joint by a relative angle (studio computes steps/speed)."""
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        self._worker.request(proto.jog_joint, int(joint_no), float(delta_deg), float(speed_pct))
        return True

    def jog_start(self, joint_no: int, direction: int, speed_pct: float = 50.0) -> bool:
        """Start continuous jogging of one joint (hold-to-run)."""
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        self._worker.request(proto.jog_start, int(joint_no), int(direction), float(speed_pct))
        return True

    def jog_stop(self) -> bool:
        """Halt all motion (continuous jog / any move)."""
        if not self.is_connected or not self.protocol:
            return False
        proto = self.protocol
        self._worker.request(proto.jog_stop)
        return True

    def coordinated_move(self, delta_deg: list) -> bool:
        """Send all 6 joint deltas in one #M command (synchronous).

        Used by world jog for smooth multi-joint motion.
        """
        nite = self.nite_protocol
        if nite:
            return nite.coordinated_move(delta_deg)
        return False

    def get_joints(self) -> list:
        if self.protocol:
            return self.protocol.get_joints()
        return [0.0] * 6

    def get_status(self) -> dict:
        if self.protocol:
            return self.protocol.get_status()
        return {"state": "simulation", "joints": [0.0] * 6, "feed_rate": 0}

    def get_available_ports(self) -> list:
        if self.transport:
            return self.transport.available_ports()
        return SerialTransport.available_ports()

    def set_mode(self, mode: str):
        try:
            self.mode = ConnectionMode(mode)
            self.modeChanged.emit(self.mode.value)
        except ValueError:
            log.warning("Unknown connection mode requested: %s", mode)
            self.errorOccurred.emit(f"Unknown connection mode: {mode}")

    def _wire_transport_signals(self):
        if self.transport:
            self.transport.state_changed.connect(self._on_transport_state)
            self.transport.data_received.connect(
                lambda data: self.messageReceived.emit(data.decode("ascii", errors="replace"))
            )
            self.transport.error_occurred.connect(self.errorOccurred.emit)

    def _wire_protocol_signals(self):
        if self.protocol:
            self.protocol.state_changed.connect(self._on_protocol_state)
            self.protocol.message_received.connect(self.messageReceived.emit)
            self.protocol.error_occurred.connect(self.errorOccurred.emit)
            self.protocol.command_sent.connect(self.commandSent.emit)
            self.protocol.position_updated.connect(lambda pos: self.statusUpdate.emit(
                {"joints": pos, "state": self.protocol.state.value}
            ))
            # Wire Nite-specific signals
            if isinstance(self.protocol, Nite369Protocol):
                self.protocol.encoder_updated.connect(self.encoderUpdate.emit)
                self.protocol.tmc_status_updated.connect(self.tmcStatusUpdate.emit)
                self.protocol.tmc_reg_read.connect(self.tmcRegisterRead.emit)

    def _on_transport_state(self, state: str):
        if state == "disconnected":
            self.connectionStateChanged.emit("disconnected")
        elif state == "error":
            self.errorOccurred.emit("The connection to the robot was lost")

    def _on_protocol_state(self, state: str):
        self.statusUpdate.emit({"state": state})

    def _start_status_polling(self):
        # Status polling disabled by default: the #P query does 7 sequential
        # encoder reads on the master, which floods the bus and starves move
        # commands (the root cause of the freeze). Poll only when a panel
        # explicitly asks (e.g. encoder monitor's Start Poll button).
        pass

    def _stop_status_polling(self):
        self._status_timer.stop()

    def _poll_status(self):
        # Run the blocking status query on the worker thread so the UI
        # thread never freezes waiting on a slow robot.
        if self.is_connected and self.protocol:
            proto = self.protocol

            def _blocking_status():
                return proto.get_status()

            self._worker.request(_blocking_status)

    def _on_worker_done(self, payload):
        # Received from the worker thread -> emit on the UI thread.
        if isinstance(payload, dict):
            self.statusUpdate.emit(payload)

    def _on_worker_failed(self, err):
        log.warning("Worker job failed: %s", err)
