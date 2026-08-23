"""Transport abstraction layer for robot communication.

Provides a unified interface for Serial, Ethernet, and CAN transports.
All transports are thread-safe and emit Qt signals for state changes.
"""

import threading
import time
import socket
import logging
from enum import Enum

from PyQt5.QtCore import QObject, pyqtSignal

log = logging.getLogger("astra_studio.transports")


class TransportState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


class TransportError(Exception):
    pass


class SerialTransport(QObject):
    """pyserial-based transport with auto-detection and thread-safe I/O."""

    state_changed = pyqtSignal(str)
    data_received = pyqtSignal(bytes)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._port = None
        self._baud_rate = 115200
        self._serial = None
        self._state = TransportState.DISCONNECTED
        self._read_thread = None
        self._running = False
        self._lock = threading.Lock()
        self._buffer = bytearray()

    @property
    def state(self) -> TransportState:
        return self._state

    def _set_state(self, state: TransportState):
        self._state = state
        self.state_changed.emit(state.value)

    def connect(self, port: str = None, baud_rate: int = 115200, **kwargs) -> bool:
        if self._serial and self._serial.is_open:
            self.disconnect()

        self._set_state(TransportState.CONNECTING)
        self._baud_rate = baud_rate
        self._port = port or self._port

        try:
            import serial
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
            )
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._set_state(TransportState.CONNECTED)
            return True
        except ImportError:
            self.error_occurred.emit("The serial library isn't installed. Add it with: pip install pyserial")
            self._set_state(TransportState.ERROR)
            return False
        except Exception as e:
            log.warning("Serial connect failed on %s: %s", self._port, e)
            self.error_occurred.emit(f"Couldn't open the serial port: {e}")
            self._set_state(TransportState.ERROR)
            return False

    def disconnect(self):
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        with self._lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.close()
                except Exception as e:
                    log.debug("Serial close error: %s", e)
            self._serial = None
        self._buffer.clear()
        self._set_state(TransportState.DISCONNECTED)

    def send(self, data: bytes) -> bool:
        with self._lock:
            if not self._serial or not self._serial.is_open:
                return False
            try:
                self._serial.write(data)
                return True
            except Exception as e:
                self.error_occurred.emit(f"Failed to send data to the robot: {e}")
                return False

    def read(self, timeout=0.01) -> bytes:
        with self._lock:
            if not self._serial or not self._serial.is_open:
                return b""
            try:
                if self._serial.in_waiting > 0:
                    return self._serial.read(self._serial.in_waiting)
            except Exception as e:
                log.debug("Serial read error: %s", e)
        return b""

    def read_line(self, timeout=0.1) -> bytes:
        """Read a complete newline-terminated line from the buffer."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            idx = self._buffer.find(b"\n")
            if idx >= 0:
                line = bytes(self._buffer[: idx + 1])
                self._buffer = self._buffer[idx + 1 :]
                return line
            chunk = self.read()
            if chunk:
                self._buffer.extend(chunk)
            time.sleep(0.001)
        return b""

    def _read_loop(self):
        """Background thread for continuous reading."""
        while self._running and self._serial and self._serial.is_open:
            try:
                chunk = self.read()
                if chunk:
                    self._buffer.extend(chunk)
                    self.data_received.emit(bytes(chunk))
                else:
                    time.sleep(0.005)
            except Exception as e:
                if self._running:
                    self.error_occurred.emit(f"Failed to read data from the robot: {e}")
                break

    def is_connected(self) -> bool:
        return self._state == TransportState.CONNECTED

    def reset_buffer(self):
        with self._lock:
            self._buffer.clear()
            if self._serial and self._serial.is_open:
                self._serial.reset_input_buffer()

    @staticmethod
    def available_ports() -> list:
        try:
            from serial.tools.list_ports import comports
            ports = []
            for port in comports():
                ports.append({
                    "port": port.device,
                    "description": port.description,
                    "hwid": port.hwid,
                    "manufacturer": getattr(port, "manufacturer", ""),
                })
            return ports
        except ImportError:
            return [{"port": "COM1", "description": "pyserial not installed", "hwid": "", "manufacturer": ""}]


class EthernetTransport(QObject):
    """TCP socket transport for Ethernet-connected robot controllers."""

    state_changed = pyqtSignal(str)
    data_received = pyqtSignal(bytes)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._host = "127.0.0.1"
        self._port = 8080
        self._socket = None
        self._state = TransportState.DISCONNECTED
        self._read_thread = None
        self._running = False
        self._lock = threading.Lock()
        self._buffer = bytearray()
        self._timeout = 5.0

    @property
    def state(self) -> TransportState:
        return self._state

    def _set_state(self, state: TransportState):
        self._state = state
        self.state_changed.emit(state.value)

    def connect(self, host: str = None, port: int = None, timeout: float = 5.0, **kwargs) -> bool:
        if self._socket:
            self.disconnect()

        self._set_state(TransportState.CONNECTING)
        self._host = host or self._host
        self._port = port or self._port
        self._timeout = timeout

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Disable Nagle: tiny command writes (#G0 X5 F600\n) must go out
            # immediately, not wait for the W5500's delayed ACK. Without this
            # every command round-trip adds ~40-200ms of latency.
            try:
                self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass  # not supported on all platforms
            self._socket.settimeout(self._timeout)
            self._socket.connect((self._host, self._port))
            self._socket.settimeout(0.01)
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._set_state(TransportState.CONNECTED)
            return True
        except socket.timeout:
            self.error_occurred.emit(f"The robot at {self._host}:{self._port} didn't respond in time")
            self._set_state(TransportState.ERROR)
            return False
        except ConnectionRefusedError:
            self.error_occurred.emit(f"The robot at {self._host}:{self._port} refused the connection")
            self._set_state(TransportState.ERROR)
            return False
        except Exception as e:
            log.warning("Ethernet connect failed to %s:%s: %s", self._host, self._port, e)
            self.error_occurred.emit(f"Couldn't connect over Ethernet: {e}")
            self._set_state(TransportState.ERROR)
            return False

    def disconnect(self):
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        with self._lock:
            if self._socket:
                try:
                    self._socket.shutdown(socket.SHUT_RDWR)
                except Exception as e:
                    log.debug("Socket shutdown error: %s", e)
                try:
                    self._socket.close()
                except Exception as e:
                    log.debug("Socket close error: %s", e)
            self._socket = None
        self._buffer.clear()
        self._set_state(TransportState.DISCONNECTED)

    def send(self, data: bytes) -> bool:
        with self._lock:
            if not self._socket:
                return False
            try:
                self._socket.sendall(data)
                return True
            except (OSError, ConnectionError) as e:
                # The remote closed/refused the connection. Mark it
                # disconnected and emit ONE error (not one per retry) so the
                # UI doesn't get flooded with "Failed to send" spam.
                log.warning("Ethernet send failed (marking disconnected): %s", e)
                try:
                    self._socket.close()
                except OSError:
                    pass
                self._socket = None
                self._set_state(TransportState.DISCONNECTED)
                self.error_occurred.emit("Connection to the robot was lost")
                return False
            except Exception as e:
                self.error_occurred.emit(f"Failed to send data to the robot: {e}")
                return False

    def read(self, timeout=0.01) -> bytes:
        with self._lock:
            if not self._socket:
                return b""
            try:
                self._socket.settimeout(timeout)
                data = self._socket.recv(4096)
                return data if data else b""
            except socket.timeout:
                return b""
            except (OSError, ConnectionError) as e:
                # Remote closed the connection -> mark disconnected once.
                log.debug("Socket read error: %s", e)
                try:
                    self._socket.close()
                except OSError:
                    pass
                self._socket = None
                self._set_state(TransportState.DISCONNECTED)
                return b""
            except Exception as e:
                log.debug("Socket read error: %s", e)
                return b""

    def read_line(self, timeout=0.1) -> bytes:
        deadline = time.time() + timeout
        while time.time() < deadline:
            idx = self._buffer.find(b"\n")
            if idx >= 0:
                line = bytes(self._buffer[: idx + 1])
                self._buffer = self._buffer[idx + 1 :]
                return line
            chunk = self.read()
            if chunk:
                self._buffer.extend(chunk)
            time.sleep(0.001)
        return b""

    def _read_loop(self):
        while self._running and self._socket:
            try:
                chunk = self.read()
                if chunk:
                    self._buffer.extend(chunk)
                    self.data_received.emit(bytes(chunk))
                else:
                    time.sleep(0.005)
            except Exception as e:
                if self._running:
                    self.error_occurred.emit(f"Failed to read data from the robot: {e}")
                break

    def is_connected(self) -> bool:
        return self._state == TransportState.CONNECTED

    def reset_buffer(self):
        with self._lock:
            self._buffer.clear()

    @staticmethod
    def available_ports() -> list:
        return [{"port": "Custom Ethernet", "description": "TCP/IP Connection", "hwid": "", "manufacturer": ""}]
