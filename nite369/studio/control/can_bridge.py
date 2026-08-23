import struct
import threading
import time
import logging

log = logging.getLogger("astra_studio.can")


class CANBridge:
    """Bridge between simulation and physical CAN bus / serial hardware."""

    def __init__(self, interface="virtual", channel=0):
        self.interface = interface
        self.channel = channel
        self.bus = None
        self.running = False
        self._reader_thread = None
        self._listeners = []

    def connect(self):
        try:
            import can
            self.bus = can.Bus(interface=self.interface, channel=self.channel, bitrate=1000000)
            self.running = True
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            return True
        except ImportError:
            log.warning("python-can not installed — running in virtual mode")
            return False
        except Exception as e:
            log.warning("CAN connect failed: %s — running in virtual mode", e)
            return False

    def disconnect(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def _read_loop(self):
        while self.running and self.bus:
            msg = self.bus.recv(timeout=0.1)
            if msg:
                for listener in self._listeners:
                    listener(msg)

    def send_joint_target(self, joint_index, target_steps, velocity=None):
        """Send a joint target position over CAN."""
        if not self.bus:
            return False
        try:
            import can
            data = struct.pack("<Bi", joint_index, int(target_steps))
            if velocity is not None:
                data += struct.pack("<i", int(velocity))
            msg = can.Message(
                arbitration_id=0x100 + joint_index,
                data=data,
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            log.warning("CAN send error: %s", e)
            return False

    def send_gripper(self, open=True):
        if not self.bus:
            return False
        try:
            import can
            msg = can.Message(
                arbitration_id=0x200,
                data=b"\x01" if open else b"\x00",
                is_extended_id=False,
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            log.warning("CAN gripper error: %s", e)
            return False

    def add_listener(self, callback):
        self._listeners.append(callback)
