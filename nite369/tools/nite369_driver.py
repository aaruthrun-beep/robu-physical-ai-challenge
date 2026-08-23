#!/usr/bin/env python3
"""
RoboDK Driver for Nite369 Robot Controller
==========================================
Translates RoboDK commands to Nite369 protocol over serial/TCP.

Usage (RoboDK):
  Place this file in C:/RoboDK/api/Robot/ or set driver path in RoboDK.

Usage (Console test):
  python nite369_driver.py [COM3|192.168.1.100] [115200|23]

Nite369 Protocol:
  #M<j1>,<j2>,<j3>,<j4>,<j5>,<j6>  Move joints (steps)
  #P                              Get positions (steps)
  #H                              Halt all
  #EN<mask>                       Enable motors
  #DI<mask>                       Disable motors
  #LED<r>,<g>,<b>                 Set LED color
"""

import sys
import time
import serial
import socket
import struct
from threading import Thread, Event

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DRIVER_VERSION = "Nite369 RoboDK Driver v1.0"
BAUD_RATE = 115200
TCP_PORT = 23
SERIAL_TIMEOUT = 0.1
STEP_PER_DEG = [
    200.0 * 16 / 360,   # J1: 200steps * 16 microsteps / 360deg = 8.889 steps/deg
    200.0 * 16 / 360,   # J2: Shoulder
    200.0 * 16 / 360,   # J3: Elbow
    200.0 * 16 / 360,   # J4: Wrist roll
    200.0 * 16 / 360,   # J5: Wrist pitch
    200.0 * 16 / 360,   # J6: Wrist yaw
]
N_AXES = 6

# Status constants for RoboDK
ROBOTCOM_UNKNOWN = -1000
ROBOTCOM_CONNECTION_PROBLEMS = -3
ROBOTCOM_DISCONNECTED = -2
ROBOTCOM_NOT_CONNECTED = -1
ROBOTCOM_READY = 0
ROBOTCOM_WORKING = 1
ROBOTCOM_WAITING = 2

# ---------------------------------------------------------------------------
# Communication classes
# ---------------------------------------------------------------------------
class SerialConnection:
    """Serial (USB) connection to Master Pico."""
    def __init__(self):
        self.ser = None

    def connect(self, port, baud=BAUD_RATE):
        try:
            self.ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
            time.sleep(2)  # Wait for Pico reset
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            print_message(f"Serial connect failed: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        return True

    def send(self, data):
        if self.ser and self.ser.is_open:
            self.ser.write((data + "\n").encode())
            return True
        return False

    def recv(self, timeout=2.0):
        if not self.ser:
            return None
        self.ser.timeout = timeout
        line = self.ser.readline().decode().strip()
        return line if line else None

    def flush(self):
        if self.ser:
            self.ser.reset_input_buffer()


class TCPConnection:
    """TCP/IP connection (for Ethernet W5500 or serial-to-Ethernet bridge)."""
    def __init__(self):
        self.sock = None

    def connect(self, ip, port=TCP_PORT):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
            self.sock.connect((ip, port))
            self.sock.settimeout(0.1)
            return True
        except Exception as e:
            print_message(f"TCP connect failed: {e}")
            return False

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        return True

    def send(self, data):
        if self.sock:
            try:
                self.sock.sendall((data + "\n").encode())
                return True
            except OSError:
                return False
        return False

    def recv(self, timeout=2.0):
        if not self.sock:
            return None
        self.sock.settimeout(timeout)
        try:
            data = b""
            while True:
                chunk = self.sock.recv(1)
                if not chunk:
                    break
                if chunk == b"\n":
                    break
                data += chunk
            return data.decode().strip() if data else None
        except socket.timeout:
            return None
        except OSError:
            return None

    def flush(self):
        if self.sock:
            try:
                self.sock.settimeout(0.01)
                while True:
                    self.sock.recv(4096)
            except (socket.timeout, OSError):
                pass


# ---------------------------------------------------------------------------
# Nite369 Protocol Interface
# ---------------------------------------------------------------------------
class Nite369Controller:
    """Translates between RoboDK commands and Nite369 protocol."""
    def __init__(self):
        self.conn = None
        self.connected = False
        self.joints = [0.0] * N_AXES
        self.speed_linear = 100.0   # mm/s
        self.speed_joint = 50.0     # deg/s
        self.lock = None

    def connect(self, port_or_ip, port_num=None):
        """Connect via serial or TCP."""
        self.lock = Event()
        self.lock.set()

        # Determine connection type
        if port_or_ip.upper().startswith("COM") or "/" in port_or_ip:
            # Serial connection
            self.conn = SerialConnection()
            baud = int(port_num) if port_num else BAUD_RATE
            ok = self.conn.connect(port_or_ip, baud)
        else:
            # TCP connection
            self.conn = TCPConnection()
            tcp_port = int(port_num) if port_num else TCP_PORT
            ok = self.conn.connect(port_or_ip, tcp_port)

        if ok:
            self.connected = True
            # Flush any pending data
            self.conn.flush()
            # Request version to verify connection
            resp = self.send_command("#V", timeout=2.0)
            if resp:
                print_message(f"Connected: {resp}")
            # Enable all motors
            self.send_command("#EN63")  # bits 0-5 = 0b00111111 = 63
            # Query initial position
            self._update_joints()
            return True
        return False

    def disconnect(self):
        if self.conn:
            # Disable motors before disconnect
            self.send_command("#DI63")
            self.conn.disconnect()
        self.connected = False
        return True

    def send_command(self, cmd, timeout=2.0):
        """Send Nite369 command and return response."""
        if not self.connected or not self.conn:
            return None
        self.conn.flush()
        self.conn.send(cmd)
        return self.conn.recv(timeout)

    def _update_joints(self):
        """Read current joint positions from robot."""
        resp = self.send_command("#P", timeout=2.0)
        if resp and resp.startswith("P:"):
            try:
                # Parse "P:j1,j2,j3,j4,j5,j6" (in steps)
                vals = resp[2:].split(",")
                if len(vals) >= N_AXES:
                    for i in range(N_AXES):
                        steps = int(vals[i])
                        self.joints[i] = steps / STEP_PER_DEG[i]
            except (ValueError, IndexError):
                pass

    def get_joints(self):
        """Get current joint positions in degrees."""
        self._update_joints()
        return list(self.joints)

    def move_joints(self, joints_deg):
        """Execute joint move (degrees)."""
        if len(joints_deg) < N_AXES:
            return False

        # Convert degrees to steps
        steps = []
        for i in range(N_AXES):
            s = int(joints_deg[i] * STEP_PER_DEG[i])
            steps.append(s)

        # Send move command: #M<j1>,<j2>,<j3>,<j4>,<j5>,<j6>
        cmd = "#M" + ",".join(str(s) for s in steps)
        resp = self.send_command(cmd, timeout=10.0)

        # Wait for movement to complete (poll position)
        time.sleep(0.1)
        self._update_joints()
        return True

    def stop(self):
        """Emergency stop."""
        self.send_command("#H")
        return True

    def set_speed(self, linear_mm_s=None, joint_deg_s=None):
        """Set speed (stored locally, applied to moves)."""
        if linear_mm_s is not None:
            self.speed_linear = linear_mm_s
        if joint_deg_s is not None:
            self.speed_joint = joint_deg_s

    def set_tool(self, x, y, z, w, p, r):
        """Set TCP (stored locally for RoboDK tracking)."""
        # TODO: Store TCP offset if needed
        pass

    def move_gripper(self, steps):
        """Move gripper (Slave 2 axis 3). Positive = open, negative = close."""
        cmd = f"#G{steps}"
        resp = self.send_command(cmd, timeout=5.0)
        return resp is not None and not resp.startswith(">ER")

    def set_digital_output(self, io_id, value):
        """Set digital output. Maps to LED or gripper."""
        if io_id == 0:
            # LED control
            if value:
                self.send_command("#LED255,0,0")
            else:
                self.send_command("#LED0,0,0,255")
        elif io_id == 1:
            # Gripper control: value = steps (positive=open, negative=close)
            self.move_gripper(int(value))
        return True

    def get_digital_input(self, io_id):
        """Read digital input (limit switches)."""
        resp = self.send_command("#L", timeout=1.0)
        if resp and resp.startswith("L:"):
            try:
                mask = int(resp[2:])
                return 1 if (mask & (1 << io_id)) else 0
            except ValueError:
                pass
        return 0


# ---------------------------------------------------------------------------
# RoboDK Status helpers
# ---------------------------------------------------------------------------
STATUS = ROBOTCOM_DISCONNECTED

def print_message(message):
    """Display status message in RoboDK (SMS:)."""
    print("SMS:" + message)
    sys.stdout.flush()

def show_message(message):
    """Display in status bar (SMS2:)."""
    print("SMS2:" + message)
    sys.stdout.flush()

def print_response(message):
    """Response to API command (RE:)."""
    print("RE:" + message)
    sys.stdout.flush()

def print_joints(joints, is_moving=False):
    """Report joint positions to RoboDK."""
    if is_moving:
        print("JNTS_MOVING " + " ".join(format(x, ".3f") for x in joints))
    else:
        print("JNTS " + " ".join(format(x, ".6f") for x in joints))
    sys.stdout.flush()

def UpdateStatus(set_status=None):
    """Update RoboDK connection status."""
    global STATUS
    if set_status is not None:
        STATUS = set_status

    status_messages = {
        ROBOTCOM_CONNECTION_PROBLEMS: "Connection problems",
        ROBOTCOM_DISCONNECTED: "Disconnected",
        ROBOTCOM_NOT_CONNECTED: "Not connected",
        ROBOTCOM_READY: "Ready",
        ROBOTCOM_WORKING: "Working...",
        ROBOTCOM_WAITING: "Waiting...",
    }
    msg = status_messages.get(STATUS, "Unknown")
    print_message(msg)


# ---------------------------------------------------------------------------
# Main RoboDK Driver Loop
# ---------------------------------------------------------------------------
ROBOT = Nite369Controller()

def RunCommand(cmd_line):
    """Parse and execute a RoboDK command."""
    if cmd_line.strip() == "":
        return

    global ROBOT

    parts = cmd_line.strip().split(" ")
    cmd = parts[0]

    try:
        values = [float(x) for x in parts[1:] if _is_float(x)]
    except ValueError:
        values = []

    n_values = len(values)

    # --- CONNECT ---
    if cmd == "CONNECT":
        UpdateStatus(ROBOTCOM_WORKING)
        port_or_ip = parts[1] if len(parts) > 1 else "COM3"
        port_num = parts[2] if len(parts) > 2 else None
        if ROBOT.connect(port_or_ip, port_num):
            UpdateStatus(ROBOTCOM_READY)
        else:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    # --- DISCONNECT ---
    elif cmd == "DISCONNECT":
        ROBOT.disconnect()
        UpdateStatus(ROBOTCOM_DISCONNECTED)

    # --- QUIT / STOP ---
    elif cmd in ("QUIT", "STOP"):
        ROBOT.stop()
        ROBOT.disconnect()
        UpdateStatus(ROBOTCOM_DISCONNECTED)
        quit(0)

    # --- MOVJ (Joint move) ---
    elif cmd == "MOVJ" and n_values >= N_AXES:
        UpdateStatus(ROBOTCOM_WORKING)
        joints = values[:N_AXES]
        if ROBOT.move_joints(joints):
            UpdateStatus(ROBOTCOM_READY)
        else:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    # --- MOVL (Linear move) ---
    # For now, treat as joint move (full IK requires robot model)
    elif cmd == "MOVL" and n_values >= N_AXES:
        UpdateStatus(ROBOTCOM_WORKING)
        joints = values[:N_AXES]
        if ROBOT.move_joints(joints):
            UpdateStatus(ROBOTCOM_READY)
        else:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    # --- CJNT (Get current joints) ---
    elif cmd == "CJNT":
        UpdateStatus(ROBOTCOM_WORKING)
        joints = ROBOT.get_joints()
        print_joints(joints)
        UpdateStatus(ROBOTCOM_READY)

    # --- SPEED ---
    elif cmd == "SPEED" and n_values >= 2:
        ROBOT.set_speed(
            linear_mm_s=values[0] if values[0] >= 0 else None,
            joint_deg_s=values[1] if values[1] >= 0 else None
        )
        UpdateStatus(ROBOTCOM_READY)

    # --- SETTOOL ---
    elif cmd == "SETTOOL" and n_values >= 6:
        ROBOT.set_tool(*values[:6])
        UpdateStatus(ROBOTCOM_READY)

    # --- SETDO (Digital output) ---
    elif cmd == "SETDO" and n_values >= 2:
        ROBOT.set_digital_output(int(values[0]), int(values[1]))
        UpdateStatus(ROBOTCOM_READY)

    # --- WAITDI (Digital input) ---
    elif cmd == "WAITDI" and n_values >= 2:
        # Simple poll
        val = ROBOT.get_digital_input(int(values[0]))
        UpdateStatus(ROBOTCOM_READY)

    # --- PAUSE ---
    elif cmd == "PAUSE" and n_values >= 1:
        time.sleep(values[0] / 1000.0)
        UpdateStatus(ROBOTCOM_READY)

    # --- SETROUNDING ---
    elif cmd == "SETROUNDING":
        UpdateStatus(ROBOTCOM_READY)

    # --- RUNPROG ---
    elif cmd == "RUNPROG":
        UpdateStatus(ROBOTCOM_READY)

    # --- POPUP ---
    elif cmd == "POPUP":
        UpdateStatus(ROBOTCOM_READY)

    # --- Custom commands ---
    elif cmd.startswith("c "):
        # Forward custom command to robot
        custom = cmd_line[2:]
        resp = ROBOT.send_command("#" + custom)
        if resp:
            print_response(resp)
        UpdateStatus(ROBOTCOM_READY)

    else:
        print("Unknown command: " + cmd_line)


def _is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def RunDriver():
    """Main driver loop. Reads STDIN commands from RoboDK."""
    for line in sys.stdin:
        RunCommand(line)


def TestDriver():
    """Test driver with sample commands (console mode)."""
    RunCommand("CONNECT")
    RunCommand("CJNT")
    RunCommand("MOVJ 0 0 0 0 0 0")
    RunCommand("MOVJ 10 0 0 0 0 0")
    RunCommand("CJNT")
    RunCommand("SPEED 200 100")
    RunCommand("SETDO 0 1")
    RunCommand("PAUSE 1000")
    RunCommand("SETDO 0 0")
    RunCommand("DISCONNECT")


def RunMain():
    """Entry point."""
    import atexit
    atexit.register(lambda: ROBOT.disconnect())

    print_message(DRIVER_VERSION)
    UpdateStatus()

    # Check if running from RoboDK (STDIN has data) or console mode
    if len(sys.argv) > 1:
        # Console mode with connection parameter
        port_or_ip = sys.argv[1]
        port_num = sys.argv[2] if len(sys.argv) > 2 else None
        ROBOT.connect(port_or_ip, port_num)
        UpdateStatus(ROBOTCOM_READY)
        RunDriver()
    else:
        # Try STDIN (RoboDK mode) or console test
        try:
            RunDriver()
        except (KeyboardInterrupt, EOFError):
            pass

    # Fallback: run test
    TestDriver()


if __name__ == "__main__":
    RunMain()
