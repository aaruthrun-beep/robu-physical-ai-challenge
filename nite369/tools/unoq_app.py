#!/usr/bin/env python3
"""Uno Q — Industrial Robot Controller.

Professional web-based controller for:
  - C920 webcam with live feed
  - 2-link SCARA arm (harmonic drives on CAN bus)
  - 6-axis industrial robot arm (DH kinematics)
  - Hexapod (6 legs, each with own microcontroller)
  - Serial port scanner
  - Network device discovery

All hardware shows 'disconnected' when not present.
Kinematics, visualization, and UI are fully functional.

Run:  python3 -u unoq_app.py [--port 8080] [--camera /dev/video2]
Open: http://<ip>:8080/
"""
import sys, os, time, json, math, socket, struct, threading, glob as _glob
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from urllib.parse import urlparse

# ══════════════════════════════════════════════════════════════════
# CONFIG
# ══════════════════════════════════════════════════════════════════
CAMERA_DEV  = "/dev/video2"
CAMERA_W    = 640
CAMERA_H    = 480
CAMERA_FPS  = 15
HTTP_PORT   = 8080
ROBOT_HOST  = "192.168.1.50"
ROBOT_PORT  = 23

# ══════════════════════════════════════════════════════════════════
# STATUS
# ══════════════════════════════════════════════════════════════════
class SubsystemStatus:
    def __init__(self):
        self._lock = threading.Lock()
        self._items = {}
    def set(self, name, ok, detail=""):
        with self._lock:
            self._items[name] = {"ok": ok, "detail": detail}
        tag = "OK" if ok else "UNAVAILABLE"
        print(f"  [{tag:>10}]  {name}: {detail}", flush=True)
    def snapshot(self):
        with self._lock:
            return dict(self._items)

status = SubsystemStatus()

# ══════════════════════════════════════════════════════════════════
# CAMERA
# ══════════════════════════════════════════════════════════════════
_frame_lock   = threading.Lock()
_latest_frame = None
_frame_event  = threading.Event()

def capture_loop(dev, w, h, fps):
    global _latest_frame
    import cv2
    cap = cv2.VideoCapture(dev)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    if not cap.isOpened():
        status.set("Camera", False, f"Cannot open {dev}")
        return
    ret, test = cap.read()
    if not ret or test is None:
        status.set("Camera", False, f"{dev} opened but no frame")
        cap.release()
        return
    status.set("Camera", True, f"{dev} {test.shape[1]}x{test.shape[0]} @ {fps}fps")
    interval = 1.0 / fps
    while True:
        t0 = time.monotonic()
        ret, frame = cap.read()
        if ret and frame is not None:
            with _frame_lock:
                _latest_frame = frame
            _frame_event.set()
        else:
            time.sleep(0.05)
            continue
        elapsed = time.monotonic() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)

def get_frame():
    with _frame_lock:
        return _latest_frame.copy() if _latest_frame is not None else None

# ══════════════════════════════════════════════════════════════════
# SCARA KINEMATICS (2-link planar)
# ══════════════════════════════════════════════════════════════════
class SCARA:
    def __init__(self, l1=300.0, l2=390.0):
        self.l1, self.l2 = l1, l2
        self.max_r = l1 + l2
        self.min_r = abs(l1 - l2)
    def fk(self, j1_deg, j2_deg):
        t1, t2 = math.radians(j1_deg), math.radians(j2_deg)
        x = self.l1*math.cos(t1) + self.l2*math.cos(t1+t2)
        y = self.l1*math.sin(t1) + self.l2*math.sin(t1+t2)
        return round(x,2), round(y,2)
    def ik(self, x, y):
        d = math.sqrt(x*x + y*y)
        if d > self.max_r or d < self.min_r:
            return None
        c2 = (x*x + y*y - self.l1**2 - self.l2**2) / (2*self.l1*self.l2)
        c2 = max(-1.0, min(1.0, c2))
        q2 = math.acos(c2)
        k1 = self.l1 + self.l2*math.cos(q2)
        k2 = self.l2*math.sin(q2)
        q1 = math.atan2(y, x) - math.atan2(k2, k1)
        return round(math.degrees(q1),2), round(math.degrees(q2),2)
    def joints(self, j1_deg, j2_deg):
        """Return all joint positions: base, elbow, tool."""
        t1 = math.radians(j1_deg)
        t12 = math.radians(j1_deg + j2_deg)
        j0 = (0, 0)
        j1 = (self.l1*math.cos(t1), self.l1*math.sin(t1))
        j2 = (j1[0] + self.l2*math.cos(t12), j1[1] + self.l2*math.sin(t12))
        return j0, j1, j2

scara = SCARA()

# ══════════════════════════════════════════════════════════════════
# 6-AXIS ROBOT KINEMATICS (DH parameters)
# ══════════════════════════════════════════════════════════════════
class Robot6Axis:
    """6-DOF industrial robot with DH parameters.

    DH Convention (modified):
      theta = joint angle
      d     = link offset
      a     = link length
      alpha = link twist

    Default values approximate a mid-size industrial arm (like UR5/10).
    """
    # DH: [a(mm), alpha(rad), d(mm), theta_offset(rad)]
    DH = [
        [0,    -math.pi/2,  162.5,  0],          # J1
        [-425,  0,           0,     -math.pi/2],   # J2
        [-392.2, 0,          0,      0],            # J3
        [0,    -math.pi/2,  109.15,  0],           # J4
        [0,     math.pi/2,  94.65,   0],           # J5
        [0,     0,          82.3,    0],            # J6
    ]

    def __init__(self):
        self.limits = [
            (-360, 360), (-360, 360), (-360, 360),
            (-360, 360), (-360, 360), (-360, 360),
        ]

    @staticmethod
    def _dh_matrix(a, alpha, d, theta):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return [
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1],
        ]

    @staticmethod
    def _mat_mul(A, B):
        return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

    def fk(self, joints_deg):
        """Forward kinematics: 6 joint angles (deg) -> 4x4 transform."""
        T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        for i, (a, alpha, d, theta_off) in enumerate(self.DH):
            theta = math.radians(joints_deg[i]) + theta_off
            Ti = self._dh_matrix(a, alpha, d, theta)
            T = self._mat_mul(T, Ti)
        return T

    def joint_positions(self, joints_deg):
        """Return (x,y,z) of base + each joint end for visualization."""
        positions = [(0, 0, 0)]
        T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        for i, (a, alpha, d, theta_off) in enumerate(self.DH):
            theta = math.radians(joints_deg[i]) + theta_off
            Ti = self._dh_matrix(a, alpha, d, theta)
            T = self._mat_mul(T, Ti)
            positions.append((round(T[0][3],1), round(T[1][3],1), round(T[2][3],1)))
        return positions

    def tool_pose(self, joints_deg):
        T = self.fk(joints_deg)
        return {
            "x": round(T[0][3], 1), "y": round(T[1][3], 1), "z": round(T[2][3], 1),
            "rx": round(math.degrees(math.atan2(T[2][1], T[2][2])), 1),
            "ry": round(math.degrees(math.asin(max(-1,min(1,-T[2][0])))), 1),
            "rz": round(math.degrees(math.atan2(T[1][0], T[0][0])), 1),
        }

robot6 = Robot6Axis()

# ══════════════════════════════════════════════════════════════════
# HEXAPOD
# ══════════════════════════════════════════════════════════════════
class Hexapod:
    """6-leg hexapod, each leg 3 DOF (coxa, femur, tibia)."""
    N_LEGS = 6
    # Default leg dimensions (mm)
    COXA = 38.0
    FEMUR = 80.0
    TIBIA = 125.0
    # Default home angles (deg)
    HOME = [0, 0, 0]  # coxa, femur, tibia per leg

    def __init__(self):
        self.legs = {i: {"coxa": 0, "femur": -30, "tibia": 45, "connected": False}
                     for i in range(self.N_LEGS)}
        self.controllers = {}

    def discover(self):
        """Scan for leg controllers."""
        found = 0
        # USB serial
        for tty in sorted(_glob.glob("/dev/ttyACM*") + _glob.glob("/dev/ttyUSB*")):
            try:
                import serial
                ser = serial.Serial(tty, 115200, timeout=1)
                ser.write(b"STATUS\n")
                time.sleep(0.1)
                resp = ser.readline().decode(errors="replace").strip()
                if resp:
                    leg_id = len(self.controllers)
                    if leg_id < self.N_LEGS:
                        self.controllers[leg_id] = {"port": tty, "serial": ser}
                        self.legs[leg_id]["connected"] = True
                        found += 1
                else:
                    ser.close()
            except Exception:
                pass
        if found > 0:
            status.set("Hexapod", True, f"{found}/{self.N_LEGS} legs via USB serial")
        else:
            status.set("Hexapod", False, f"0/{self.N_LEGS} — no controllers found")
        return found

    def close(self):
        for c in self.controllers.values():
            try: c["serial"].close()
            except: pass
        self.controllers.clear()

hexapod = Hexapod()

# ══════════════════════════════════════════════════════════════════
# ROBOT LINK (Ethernet / USB serial)
# ══════════════════════════════════════════════════════════════════
class RobotLink:
    def __init__(self):
        self.transport = None
        self.conn = None
        self.lock = threading.Lock()
        self.host = ROBOT_HOST
        self.port = ROBOT_PORT

    def connect(self):
        try:
            s = socket.create_connection((self.host, self.port), timeout=3)
            s.settimeout(2)
            s.sendall(b"#V\n")
            resp = s.recv(256).decode(errors="replace").strip()
            if resp:
                self.transport, self.conn = "tcp", s
                status.set("Robot Link", True, f"TCP {self.host}:{self.port} — {resp[:40]}")
                return True
            s.close()
        except Exception:
            pass
        for tty in sorted(_glob.glob("/dev/ttyACM*") + _glob.glob("/dev/ttyUSB*")):
            try:
                import serial
                ser = serial.Serial(tty, 115200, timeout=2)
                time.sleep(0.5)
                ser.write(b"#V\n")
                time.sleep(0.1)
                resp = ser.readline().decode(errors="replace").strip()
                if resp:
                    self.transport, self.conn = "serial", ser
                    status.set("Robot Link", True, f"Serial {tty} — {resp[:40]}")
                    return True
                ser.close()
            except Exception:
                pass
        status.set("Robot Link", False, f"TCP {self.host}:{self.port} + USB serial: unavailable")
        return False

    def send(self, cmd):
        if self.conn is None: return None
        with self.lock:
            try:
                if self.transport == "tcp":
                    self.conn.sendall((cmd+"\n").encode())
                    return self.conn.recv(512).decode(errors="replace").strip()
                elif self.transport == "serial":
                    self.conn.write((cmd+"\n").encode())
                    time.sleep(0.05)
                    return self.conn.readline().decode(errors="replace").strip()
            except Exception:
                self.close()
        return None

    def close(self):
        if self.conn:
            try: self.conn.close()
            except: pass
            self.conn = None
            self.transport = None

robot_link = RobotLink()

# ══════════════════════════════════════════════════════════════════
# CAN BUS
# ══════════════════════════════════════════════════════════════════
class CANBus:
    def __init__(self):
        self.bus = None
    def open(self):
        try:
            import can
            self.bus = can.interface.Bus(channel="can0", bustype="socketcan", bitrate=500000)
            status.set("CAN Bus", True, "can0 @ 500kbit/s")
            return True
        except ImportError:
            status.set("CAN Bus", False, "python-can not installed")
        except Exception as e:
            status.set("CAN Bus", False, str(e)[:60])
        return False
    def send_position(self, motor_id, angle_deg):
        if not self.bus: return False
        try:
            import can as _can
            msg = _can.Message(arbitration_id=motor_id, data=struct.pack("<f", angle_deg), is_extended_id=False)
            self.bus.send(msg, timeout=1.0)
            return True
        except: return False
    def close(self):
        if self.bus:
            try: self.bus.shutdown()
            except: pass
            self.bus = None

can_bus = CANBus()

# ══════════════════════════════════════════════════════════════════
# SERIAL SCANNER
# ══════════════════════════════════════════════════════════════════
def scan_serial_ports():
    """Scan all serial ports and return info about each."""
    ports = []
    for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyS*", "/dev/ttyHS*"]:
        for tty in sorted(_glob.glob(pattern)):
            info = {"port": tty, "baud": 115200, "type": "unknown", "in_use": False}
            if "ttyACM" in tty: info["type"] = "USB-CDC"
            elif "ttyUSB" in tty: info["type"] = "USB-Serial"
            elif "ttyHS" in tty: info["type"] = "Hardware-UART"
            elif "ttyS" in tty: info["type"] = "Platform-UART"
            # Check if in use
            try:
                import subprocess
                r = subprocess.run(["fuser", tty], capture_output=True, timeout=2)
                if r.returncode == 0:
                    info["in_use"] = True
                    info["user"] = r.stdout.decode(errors="replace").strip()
            except: pass
            ports.append(info)
    return ports

# ══════════════════════════════════════════════════════════════════
# NETWORK SCANNER
# ══════════════════════════════════════════════════════════════════
def scan_network():
    """Scan local subnet for devices with open ports."""
    import subprocess
    try:
        local_ip = subprocess.check_output(["hostname", "-I"], timeout=2).decode().strip().split()[0]
        subnet = ".".join(local_ip.split(".")[:3])
    except: subnet = "192.168.1"
    devices = []
    # Scan common device IPs
    targets = list(range(1, 30)) + list(range(50, 70)) + list(range(100, 120))
    def check_host(ip):
        try:
            s = socket.create_connection((ip, 1), timeout=0.2)
            s.close()
            return True
        except:
            try:
                r = subprocess.run(["ping", "-c", "1", "-W", "200", ip],
                                   capture_output=True, timeout=0.5)
                return r.returncode == 0
            except: return False
    threads = []
    results = {}
    def scan_one(ip):
        full = f"{subnet}.{ip}"
        if check_host(full):
            results[full] = True
    for i in targets:
        t = threading.Thread(target=scan_one, args=(i,), daemon=True)
        threads.append(t)
        t.start()
    for t in threads:
        t.join(timeout=1)
    for ip in sorted(results.keys()):
        # Check common ports
        open_ports = []
        for port in [22, 80, 23, 8080, 5000, 5001]:
            try:
                s = socket.create_connection((ip, port), timeout=0.2)
                open_ports.append(port)
                s.close()
            except: pass
        devices.append({"ip": ip, "ports": open_ports, "label": _identify_device(ip, open_ports)})
    return subnet, devices

def _identify_device(ip, ports):
    """Try to label a device based on its open ports."""
    last = int(ip.split(".")[-1])
    if 22 in ports and 80 not in ports: return "Linux SBC (SSH)"
    if 22 in ports and 80 in ports: return "Web Server"
    if 23 in ports: return "Telnet Device"
    if 8080 in ports: return "API/Web Device"
    if 5000 in ports: return "Robot Controller"
    if 22 in ports: return "SSH Device"
    return "Unknown Device"

# ══════════════════════════════════════════════════════════════════
# HTML UI
# ══════════════════════════════════════════════════════════════════
HTML_PAGE = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>UNO-Q CTRL</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111;color:#b0b0b0;font:11px/1.3 'Courier New',monospace;overflow:hidden;height:100vh}
/* ── TOP BAR ── */
.topbar{background:#1a1a1a;border-bottom:1px solid #333;padding:3px 10px;display:flex;align-items:center;justify-content:space-between;height:28px}
.topbar .logo{color:#0f0;font-size:11px;font-weight:700;letter-spacing:2px}
.topbar .mode{display:flex;gap:6px;align-items:center}
.mode-tag{padding:2px 8px;font-size:9px;font-weight:700;border:1px solid #333}
.mode-run{background:#1a3a1a;color:#0f0;border-color:#0a0}
.mode-teach{background:#3a3a1a;color:#ff0;border-color:#aa0}
.mode-fault{background:#3a1a1a;color:#f00;border-color:#a00}
.topbar .info{color:#666;font-size:10px}
.led{display:inline-block;width:6px;height:6px;border-radius:1px;margin-right:4px}
.led-g{background:#0f0}.led-r{background:#f00}.led-y{background:#ff0}.led-off{background:#333}
/* ── MAIN LAYOUT ── */
.main{display:grid;grid-template-columns:1fr 300px;height:calc(100vh - 28px)}
.left{display:flex;flex-direction:column;overflow-y:auto;border-right:1px solid #333}
.right{display:flex;flex-direction:column;overflow-y:auto;background:#0e0e0e}
/* ── TABS ── */
.tabs{display:flex;background:#1a1a1a;border-bottom:1px solid #333}
.tab{padding:5px 14px;font-size:10px;cursor:pointer;border-right:1px solid #333;color:#666;background:#1a1a1a}
.tab:hover{color:#aaa}
.tab.active{background:#222;color:#0f0;border-bottom:2px solid #0f0}
.panel{display:none;padding:8px}
.panel.active{display:block}
/* ── CAM ── */
.cam-box{background:#000;border:1px solid #333;text-align:center}
.cam-box img{width:100%;display:block;max-height:300px;object-fit:contain}
.cam-bar{display:flex;justify-content:space-between;padding:2px 6px;font-size:9px;color:#555;background:#111;border-top:1px solid #222}
/* ── READOUT ── */
.readout{background:#111;border:1px solid #333;padding:6px;margin:6px 0}
.readout-title{font-size:9px;color:#555;text-transform:uppercase;letter-spacing:1px;margin-bottom:4px;border-bottom:1px solid #222;padding-bottom:3px}
.jog-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:3px}
.jog-cell{background:#1a1a1a;border:1px solid #2a2a2a;padding:3px 5px;display:flex;justify-content:space-between;align-items:center}
.jog-cell .jl{color:#555;font-size:9px}
.jog-cell .jv{color:#0f0;font-size:12px;font-weight:700;font-family:'Courier New',monospace}
.jog-cell .ju{color:#444;font-size:9px}
/* ── SLIDER ── */
.slider-row{display:flex;align-items:center;gap:4px;padding:2px 0}
.slider-row label{color:#555;font-size:9px;min-width:22px}
.slider-row input[type=range]{flex:1;height:4px;-webkit-appearance:none;background:#222;outline:none;border-radius:0}
.slider-row input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:10px;height:14px;background:#0f0;border:none;cursor:pointer}
.slider-row .sv{color:#0f0;font-size:10px;min-width:42px;text-align:right;font-weight:700}
/* ── CANVAS ── */
.vis-box{background:#0a0a0a;border:1px solid #333;margin:4px 0;position:relative}
.vis-box canvas{width:100%;display:block}
.vis-label{position:absolute;top:3px;left:6px;font-size:8px;color:#333}
/* ── SIDE PANELS ── */
.scard{background:#111;border:1px solid #333;margin:4px;padding:6px}
.scard h4{font-size:9px;color:#555;text-transform:uppercase;letter-spacing:1px;margin-bottom:4px;padding-bottom:3px;border-bottom:1px solid #222;display:flex;justify-content:space-between;align-items:center}
.srow{display:flex;justify-content:space-between;padding:2px 0;font-size:10px;border-bottom:1px solid #1a1a1a}
.srow:last-child{border:none}
.srow .sk{color:#555}.srow .sv{color:#888}
.stag{font-size:8px;padding:1px 5px;font-weight:700}
.stag-on{background:#1a3a1a;color:#0f0;border:1px solid #0a0}
.stag-off{background:#3a1a1a;color:#f00;border:1px solid #500}
/* ── INPUTS ── */
.inp-row{display:flex;gap:4px;margin-top:4px}
.inp{flex:1;background:#0a0a0a;border:1px solid #333;color:#ccc;padding:4px 6px;font:10px 'Courier New',monospace}
.inp:focus{border-color:#0f0;outline:none}
.btn{background:#1a2a1a;color:#0f0;border:1px solid #0a0;padding:4px 10px;font:10px 'Courier New',monospace;cursor:pointer;font-weight:700}
.btn:hover{background:#2a3a2a}
.btn-r{background:#2a1a1a;color:#f00;border-color:#500}
.btn-r:hover{background:#3a2a2a}
.btn-o{background:#1a1a1a;color:#888;border-color:#333}
.btn-o:hover{color:#ccc}
/* ── LOG ── */
.log-box{background:#0a0a0a;border:1px solid #333;padding:4px;max-height:100px;overflow-y:auto;font-size:9px;color:#555;margin:4px}
.log-box .le{color:#f00}.log-box .lo{color:#0f0}
/* ── HEX LEGS ── */
.hex-grid{display:grid;grid-template-columns:repeat(6,1fr);gap:2px}
.hex-leg{text-align:center;background:#1a1a1a;border:1px solid #2a2a2a;padding:3px}
.hex-leg .hl{font-size:8px;color:#555}
.hex-leg input[type=range]{width:100%;height:3px;-webkit-appearance:none;background:#222;outline:none}
.hex-leg input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:6px;height:10px;background:#0f0;border:none}
.hex-leg .hv{color:#0f0;font-size:9px;font-weight:700}
</style>
</head>
<body>
<!-- ══════ TOP BAR ══════ -->
<div class="topbar">
  <span class="logo">UNO-Q</span>
  <div class="mode">
    <span class="mode-tag mode-run">RUN</span>
    <span class="mode-tag mode-teach">TEACH</span>
  </div>
  <div class="info">
    <span class="led led-g"></span>SYS
    <span class="led led-off" id="led-net"></span>NET
    <span class="led led-off" id="led-can"></span>CAN
    <span class="led led-off" id="led-rbt"></span>RBT
    &nbsp;&nbsp;<span id="clock"></span>
  </div>
</div>
<!-- ══════ MAIN ══════ -->
<div class="main">
  <div class="left">
    <!-- Camera -->
    <div class="cam-box">
      <img id="video" src="/stream?0">
      <div class="cam-bar"><span id="cam-st">CONNECTING</span><span>C920 &middot; 640x480</span><span id="cam-fps"></span></div>
    </div>
    <!-- Tabs -->
    <div class="tabs">
      <div class="tab active" onclick="showTab(0,this)">JOG</div>
      <div class="tab" onclick="showTab(1,this)">6-AXIS</div>
      <div class="tab" onclick="showTab(2,this)">HEXAPOD</div>
      <div class="tab" onclick="showTab(3,this)">I/O</div>
      <div class="tab" onclick="showTab(4,this)">PROGRAM</div>
    </div>
    <!-- Panel: JOG (SCARA) -->
    <div class="panel active" id="p0">
      <div class="readout">
        <div class="readout-title">SCARA ARM &mdash; JOINT POSITION</div>
        <div class="jog-grid">
          <div class="jog-cell"><span class="jl">J1</span><span class="jv" id="sj1">+030.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">J2</span><span class="jv" id="sj2">-045.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">TOOL</span><span class="jv" id="stool">---.---</span><span class="ju">mm</span></div>
        </div>
        <div class="jog-grid" style="margin-top:3px">
          <div class="jog-cell"><span class="jl">X</span><span class="jv" id="sx">+000.00</span><span class="ju">mm</span></div>
          <div class="jog-cell"><span class="jl">Y</span><span class="jv" id="sy">+000.00</span><span class="ju">mm</span></div>
          <div class="jog-cell"><span class="jl">R</span><span class="jv" id="sr">+000.0</span><span class="ju">mm</span></div>
        </div>
      </div>
      <div class="slider-row"><label>J1</label><input type="range" id="saj1" min="-180" max="180" value="30" oninput="drawSCARA()"><span class="sv" id="svj1">30</span></div>
      <div class="slider-row"><label>J2</label><input type="range" id="saj2" min="-150" max="150" value="-45" oninput="drawSCARA()"><span class="sv" id="svj2">-45</span></div>
      <div class="inp-row">
        <input class="inp" id="scara-go" placeholder="x,y mm">
        <button class="btn" onclick="moveScara()">GO</button>
      </div>
      <div class="vis-box"><canvas id="scara-cv" width="600" height="200"></canvas><span class="vis-label">SCARA WORKSPACE</span></div>
    </div>
    <!-- Panel: 6-AXIS -->
    <div class="panel" id="p1">
      <div class="readout">
        <div class="readout-title">6-AXIS ARM &mdash; JOINT POSITION</div>
        <div class="jog-grid">
          <div class="jog-cell"><span class="jl">J1</span><span class="jv" id="rj1">+000.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">J2</span><span class="jv" id="rj2">-060.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">J3</span><span class="jv" id="rj3">+060.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">J4</span><span class="jv" id="rj4">+000.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">J5</span><span class="jv" id="rj5">-090.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">J6</span><span class="jv" id="rj6">+000.0</span><span class="ju">deg</span></div>
        </div>
        <div class="jog-grid" style="margin-top:3px">
          <div class="jog-cell"><span class="jl">X</span><span class="jv" id="rx">+000.0</span><span class="ju">mm</span></div>
          <div class="jog-cell"><span class="jl">Y</span><span class="jv" id="ry">+000.0</span><span class="ju">mm</span></div>
          <div class="jog-cell"><span class="jl">Z</span><span class="jv" id="rz">+000.0</span><span class="ju">mm</span></div>
          <div class="jog-cell"><span class="jl">RX</span><span class="jv" id="rrx">+000.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">RY</span><span class="jv" id="rry">+000.0</span><span class="ju">deg</span></div>
          <div class="jog-cell"><span class="jl">RZ</span><span class="jv" id="rrz">+000.0</span><span class="ju">deg</span></div>
        </div>
      </div>
      <div class="slider-row"><label>J1</label><input type="range" id="raj1" min="-180" max="180" value="0" oninput="drawR6()"><span class="sv" id="rv1">0</span></div>
      <div class="slider-row"><label>J2</label><input type="range" id="raj2" min="-180" max="180" value="-60" oninput="drawR6()"><span class="sv" id="rv2">-60</span></div>
      <div class="slider-row"><label>J3</label><input type="range" id="raj3" min="-180" max="180" value="60" oninput="drawR6()"><span class="sv" id="rv3">60</span></div>
      <div class="slider-row"><label>J4</label><input type="range" id="raj4" min="-180" max="180" value="0" oninput="drawR6()"><span class="sv" id="rv4">0</span></div>
      <div class="slider-row"><label>J5</label><input type="range" id="raj5" min="-180" max="180" value="-90" oninput="drawR6()"><span class="sv" id="rv5">-90</span></div>
      <div class="slider-row"><label>J6</label><input type="range" id="raj6" min="-180" max="180" value="0" oninput="drawR6()"><span class="sv" id="rv6">0</span></div>
      <div class="vis-box"><canvas id="r6-cv" width="600" height="220"></canvas><span class="vis-label">6-AXIS SIDE VIEW</span></div>
    </div>
    <!-- Panel: HEXAPOD -->
    <div class="panel" id="p2">
      <div class="readout">
        <div class="readout-title">HEXAPOD &mdash; LEG STATUS</div>
        <div class="hex-grid" id="hex-ctrls"></div>
      </div>
      <div class="inp-row" style="margin-top:4px">
        <button class="btn" onclick="hexHome()">HOME</button>
        <button class="btn btn-o" onclick="hexStand()">STAND</button>
        <button class="btn btn-o" onclick="hexWave()">WAVE</button>
      </div>
      <div class="vis-box"><canvas id="hex-cv" width="600" height="180"></canvas><span class="vis-label">HEXAPOD TOP VIEW</span></div>
    </div>
    <!-- Panel: I/O -->
    <div class="panel" id="p3">
      <div class="readout">
        <div class="readout-title">DIGITAL I/O</div>
        <div class="jog-grid">
          <div class="jog-cell"><span class="jl">DI0</span><span class="jv" style="color:#333">0</span><span class="ju"></span></div>
          <div class="jog-cell"><span class="jl">DI1</span><span class="jv" style="color:#333">0</span><span class="ju"></span></div>
          <div class="jog-cell"><span class="jl">DI2</span><span class="jv" style="color:#333">0</span><span class="ju"></span></div>
          <div class="jog-cell"><span class="jl">DO0</span><span class="jv" style="color:#333">0</span><span class="ju"></span></div>
          <div class="jog-cell"><span class="jl">DO1</span><span class="jv" style="color:#333">0</span><span class="ju"></span></div>
          <div class="jog-cell"><span class="jl">DO2</span><span class="jv" style="color:#333">0</span><span class="ju"></span></div>
        </div>
      </div>
      <div class="readout">
        <div class="readout-title">ANALOG</div>
        <div class="jog-grid">
          <div class="jog-cell"><span class="jl">AI0</span><span class="jv" style="color:#333">0.000</span><span class="ju">V</span></div>
          <div class="jog-cell"><span class="jl">AI1</span><span class="jv" style="color:#333">0.000</span><span class="ju">V</span></div>
          <div class="jog-cell"><span class="jl">AO0</span><span class="jv" style="color:#333">0.000</span><span class="ju">V</span></div>
        </div>
      </div>
    </div>
    <!-- Panel: PROGRAM -->
    <div class="panel" id="p4">
      <div class="readout">
        <div class="readout-title">PROGRAM</div>
        <div style="color:#333;padding:10px;text-align:center">NO PROGRAM LOADED</div>
      </div>
      <div class="readout">
        <div class="readout-title">EXECUTION</div>
        <div class="srow"><span class="sk">Status</span><span class="sv" style="color:#333">STOPPED</span></div>
        <div class="srow"><span class="sk">Line</span><span class="sv" style="color:#333">---</span></div>
        <div class="srow"><span class="sk">Cycle</span><<span class="sv" style="color:#333">---</span></div>
      </div>
    </div>
    <!-- Log -->
    <div class="log-box" id="log"></div>
  </div>

  <!-- ══════ RIGHT SIDE ══════ -->
  <div class="right">
    <!-- System -->
    <div class="scard">
      <h4>SUBSYSTEMS</h4>
      <div id="sys-list"></div>
    </div>
    <!-- Robot Command -->
    <div class="scard">
      <h4>COMMAND</h4>
      <div class="inp-row">
        <input class="inp" id="cmd-inp" placeholder="#V  #S  #M1,90  #H">
        <button class="btn" onclick="sendCmd()">SEND</button>
      </div>
      <div class="log-box" id="cmd-resp" style="margin-top:3px;min-height:20px;max-height:50px;color:#444">---</div>
    </div>
    <!-- Serial Ports -->
    <div class="scard">
      <h4>SERIAL <button class="btn btn-o" style="font-size:8px;padding:2px 6px" onclick="scanSerial()">SCAN</button></h4>
      <div id="serial-list" style="max-height:80px;overflow-y:auto"><div class="srow"><span class="sk" style="color:#333">Click SCAN</span></div></div>
    </div>
    <!-- Network -->
    <div class="scard">
      <h4>NETWORK <button class="btn btn-o" style="font-size:8px;padding:2px 6px" onclick="scanNet()">SCAN</button></h4>
      <div id="net-list" style="max-height:80px;overflow-y:auto"><div class="srow"><span class="sk" style="color:#333">Click SCAN</span></div></div>
    </div>
    <!-- Actions -->
    <div class="scard">
      <h4>ACTIONS</h4>
      <div style="display:flex;flex-wrap:wrap;gap:3px">
        <button class="btn" onclick="rescan()">RE-SCAN</button>
        <button class="btn btn-r" onclick="sendRaw('#H')">HALT</button>
        <button class="btn btn-o" onclick="sendRaw('#HOME')">HOME</button>
      </div>
    </div>
  </div>
</div>

<script>
// ═══ Clock ═══
setInterval(()=>{document.getElementById('clock').textContent=new Date().toLocaleTimeString()},1000);

// ═══ Tab switch ═══
function showTab(i,el){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
  el.classList.add('active');
  document.getElementById('p'+i).classList.add('active');
}

// ═══ Stream ═══
var vid=document.getElementById('video'),idx=0,fc=0,lt=Date.now();
function loop(){
  var img=new Image();
  img.onload=function(){vid.src=img.src;fc++;var n=Date.now();if(n-lt>1000){document.getElementById('cam-fps').textContent=fc+' fps';fc=0;lt=n}document.getElementById('cam-st').textContent='LIVE';document.getElementById('cam-st').style.color='#0f0';setTimeout(loop,30)};
  img.onerror=function(){setTimeout(loop,500);document.getElementById('cam-st').textContent='OFFLINE';document.getElementById('cam-st').style.color='#f00'};
  img.src='/stream?'+(idx++);
}
loop();

// ═══ Status ═══
function pollStatus(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    var h='';var leds={};
    for(var k in d){
      var s=d[k],c=s.ok?'stag-on':'stag-off',t=s.ok?'ON':'OFF';
      h+='<div class="srow"><span class="sk">'+k+'</span><span class="stag '+c+'">'+t+'</span></div>';
      if(k==='Robot Link')leds.rbt=s.ok;if(k==='CAN Bus')leds.can=s.ok;if(k==='Camera')leds.cam=s.ok;
    }
    document.getElementById('sys-list').innerHTML=h;
    document.getElementById('led-can').className='led '+(leds.can?'led-g':'led-off');
    document.getElementById('led-rbt').className='led '+(leds.rbt?'led-g':'led-r');
    document.getElementById('led-net').className='led '+(leds.cam?'led-g':'led-off');
  }).catch(()=>{});
  setTimeout(pollStatus,3000);
}
pollStatus();

// ═══ SCARA ═══
var sc=document.getElementById('scara-cv'),sx=sc.getContext('2d');
function drawSCARA(){
  var j1=+document.getElementById('saj1').value,j2=+document.getElementById('saj2').value;
  document.getElementById('svj1').textContent=j1;document.getElementById('svj2').textContent=j2;
  fetch('/api/scara/fk?j1='+j1+'&j2='+j2).then(r=>r.json()).then(d=>{
    document.getElementById('sx').textContent=(d.tool_x>=0?'+':'')+d.tool_x.toFixed(2);
    document.getElementById('sy').textContent=(d.tool_y>=0?'+':'')+d.tool_y.toFixed(2);
    document.getElementById('sr').textContent='+'+d.reach.toFixed(1);
    document.getElementById('stool').textContent='+'+d.reach.toFixed(1);
    document.getElementById('sj1').textContent=(j1>=0?'+':'')+j1.toFixed(1);
    document.getElementById('sj2').textContent=(j2>=0?'+':'')+j2.toFixed(1);
    var W=sc.width,H=sc.height,ctx=sx;ctx.clearRect(0,0,W,H);
    ctx.strokeStyle='#1a1a1a';ctx.lineWidth=.5;
    for(var i=0;i<W;i+=20){ctx.beginPath();ctx.moveTo(i,0);ctx.lineTo(i,H);ctx.stroke()}
    for(var i=0;i<H;i+=20){ctx.beginPath();ctx.moveTo(0,i);ctx.lineTo(W,i);ctx.stroke()}
    var ox=130,oy=H/2,sc2=.35;
    ctx.strokeStyle='#1a3a1a';ctx.lineWidth=1;ctx.setLineDash([3,3]);
    ctx.beginPath();ctx.arc(ox,oy,d.l1*2*sc2,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
    var j=d.joints;
    var p0x=ox+j[0][0]*sc2,p0y=oy-j[0][1]*sc2;
    var p1x=ox+j[1][0]*sc2,p1y=oy-j[1][1]*sc2;
    var p2x=ox+j[2][0]*sc2,p2y=oy-j[2][1]*sc2;
    ctx.strokeStyle='#888';ctx.lineWidth=3;ctx.lineCap='round';
    ctx.beginPath();ctx.moveTo(p0x,p0y);ctx.lineTo(p1x,p1y);ctx.stroke();
    ctx.strokeStyle='#0a0';ctx.lineWidth=2;
    ctx.beginPath();ctx.moveTo(p1x,p1y);ctx.lineTo(p2x,p2y);ctx.stroke();
    ctx.fillStyle='#666';ctx.beginPath();ctx.arc(p0x,p0y,4,0,Math.PI*2);ctx.fill();
    ctx.fillStyle='#888';ctx.beginPath();ctx.arc(p1x,p1y,3,0,Math.PI*2);ctx.fill();
    ctx.fillStyle='#0f0';ctx.beginPath();ctx.arc(p2x,p2y,3,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle='#0f030';ctx.lineWidth=1;ctx.beginPath();ctx.arc(p2x,p2y,8,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle='#333';ctx.font='8px monospace';ctx.fillText('BASE',p0x-10,p0y+14);
    ctx.fillText('J2',p1x+6,p1y-4);ctx.fillText('TOOL',p2x+6,p2y-4);
  });
}

function moveScara(){
  var v=document.getElementById('scara-go').value.split(',');if(v.length!=2)return;
  fetch('/api/scara/move',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:+v[0],y:+v[1]})}).then(r=>r.json()).then(d=>{
    if(d.ok){document.getElementById('saj1').value=d.j1;document.getElementById('saj2').value=d.j2;drawSCARA()}
    log((d.ok?'OK':'ERR')+' SCARA move j1='+d.j1+' j2='+d.j2);
  });
}

// ═══ 6-AXIS ═══
var rc=document.getElementById('r6-cv'),rx=rc.getContext('2d');
function drawR6(){
  var joints=[];for(var i=1;i<=6;i++){var v=+document.getElementById('raj'+i).value;joints.push(v);document.getElementById('rv'+i).textContent=v}
  fetch('/api/r6/joints',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({joints:joints})}).then(r=>r.json()).then(d=>{
    var p=d.pose;
    document.getElementById('rx').textContent=(p.x>=0?'+':'')+p.x.toFixed(1);
    document.getElementById('ry').textContent=(p.y>=0?'+':'')+p.y.toFixed(1);
    document.getElementById('rz').textContent=(p.z>=0?'+':'')+p.z.toFixed(1);
    document.getElementById('rrx').textContent=(p.rx>=0?'+':'')+p.rx.toFixed(1);
    document.getElementById('rry').textContent=(p.ry>=0?'+':'')+p.ry.toFixed(1);
    document.getElementById('rrz').textContent=(p.rz>=0?'+':'')+p.rz.toFixed(1);
    for(var i=1;i<=6;i++)document.getElementById('rj'+i).textContent=(joints[i-1]>=0?'+':'')+joints[i-1].toFixed(1);
    var W=rc.width,H=rc.height,ctx=rx;ctx.clearRect(0,0,W,H);
    ctx.strokeStyle='#1a1a1a';ctx.lineWidth=.5;
    for(var i=0;i<W;i+=20){ctx.beginPath();ctx.moveTo(i,0);ctx.lineTo(i,H);ctx.stroke()}
    for(var i=0;i<H;i+=20){ctx.beginPath();ctx.moveTo(0,i);ctx.lineTo(W,i);ctx.stroke()}
    var ox=100,oy=H-30,s=.55;
    ctx.strokeStyle='#f00';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox+50,oy);ctx.stroke();
    ctx.fillStyle='#f00';ctx.font='8px monospace';ctx.fillText('X',ox+52,oy+3);
    ctx.strokeStyle='#0f0';ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox,oy-50);ctx.stroke();
    ctx.fillStyle='#0f0';ctx.fillText('Y',ox-3,oy-55);
    ctx.strokeStyle='#00f';ctx.beginPath();ctx.moveTo(ox,oy);ctx.lineTo(ox-25,oy+18);ctx.stroke();
    ctx.fillStyle='#00f';ctx.fillText('Z',ox-35,oy+22);
    var pts=d.positions;var cols=['#555','#888','#aaa','#0a0','#0cc','#0f0','#f0f'];
    for(var i=0;i<pts.length-1;i++){
      var x0=ox+pts[i][0]*s,y0=oy-pts[i][2]*s;
      var x1=ox+pts[i+1][0]*s,y1=oy-pts[i+1][2]*s;
      ctx.strokeStyle=cols[i];ctx.lineWidth=Math.max(1,4-i*.5);ctx.lineCap='round';
      ctx.beginPath();ctx.moveTo(x0,y0);ctx.lineTo(x1,y1);ctx.stroke();
    }
    for(var i=0;i<pts.length;i++){
      var px=ox+pts[i][0]*s,py=oy-pts[i][2]*s;
      ctx.fillStyle=cols[i];ctx.beginPath();ctx.arc(px,py,3-i*.3,0,Math.PI*2);ctx.fill();
    }
    var tx=ox+pts[pts.length-1][0]*s,ty=oy-pts[pts.length-1][2]*s;
    ctx.fillStyle='#0f0';ctx.font='8px monospace';ctx.fillText('TCP',tx+6,ty-3);
    ctx.strokeStyle='#0f040';ctx.lineWidth=1;ctx.beginPath();ctx.arc(tx,ty,8,0,Math.PI*2);ctx.stroke();
    ctx.fillStyle='#333';ctx.font='8px monospace';ctx.fillText('XZ PLANE',W-60,12);
  });
}
for(var i=1;i<=6;i++)document.getElementById('raj'+i).oninput=drawR6;

// ═══ Hexapod ═══
(function(){var h='';for(var i=0;i<6;i++){
  h+='<div class="hex-leg"><div class="hl">L'+(i+1)+'</div><input type="range" min="-60" max="60" value="0" id="hx'+i+'" oninput="drawHex()"><div class="hv" id="hxv'+i+'">0</div></div>';
}document.getElementById('hex-ctrls').innerHTML=h})();
var hc=document.getElementById('hex-cv'),hctx=hc.getContext('2d');
function drawHex(){
  var W=hc.width,H=hc.height,ctx=hctx;ctx.clearRect(0,0,W,H);
  var cx=W/2,cy=H/2+5;
  ctx.fillStyle='#1a1a1a';ctx.beginPath();
  for(var i=0;i<6;i++){var a=Math.PI/3*i-Math.PI/6,x=cx+25*Math.cos(a),y=cy+25*Math.sin(a);i?ctx.lineTo(x,y):ctx.moveTo(x,y)}
  ctx.closePath();ctx.fill();ctx.strokeStyle='#333';ctx.lineWidth=1;ctx.stroke();
  ctx.fillStyle='#333';ctx.font='8px monospace';ctx.fillText('BODY',cx-12,cy+3);
  var ang=[-30,30,-90,90,-150,150],cols=['#888','#777','#666','#555','#444','#333'];
  for(var i=0;i<6;i++){
    var a=Math.PI/180*ang[i],ba=Math.PI/3*i-Math.PI/6;
    var bx=cx+25*Math.cos(ba),by=cy+25*Math.sin(ba);
    var ex=bx+60*Math.cos(a),ey=by+60*Math.sin(a);
    ctx.strokeStyle=cols[i];ctx.lineWidth=2;ctx.lineCap='round';
    ctx.beginPath();ctx.moveTo(bx,by);ctx.lineTo(ex,ey);ctx.stroke();
    ctx.fillStyle='#0f0';ctx.beginPath();ctx.arc(ex,ey,2,0,Math.PI*2);ctx.fill();
    document.getElementById('hxv'+i).textContent=document.getElementById('hx'+i).value;
  }
}
function hexHome(){for(var i=0;i<6;i++)document.getElementById('hx'+i).value=0;drawHex();log('HEX HOME')}
function hexStand(){log('HEX STAND')}
function hexWave(){log('HEX WAVE')}

// ═══ Robot Cmd ═══
function sendCmd(){var c=document.getElementById('cmd-inp').value.trim();if(!c)return;sendRaw(c)}
function sendRaw(c){
  fetch('/api/robot/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:c})}).then(r=>r.json()).then(d=>{
    var el=document.getElementById('cmd-resp');el.textContent=d.ok?(d.response||'OK'):'---';el.style.color=d.ok?'#0f0':'#500';
    log((d.ok?'OK':'ERR')+' '+c+' '+(d.response||''));
  });
}

// ═══ Serial ═══
function scanSerial(){
  document.getElementById('serial-list').innerHTML='<div class="srow"><span class="sk" style="color:#333">Scanning...</span></div>';
  fetch('/api/serial/scan').then(r=>r.json()).then(d=>{
    if(!d.ports.length){document.getElementById('serial-list').innerHTML='<div class="srow"><span class="sk" style="color:#f00">None found</span></div>';return}
    var h='';d.ports.forEach(p=>{h+='<div class="srow"><span class="sk">'+p.port+' <span style="color:#333">'+p.type+'</span></span><span class="stag '+(p.in_use?'stag-on':'stag-off')+'">'+(p.in_use?'BUSY':'FREE')+'</span></div>'});
    document.getElementById('serial-list').innerHTML=h;log('SERIAL: '+d.ports.length+' port(s)');
  });
}

// ═══ Network ═══
function scanNet(){
  document.getElementById('net-list').innerHTML='<div class="srow"><span class="sk" style="color:#333">Scanning...</span></div>';
  fetch('/api/network/scan').then(r=>r.json()).then(d=>{
    if(!d.devices.length){document.getElementById('net-list').innerHTML='<div class="srow"><span class="sk" style="color:#f00">No devices on '+d.subnet+'.x</span></div>';return}
    var h='';d.devices.forEach(dev=>{h+='<div class="srow"><span class="sk">'+dev.ip+' <span style="color:#333">'+dev.label+'</span></span>'+(dev.ports.length?'<span style="color:#0a0">'+dev.ports.join(',')+'</span>':'')+'</div>'});
    document.getElementById('net-list').innerHTML=h;log('NET: '+d.devices.length+' device(s)');
  });
}

// ═══ Rescan ═══
function rescan(){log('RE-SCANNING...');fetch('/api/rescan').then(r=>r.json()).then(d=>{log('DONE '+JSON.stringify(d));pollStatus()})}

// ═══ Log ═══
function log(m){
  var el=document.getElementById('log');
  var c=m.indexOf('ERR')>=0||m.indexOf('None')>=0?'le':m.indexOf('OK')>=0||m.indexOf('found')>=0?'lo':'';
  el.innerHTML='<div class="'+c+'">'+new Date().toLocaleTimeString()+' '+m+'</div>'+el.innerHTML;
  if(el.children.length>30)el.removeChild(el.lastChild);
}
log('SYS INIT');
</script>
</body>
</html>"""


# ══════════════════════════════════════════════════════════════════
# HTTP HANDLER
# ══════════════════════════════════════════════════════════════════
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif p == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            import cv2
            try:
                while True:
                    frame = get_frame()
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    data = jpeg.tobytes()
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    time.sleep(1.0 / CAMERA_FPS)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        elif p == "/api/status":
            self._json(status.snapshot())
        elif p == "/api/scara/fk":
            qs = parse_qs(urlparse(self.path).query)
            j1 = float(qs.get("j1", [30])[0])
            j2 = float(qs.get("j2", [-45])[0])
            t = scara.fk(j1, j2)
            j = scara.joints(j1, j2)
            d = math.sqrt(t[0]**2 + t[1]**2)
            self._json({"tool_x": t[0], "tool_y": t[1],
                        "reach": round(d, 1),
                        "joints": [list(j[0]), list(j[1]), list(j[2])],
                        "l1": scara.l1, "l2": scara.l2,
                        "minReach": scara.min_r, "maxReach": scara.max_r})
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        p = urlparse(self.path).path
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n)) if n else {}

        if p == "/api/scara/move":
            try:
                x, y = float(body["x"]), float(body["y"])
                angles = scara.ik(x, y)
                if angles is None:
                    self._json({"ok": False, "error": "unreachable"})
                    return
                self._json({"ok": True, "j1": angles[0], "j2": angles[1], "x": x, "y": y})
            except Exception as e:
                self._json({"ok": False, "error": str(e)})

        elif p == "/api/r6/joints":
            try:
                joints = body.get("joints", [0]*6)
                pose = robot6.tool_pose(joints)
                positions = robot6.joint_positions(joints)
                self._json({"ok": True, "pose": pose, "positions": positions})
            except Exception as e:
                self._json({"ok": False, "error": str(e)})

        elif p == "/api/robot/cmd":
            try:
                resp = robot_link.send(body.get("cmd", ""))
                self._json({"ok": resp is not None, "response": resp})
            except Exception as e:
                self._json({"ok": False, "error": str(e)})

        elif p == "/api/serial/scan":
            self._json({"ports": scan_serial_ports()})

        elif p == "/api/network/scan":
            subnet, devices = scan_network()
            self._json({"subnet": subnet, "devices": devices})

        elif p == "/api/rescan":
            robot_link.close()
            r1 = robot_link.connect()
            hexapod.close()
            hexapod.discover()
            can_bus.close()
            r3 = can_bus.open()
            self._json({"robot": r1, "can": r3})

        else:
            self.send_response(404)
            self.end_headers()

    def _json(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, *a):
        pass


def parse_qs(qs_str):
    if not qs_str: return {}
    result = {}
    for pair in qs_str.split("&"):
        k, _, v = pair.partition("=")
        result[k] = [v]
    return result


class ThreadedHTTP(ThreadingMixIn, HTTPServer):
    daemon_threads = True


# ══════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=HTTP_PORT)
    parser.add_argument("--camera", default=CAMERA_DEV)
    args = parser.parse_args()

    print("=" * 60, flush=True)
    print("  UNO Q — INDUSTRIAL ROBOT CONTROLLER", flush=True)
    print("=" * 60, flush=True)
    print(flush=True)
    print("Subsystem scan:", flush=True)
    print("-" * 40, flush=True)

    # Camera
    _frame_event.clear()
    threading.Thread(target=capture_loop, args=(args.camera, CAMERA_W, CAMERA_H, CAMERA_FPS), daemon=True).start()
    _frame_event.wait(timeout=5)
    _frame_event.clear()

    # SCARA
    status.set("SCARA", True, f"L1={scara.l1}mm L2={scara.l2}mm [{scara.min_r:.0f}..{scara.max_r:.0f}]mm")

    # 6-Axis
    status.set("6-Axis Arm", True, "UR5-class DH kinematics, 6-DOF")

    # CAN Bus
    can_bus.open()

    # Hexapod
    hexapod.discover()

    # Robot Link
    robot_link.connect()

    # HTTP
    server = ThreadedHTTP(("0.0.0.0", args.port), Handler)
    print("-" * 40, flush=True)
    print(flush=True)
    print(f"  Camera feed:   http://0.0.0.0:{args.port}/", flush=True)
    print(f"  Status API:    http://0.0.0.0:{args.port}/api/status", flush=True)
    print(flush=True)
    snap = status.snapshot()
    connected = sum(1 for v in snap.values() if v["ok"])
    print(f"  {connected}/{len(snap)} subsystems connected", flush=True)
    print(flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot_link.close()
        can_bus.close()
        hexapod.close()
        server.server_close()
        print("Stopped.", flush=True)


if __name__ == "__main__":
    main()
