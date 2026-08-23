# Parallel SCARA Robot Arm — Hardware

## Overview

| Component | Details |
|---|---|
| Controller | Arduino Uno Q (aarch64, Debian Linux) |
| Motors | 2× harmonic drive motors (CAN bus) |
| CAN Interface | MCP2515 CAN module (SPI) |
| Camera | Logitech C920 webcam (USB, 640×480 @ 15fps) |
| Feedback | Optional hexapod potentiometers (joint position) |
| Link Lengths | L1 = 300mm, L2 = 390mm |
| Workspace | 90mm – 690mm reach |

## Architecture

```
┌─────────────────────────────────────┐
│           HOST (PC)                 │
│    USB Serial / Network             │
└───────────────┬─────────────────────┘
                │
┌───────────────┴─────────────────────┐
│        ARDUINO UNO Q                │
│        (STM32U585 / aarch64)        │
│                                     │
│  SPI: MCP2515 CAN controller       │
│    └── CAN Bus (500 kbps)          │
│        ├── Motor L1 (ID 0x10)      │
│        └── Motor L2 (ID 0x11)      │
│                                     │
│  USB: C920 Webcam (video feed)      │
│  Serial: Debug / commands           │
└─────────────────────────────────────┘
```

## CAN Bus

| Parameter | Value |
|---|---|
| Bit Rate | 500 kbps |
| Controller | MCP2515 (SPI) |
| Crystal | 8 MHz |
| CS Pin | D10 |
| Protocol | Position command (4-byte float, little-endian) |

### Motor CAN IDs

| Motor | CAN ID | Joint | Link |
|---|---|---|---|
| Harmonic Drive #1 | 0x10 | J1 (Base rotation) | L1 = 300mm |
| Harmonic Drive #2 | 0x11 | J2 (Elbow) | L2 = 390mm |

### CAN Message Format

```c
// Position command
struct can_msg {
    uint32_t id;       // Motor CAN ID (0x10 or 0x11)
    uint8_t data[8];
    // data[0..3] = float angle_deg (little-endian, struct.pack("<f", angle))
};
```

## Kinematics

### Forward Kinematics

Given joint angles (θ1, θ2), compute tool position (x, y):

```
x = L1·cos(θ1) + L2·cos(θ1 + θ2)
y = L1·sin(θ1) + L2·sin(θ1 + θ2)
```

### Inverse Kinematics

Given target position (x, y), compute joint angles:

```
d = √(x² + y²)
cos(θ2) = (d² - L1² - L2²) / (2·L1·L2)
θ2 = acos(cos(θ2))
θ1 = atan2(y, x) - atan2(L2·sin(θ2), L1 + L2·cos(θ2))
```

### Workspace

| Parameter | Value |
|---|---|
| L1 | 300 mm |
| L2 | 390 mm |
| Min reach | |L1 - L2| = 90 mm |
| Max reach | L1 + L2 = 690 mm |
| Reachable area | Annular ring (90mm – 690mm radius) |

```
        L1=300mm        L2=390mm
  Base ═══════════ Elbow ═══════════ Tool
  (0,0)           (x1,y1)         (x,y)
       θ1 ↻              θ2 ↻
```

## Pin Map (Arduino Uno Q)

### MCP2515 CAN Module → Arduino

```
MCP2515 Module          Arduino
──────────────          ───────
VCC          ────→      3.3V
GND          ────→      GND
CS           ────→      D10
SO (MISO)    ←────       D12
SI (MOSI)    ────→      D11
SCK          ────→      D13
INT          ────→      D2 (optional, interrupt)

CANH        ────→      CAN Bus High
CANL        ────→      CAN Bus Low
```

### CAN Bus (Twisted Pair)

```
MCP2515                    Motor Controllers
────────                   ─────────────────
CANH ────[twisted pair]───→ Motor L1 CANH
CANL ────[twisted pair]───→ Motor L1 CANL
      │
      └────[twisted pair]──→ Motor L2 CANH
                        └──→ Motor L2 CANL

⚠  120Ω termination resistor at EACH end of the CAN bus
   (usually built into the MCP2515 and last motor module)
```

### C920 Webcam

```
C920 USB ──────→ Uno Q USB Host Port
                 (auto-detected as /dev/video2)
```

Camera specs:
- Resolution: 640×480 (default), up to 1920×1080
- FPS: 15 (at 640×480)
- Format: MJPEG (compressed JPEG frames)
- Connection: USB 2.0

## Power Distribution

```
12V DC PSU
  ├── Motor Supply → Harmonic drive motors (CAN bus powered)
  └── 5V Buck → Arduino Uno Q (via barrel jack or USB)

⚠  Harmonic drives may need higher voltage/current.
   Check motor specs before connecting.
```

## Serial Commands

```
GO <x> <y>       Move to position (mm) — uses inverse kinematics
JOINT <j1> <j2>  Move joints directly (degrees)
STATUS            Report current position (J1, J2, X, Y)
HOME              Return to home position (J1=0, J2=0)
```

## Integration with Uno Q Controller

The SCARA arm integrates with the web-based controller (`tools/unoq_app.py`):
- **Canvas visualization** of arm position in real-time
- **IK solver** computes joint angles from x,y input
- **CAN bus commands** sent to harmonic drive motors
- **Camera feed** for object detection and tracking
- **Combined camera + SCARA** for pick-and-place automation
