# Parallel SCARA Robot Arm

2-link parallel SCARA arm with two harmonic drive motors on CAN bus.
Links: L1 = 300mm, L2 = 390mm. Workspace reach: 90–690mm.

## Hardware

- **2x harmonic drive motors** (CAN bus, IDs 0x10, 0x11)
- **1x Arduino Uno Q** (coordinator, CAN + camera)
- **1x MCP2515 CAN module** (or native CAN on Uno Q)
- **1x C920 webcam** (object detection + tracking)
- **1x hexapod potentiometer** (joint feedback, optional)

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
- **Reach**: 90mm (min) to 690mm (max)
- **L1**: 300mm
- **L2**: 390mm

## Control Protocol (CAN Bus)

```c
// Position command (degrees)
struct can_msg {
    uint32_t id;       // Motor ID (0x10 = L1, 0x11 = L2)
    float angle_deg;   // Target angle in degrees
    uint8_t data[4];   // struct.pack("<f", angle_deg)
};
```

## Integration

The SCARA arm integrates with:
- **Uno Q controller** (`tools/unoq_app.py`) — web UI with canvas visualization
- **Astra Studio** — full kinematics + path planning
- **Camera tracking** — HSV color histogram object detection → SCARA automation

## Status

- [x] Forward kinematics
- [x] Inverse kinematics
- [x] Web UI visualization (Uno Q controller)
- [ ] CAN bus motor control
- [ ] Object tracking integration
- [ ] Path planning
- [ ] Calibration
