# Robu Physical AI Challenge 2026

Three robot projects built with Arduino UNO Q for the
[Arduino Physical AI Challenge India 2026](https://robu.in/arduino-physical-ai-challenge-india-2026/).

## Projects

### 1. Nite369 — 6-Axis Industrial Robot Arm

Open-source 6-axis robot arm with RP2040 Pico multi-core architecture,
TMC2209 stepper drivers, W5500 Ethernet, and Astra Studio GUI.

| Component | Details |
|---|---|
| Controller | 3x RP2040 Pico (Master + Slave 1 + Slave 2) |
| Drivers | TMC2209 UART stepper drivers (1 per joint) |
| Encoders | AS5600 magnetic encoders (closed-loop) |
| Network | W5500 Ethernet TCP/UDP |
| Software | Astra Studio (Python/PyQt5) + Uno Q web controller |
| Kinematics | Full FK/IK, DH parameters, path planning |

**Location:** `nite369/`

```
nite369/
├── firmware/          # RP2040 C firmware (CMake + PIO)
├── nite369_v2/        # Multi-Pico version (master + 2 slaves)
├── studio/            # Astra Studio Python GUI
└── tools/             # TCP client, IK solver, Uno Q controller
```

### 2. XM9X6 — Heads of Heart Hexapod

6-legged walking robot with 3 DOF per leg (18 servos), each leg has its own
microcontroller. Supports wave, trot, and ripple gaits.

| Component | Details |
|---|---|
| Legs | 6x, 3 DOF each (coxa, femur, tibia) |
| Servos | 18x servo motors |
| MCU | 1x coordinator (Uno Q) + 6x leg controllers |
| Sensors | IMU (balance), ToF (obstacle avoidance) |
| Gaits | Wave, trot, ripple |

**Location:** `hexapod/`

```
hexapod/
└── xm9x6/
    ├── firmware/      # Leg controller sketches
    ├── README.md      # Hardware + protocol docs
    └── tools/         # (coming soon)
```

### 3. Parallel SCARA Robot Arm

2-link parallel SCARA arm with two harmonic drive motors on CAN bus.
Links: L1 = 300mm, L2 = 390mm. Reach: 90–690mm.

| Component | Details |
|---|---|
| Links | L1 = 300mm, L2 = 390mm |
| Motors | 2x harmonic drive (CAN bus, IDs 0x10, 0x11) |
| Controller | Arduino Uno Q + MCP2515 CAN module |
| Camera | C920 webcam (object detection + tracking) |
| Kinematics | FK/IK with elbow-up solution |

**Location:** `scara/`

```
scara/
└── parallel/
    ├── firmware/      # SCARA controller sketch
    ├── README.md      # Kinematics + protocol docs
    └── tools/         # (coming soon)
```

## Hardware Overview

```
                    ┌─────────────┐
                    │  Arduino    │
                    │  UNO Q      │
                    │  (aarch64)  │
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
        ┌─────┴─────┐ ┌───┴───┐ ┌─────┴─────┐
        │  Nite369  │ │XM9X6  │ │  SCARA    │
        │  6-Axis   │ │Hexapod│ │  Parallel │
        │  Robot    │ │       │ │  Arm      │
        └───────────┘ └───────┘ └───────────┘
```

## Competition

- **Event:** Arduino Physical AI Challenge India 2026
- **Platform:** Arduino UNO Q
- **Status:** Active development

## License

Custom — Nite369 Robot Project
