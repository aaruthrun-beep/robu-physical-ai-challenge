# XM9X6 — Heads of Heart Hexapod Hardware

## Overview

| Component | Details |
|---|---|
| Coordinator | Arduino Pico (RP2040) — USB serial to host |
| Legs | 6×, 3 DOF each (coxa, femur, tibia) = 18 servos |
| Servo Drivers | 2× PCA9685 (16 channels each) |
| ADC | ADS1115 16-bit ADC via TCA9548A I2C mux |
| Feedback | 18× potentiometers (joint position feedback) |
| Sensors | IMU (balance), ToF (obstacle avoidance) |
| Gaits | Wave, trot, ripple |
| Power | 5V/6V servo supply, 3.3V logic |

## Architecture

```
┌─────────────────────────────────────┐
│         HOST (PC / Uno Q)           │
│      USB Serial 115200 baud         │
└───────────────┬─────────────────────┘
                │
┌───────────────┴─────────────────────┐
│       COORDINATOR PICO (RP2040)     │
│                                     │
│  I2C0: GP4(SDA), GP5(SCL)          │
│    ├── TCA9548A I2C Mux (0x70)     │
│    │   ├── Ch0: ADS1115 (0x48)     │
│    │   ├── Ch1: ADS1115 (0x48)     │
│    │   ├── Ch2: ADS1115 (0x48)     │
│    │   ├── Ch3: ADS1115 (0x48)     │
│    │   └── Ch4: ADS1115 (0x48)     │
│    ├── PCA9685 #1 (0x40) — Legs 1-3│
│    └── PCA9685 #2 (0x41) — Legs 4-6│
└─────────────────────────────────────┘
```

## I2C Bus

| Device | Address | Bus | Function |
|---|---|---|---|
| TCA9548A | 0x70 | I2C0 main | 8-channel I2C mux |
| PCA9685 #1 | 0x40 | I2C0 main | Servo driver (Legs 1-3) |
| PCA9685 #2 | 0x41 | I2C0 main | Servo driver (Legs 4-6) |
| ADS1115 | 0x48 | Via TCA9548A | 16-bit ADC (pot feedback) |

### TCA9548A Channel Mapping

| TCA Channel | Connected To | Purpose |
|---|---|---|
| Ch 0 | ADS1115 #0 | Leg 1-2 pots (4 AIN) |
| Ch 1 | ADS1115 #1 | Leg 3-4 pots (4 AIN) |
| Ch 2 | ADS1115 #2 | Leg 5-6 pots (4 AIN) |
| Ch 3 | ADS1115 #3 | Spare / extra sensors |
| Ch 4 | ADS1115 #4 | Spare / IMU / ToF |

## PCA9685 Servo Channel Mapping

### Physical Wiring (with mirror ALL enabled)

```
PCA9685 #1 (0x40) — Left Side Legs
──────────────────────────────────────
Channel 15, 14, 13  →  Leg 1 (coxa, femur, tibia)
Channel 12, 11, 10  →  Leg 2 (coxa, femur, tibia)
Channel  9,  8,  7  →  Leg 3 (coxa, femur, tibia)

PCA9685 #2 (0x41) — Right Side Legs
──────────────────────────────────────
Channel 31, 30, 29  →  Leg 4 (coxa, femur, tibia)
Channel 28, 27, 26  →  Leg 5 (coxa, femur, tibia)
Channel 25, 24, 23  →  Leg 6 (coxa, femur, tibia)
```

### Servo Pulse Range

| Parameter | Value |
|---|---|
| PWM Frequency | 50 Hz (prescaler = 121) |
| Min Pulse | 102 (~0.5 ms) |
| Max Pulse | 560 (~2.7 ms) |
| Neutral (90°) | ~307 |
| Angle Range | 0°–180° (GUI), 0°–140° (effective) |

## Leg Geometry

```
        ┌─── Coxa (hip)
        │    Length: ~48mm
   ═════╪═════
        │
        ├─── Femur (thigh)
        │    Length: ~80mm
        │
        ├─── Tibia (shin)
             Length: ~147mm
             ↓
           Foot (ground contact)
```

| Parameter | Value |
|---|---|
| Coxa Length | 0.04823 m (48.23 mm) |
| Femur Length | 0.079924 m (79.92 mm) |
| Tibia Length | 0.1465 m (146.5 mm) |
| Hip Mount Yaw | 0°, -60°, -120°, 180°, 120°, 60° |
| Hip Z Height | 30.6 mm above base |

## Joint Limits (URDF)

| Joint | Min | Max | Notes |
|---|---|---|---|
| Hip | -90° | +90° | ±1.57 rad |
| Femur | -150° | +30° | Negative = upward (inverted) |
| Tibia | -5° | +175° | Nearly full range |

## Pose Definitions

| Pose | Coxa | Femur | Tibia | Description |
|---|---|---|---|---|
| HOME | 90° | 70° | 110° | Standing center reference |
| STAND | 90° | 135° | 100° | Body raised, legs extended |
| SIT | 90° | 70° | 160° | Legs folded, body low |
| LIFT | 90° | 160° | 60° | Leg raised off ground |
| CROUCH | 90° | 100° | 130° | Low stance |

## Pin Map (Coordinator Pico)

| GPIO | Function | Connected To |
|---|---|---|
| GP4 | I2C0 SDA | TCA9548A SDA, PCA9685 SDA, ADS1115 SDA |
| GP5 | I2C0 SCL | TCA9548A SCL, PCA9685 SCL, ADS1115 SCL |
| GP25 | Onboard LED | Status heartbeat |

## Per-Leg Controller (Arduino, Optional)

If using separate Arduino boards per leg:

| Pin | Function |
|---|---|
| D9 | Coxa servo (Servo library) |
| D10 | Femur servo |
| D11 | Tibia servo |
| Serial | 115200 baud, command bus from coordinator |

Protocol: `L<leg> J<joint> A<angle>` (e.g., `L1 J2 A90`)

## Power Distribution

```
Servo Supply (5V/6V, 5A+)
  ├── PCA9685 V+ (both boards)
  └── 18× servos via PCA9685 VCC/GND pins

Logic Supply (3.3V)
  ├── Pico (via VSYS)
  ├── TCA9548A (VCC)
  ├── PCA9685 (VCC)
  └── ADS1115 (VCC)

⚠  Servos draw significant current (up to 1A each under load).
   Use a separate high-current 5V/6V supply for servos.
   Do NOT power servos from the Pico's 3.3V or USB 5V.
```

## Calibration

Each servo channel has:
- **Offset**: Added to commanded angle (calibration offset, ±90°)
- **Min limit**: Minimum effective angle (0–200 range)
- **Max limit**: Maximum effective angle (0–200 range)

The effective range 0–200 is wider than 0–180 to allow offset joints to use their full commanded travel.
