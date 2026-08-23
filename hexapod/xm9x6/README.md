# XM9X6 — Heads of Heart Hexapod

6-legged walking robot with 3 DOF per leg (coxa, femur, tibia) = 18 servos total.
Each leg has its own microcontroller, coordinated via serial bus.

## Hardware

- **18x servo motors** (3 per leg)
- **6x microcontrollers** (one per leg, STM32/ESP32)
- **1x coordinator** (Arduino Uno Q or Raspberry Pi)
- **IMU** (MPU6050/ICM-20948) for balance
- **Ultrasonic/ToF sensors** for obstacle avoidance

## Architecture

```
Coordinator (Uno Q)
  ├── Leg 1 (MCU) ── Serial Bus ── Servos 1-3
  ├── Leg 2 (MCU) ── Serial Bus ── Servos 4-6
  ├── Leg 3 (MCU) ── Serial Bus ── Servos 7-9
  ├── Leg 4 (MCU) ── Serial Bus ── Servos 10-12
  ├── Leg 5 (MCU) ── Serial Bus ── Servos 13-15
  └── Leg 6 (MCU) ── Serial Bus ── Servos 16-18
```

## Gaits

- **Wave gait** — one leg moves at a time (most stable)
- **Trot gait** — diagonal pairs move together (fast)
- **Ripple gait** — overlapping wave (smooth)

## Control Protocol

```
L<leg> J<joint> A<angle>    — set joint angle
STATUS                        — query all legs
HOME                          — return to home position
GAIT <wave|trot|ripple>       — set gait pattern
```

## Status

- [ ] Firmware (leg MCU)
- [ ] Coordinator firmware
- [ ] Serial bus protocol
- [ ] Gait engine
- [ ] IMU integration
- [ ] Obstacle avoidance
- [ ] Uno Q controller integration
