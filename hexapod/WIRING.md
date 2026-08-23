# XM9X6 — Wiring Guide

## Coordinator Pico → I2C Bus

```
Coordinator Pico (RP2040)        I2C Devices
────────────────────────         ───────────
GP4 (SDA) ──[4.7kΩ→3V3]──┬────→ TCA9548A SDA (0x70)
                          ├────→ PCA9685 #1 SDA (0x40)
                          ├────→ PCA9685 #2 SDA (0x41)
                          └────→ ADS1115 SDA (0x48, via TCA mux)

GP5 (SCL) ──[4.7kΩ→3V3]──┬────→ TCA9548A SCL (0x70)
                          ├────→ PCA9685 #1 SCL (0x40)
                          ├────→ PCA9685 #2 SCL (0x41)
                          └────→ ADS1115 SCL (0x48, via TCA mux)
```

**Important**: All I2C devices share the same bus (GP4/GP5). Pull-ups
(4.7kΩ to 3.3V) are required on both SDA and SCL.

## PCA9685 → Servo Wiring

### PCA9685 #1 (0x40) — Left Side Legs

```
PCA9685 #1               Servos (Left Side)
──────────               ──────────────────
VCC (3.3V) ←── 3.3V PSU
V+ (5-6V)  ←── Servo Supply (5V/6V, high current)
GND        ←── Common GND

Ch15 (PWM) ──→ Leg 1 Coxa (servo signal)
Ch14 (PWM) ──→ Leg 1 Femur
Ch13 (PWM) ──→ Leg 1 Tibia

Ch12 (PWM) ──→ Leg 2 Coxa
Ch11 (PWM) ──→ Leg 2 Femur
Ch10 (PWM) ──→ Leg 2 Tibia

Ch9  (PWM) ──→ Leg 3 Coxa
Ch8  (PWM) ──→ Leg 3 Femur
Ch7  (PWM) ──→ Leg 3 Tibia
```

### PCA9685 #2 (0x41) — Right Side Legs

```
PCA9685 #2               Servos (Right Side)
──────────               ───────────────────
VCC (3.3V) ←── 3.3V PSU
V+ (5-6V)  ←── Servo Supply (5V/6V, high current)
GND        ←── Common GND

Ch31 (PWM) ──→ Leg 4 Coxa (servo signal)
Ch30 (PWM) ──→ Leg 4 Femur
Ch29 (PWM) ──→ Leg 4 Tibia

Ch28 (PWM) ──→ Leg 5 Coxa
Ch27 (PWM) ──→ Leg 5 Femur
Ch26 (PWM) ──→ Leg 5 Tibia

Ch25 (PWM) ──→ Leg 6 Coxa
Ch24 (PWM) ──→ Leg 6 Femur
Ch23 (PWM) ──→ Leg 6 Tibia
```

### Servo Connector Pinout

Each servo has 3 wires:
```
Servo Wire        PCA9685 Pin        Color (typical)
──────────        ───────────        ───────────────
Signal (PWM)  →   PWM (ch pin)       Orange/Yellow
V+ (5-6V)     →   V+ (power pin)    Red
GND           →   GND (power pin)    Brown/Black
```

**⚠  PCA9685 has separate signal (VCC) and power (V+) pins.**
- VCC = 3.3V (logic, from Pico)
- V+ = 5-6V (servo power, separate supply)
- Both GND pins must connect to the common ground

## TCA9548A → ADS1115 Wiring

```
TCA9548A                 ADS1115 (shared address 0x48)
────────                 ─────────────────────────────
Ch0 SDA ───────────────→ ADS1115 SDA (Leg 1-2 pots)
Ch0 SCL ───────────────→ ADS1115 SCL

Ch1 SDA ───────────────→ ADS1115 SDA (Leg 3-4 pots)
Ch1 SCL ───────────────→ ADS1115 SCL

Ch2 SDA ───────────────→ ADS1115 SDA (Leg 5-6 pots)
Ch2 SCL ───────────────→ ADS1115 SCL

Ch3 SDA ───────────────→ ADS1115 SDA (spare)
Ch3 SCL ───────────────→ ADS1115 SCL

Ch4 SDA ───────────────→ ADS1115 SDA (IMU/ToF)
Ch4 SCL ───────────────→ ADS1115 SCL
```

### ADS1115 → Potentiometers

```
ADS1115 AIN0 ←── Leg 1 Coxa pot (10kΩ)
ADS1115 AIN1 ←── Leg 1 Femur pot
ADS1115 AIN2 ←── Leg 1 Tibia pot
ADS1115 AIN3 ←── Leg 2 Coxa pot

(each TCA channel reads 4 pots via ADS1115 AIN0-AIN3)
```

Pot wiring:
```
3.3V ──→ Pot pin 1
Pico ADC (AIN) ←── Pot wiper (pin 2)
GND ──→ Pot pin 3
```

## Power Distribution

```
USB (5V, from PC)         Servo Supply (5V/6V, 5-10A)
    │                          │
    ▼                          ▼
┌────────┐               ┌──────────┐
│ Pico   │               │ PCA9685  │
│ VBUS   │               │ V+ pins  │
│        │               │ (all 18  │
│ 3V3 out│──→ Logic      │  servos) │
│        │    (TCA, ADS, │          │
└────────┘    PCA VCC)   └──────────┘
    │                          │
    └──────────┬───────────────┘
               │
            Common GND
```

**⚠  CRITICAL**:
- Servos can draw 1-2A each under load (up to 36A peak for 18 servos)
- Use a **dedicated 5V/6V supply** rated for the total servo load
- Do NOT power servos from the Pico or USB
- All grounds (Pico, servos, sensors) MUST be connected together

## Complete Wiring Diagram

```
                    ┌──────────────┐
                    │  COORDINATOR │
                    │  PICO        │
                    │  (RP2040)    │
                    │              │
                    │  GP4 (SDA)──┼──┬──→ TCA9548A SDA
                    │  GP5 (SCL)──┼──┼──→ TCA9548A SCL
                    │              │  │
                    │  GP25 (LED)  │  ├──→ PCA9685 #1 SDA/SCL
                    │              │  └──→ PCA9685 #2 SDA/SCL
                    │  USB ───────┼────→ PC (serial console)
                    │  3V3 ───────┼────→ All device VCC
                    │  GND ───────┼────→ Common ground
                    └──────────────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
     ┌────────┴───┐ ┌─────┴─────┐ ┌───┴────────┐
     │ TCA9548A   │ │ PCA9685   │ │ PCA9685    │
     │ I2C Mux    │ │ #1 (0x40) │ │ #2 (0x41)  │
     │ (0x70)     │ │           │ │            │
     └──┬──┬──┬──┘ │ Ch7-15    │ │ Ch23-31    │
        │  │  │     │ Legs 1-3  │ │ Legs 4-6   │
     ┌──┘  │  └──┐  └─────┬─────┘ └──┬─────────┘
     │     │     │         │          │
  Ch0 Ch1 Ch2  Ch3      Servos     Servos
     │     │     │       (9×)      (9×)
  ┌──┘     └──┐  │
  │           │  │
ADS1115    ADS1115 ADS1115
(pots L1-2) (L3-4) (L5-6)
```

## Serial Protocol (Coordinator ↔ Host)

```
Host → Coordinator (USB Serial, 115200 baud):
  C<ch> <angle>     Set servo channel (0-31, 0-180°)
  H                  Home all to 90°
  A <angle>          All servos to angle
  R                  Report positions
  P                  Ping / status handshake
  D                  Servo diagnostic
  scan               I2C device scan
  adc                Read all ADC channels
  F                  Fast ADC feedback (20 pots)
  home               All legs to HOME pose
  pose <name>        All legs to pose (home|stand|sit|lift|crouch)
  leg <n> <pose>     One leg to pose (1-6)
  cal                Show calibration
  cal <ch> <off>     Set calibration offset
  mirror <1|2|all>   Flip channel order for upside-down boards

Coordinator → Host:
  READY              Boot complete, awaiting commands
  OK C<ch> <angle>   Servo command acknowledged
  POS <32 values>    Current positions (R command)
  FB <20 raw values> Fast ADC feedback (F command)
  PONG               Response to ping
```
