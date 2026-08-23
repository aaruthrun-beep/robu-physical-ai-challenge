# Nite369 — Wiring Guide

## Master Pico Wiring

### W5500 Ethernet Module → Master Pico

```
W5500 Module          Master Pico (RP2040)
─────────────         ────────────────────
VCC          ────→    VSYS (5V)
GND          ────→    GND
RST          ────→    GP15
MISO         ────→    GP16
SCK          ────→    GP18
MOSI         ────→    GP19
SCS (CS)     ────→    GP17
```

### MCP23017 GPIO Expander → Master Pico

```
MCP23017             Master Pico (RP2040)
─────────            ────────────────────
VDD          ────→   3V3
VSS          ────→   GND
SDA          ────→   GP0 (with 4.7kΩ pull-up to 3V3)
SCL          ────→   GP1 (with 4.7kΩ pull-up to 3V3)
A0-A2        ────→   GND (address = 0x20)
INTA/INTB    ────→   NC (polled via I2C)

MCP23017 Port B outputs (active HIGH → LED ON):
  GPB0 → 330Ω → LED_POWER (green)  → GND
  GPB1 → 330Ω → LED_MODE (yellow)  → GND
  GPB2 → 330Ω → LED_LINK (green)   → GND
  GPB3 → 330Ω → LED_FAULT (red)    → GND

MCP23017 Port A input:
  GPA4 ← DIP_MODE switch ← GND (active LOW = USB mode)
```

### Slave 1 & Slave 2 → Master Pico (SPI1)

```
Master Pico (RP2040)          Slave 1 Pico              Slave 2 Pico
───────────────────          ──────────────             ──────────────
GP10 (SCK)  ──────────────── GP2 (bit-bang SCK in)     GP10 (SPI SCK)
GP11 (MOSI) ──────────────── GP4 (bit-bang MOSI in)    GP11 (SPI MOSI)
GP12 (MISO) ←──────────────── GP3 (bit-bang MISO out)   GP12 (SPI MISO)
GP9  (CS0)  ──────────────── GP5 (bit-bang CS)          (NC)
GP13 (CS1)  ──────────────── (NC)                        GP7 (SPI CS)
GP14 (SYNC) ──────────────── GP14                        GP14
```

**Important**: SPI1 runs at 50kHz with CS-per-byte framing. Each byte is
transferred with its own CS assertion (HIGH→LOW→HIGH) because the RP2040
SPI slave only reloads its TX shift register on CS re-assert.

### Sync Pulse

```
GP14 (Master) ──────────→ GP14 (Slave 1)
                   └─────→ GP14 (Slave 2)
```

Pulse: HIGH 5μs after each SPI sync cycle. Used for coordinated motion timing.

## Slave 1 — Arm Base Wiring

### Step/Dir Pins

```
Pico GPIO         TMC2209 Driver        Stepper Motor
─────────         ──────────────        ─────────────
J1:
  GP6 (STEP)  →   STEP            →     NEMA 17 (J1)
  GP7 (DIR)   →   DIR
  GP8 (EN)    →   EN (active LOW)

J2 (Paired — two drivers, same signals):
  GP10 (STEP) →   STEP (Driver A) →     NEMA 23 (J2A)
  GP10 (STEP) →   STEP (Driver B) →     NEMA 23 (J2B)
  GP11 (DIR)  →   DIR  (Driver A)
  GP11 (DIR)  →   DIR  (Driver B)
  GP12 (EN)   →   EN   (Driver A)
  GP12 (EN)   →   EN   (Driver B)

J3:
  GP15 (STEP) →   STEP            →     NEMA 17 (J3)
  GP16 (DIR)  →   DIR
  GP17 (EN)   →   EN (active LOW)
```

### AS5600 Encoders (Slave 1)

```
Pico GPIO         AS5600 Encoder         Notes
─────────         ──────────────         ─────
GP18 (SDA) ──┬──→ SDA (Encoder J1)
             ├──→ SDA (Encoder J2)       I2C addresses differ
             └──→ SDA (Encoder J3)       or use TCA9548A mux

GP19 (SCL) ──┬──→ SCL (Encoder J1)
             ├──→ SCL (Encoder J2)
             └──→ SCL (Encoder J3)
```

AS5600 encoders: VCC=3.3V, GND=GND, SDA/SCL with 4.7kΩ pull-ups.

## Slave 2 — Wrist & Gripper Wiring

### TMC2209 UART Bus

```
Pico GPIO         TMC2209 Drivers (All 4)
─────────         ───────────────────────
GP0 (TX) ──[1kΩ]──┬──→ TMC2209#0 (J4,  ADDR 0x00)
                   ├──→ TMC2209#1 (J5,  ADDR 0x01)
GP1 (RX) ─────────┤──→ TMC2209#2 (J6,  ADDR 0x02)
                   └──→ TMC2209#3 (Grip, ADDR 0x03)

GP28 (EN) ────────┴──→ ALL TMC2209 EN pins (active LOW)
```

### TMC2209 Address Jumpers

| Driver | MS1 | MS2 | Addr | Joint |
|---|---|---|---|---|
| #0 | GND | GND | 0x00 | J4 Forearm Roll |
| #1 | VCC | GND | 0x01 | J5 Wrist Pitch |
| #2 | GND | VCC | 0x02 | J6 Wrist Roll |
| #3 | VCC | VCC | 0x03 | Gripper |

### Per-TMC2209 Wiring

```
TMC2209 Module          Stepper Motor
──────────────          ─────────────
VM (8-28V)    ←──────── 24V PSU
GND           ←──────── PSU GND
GND           ────→     Motor GND
A1, A2        ────→     Motor Coil A
B1, B2        ────→     Motor Coil B
PDN/UART      ←────┬──── Shared UART bus
                  [1kΩ]
                   │
STEP           ←──── GP0 (via resistor network, or UART-only mode)
DIR            ←──── GP1
EN             ←──── GP28
```

**Note**: TMC2209 can run in UART-only mode (no STEP/DIR needed). The Nite369
uses UART for configuration and VACTUAL constant-velocity mode. Microstepping
is controlled via UART registers, not MS1/MS2 pins.

## AS5600 Encoders (Slave 2)

```
Pico GPIO         AS5600 Encoder         Notes
─────────         ──────────────         ─────
GP18 (SDA) ──┬──→ SDA (Encoder J4)
             ├──→ SDA (Encoder J5)
             └──→ SDA (Encoder J6)

GP19 (SCL) ──┬──→ SCL (Encoder J4)
             ├──→ SCL (Encoder J5)
             └──→ SCL (Encoder J6)
```

## Power Wiring

```
24V DC PSU
  ├── VM ─────────→ All TMC2209 VM pins
  ├── GND ────────→ Common ground
  └── 5V Buck Converter
       ├── VCC ──→ Pico ×3 (VSYS/5V)
       ├── 3V3 ──→ MCP23017, AS5600, W5500 (via Pico 3V3)
       └── GND ──→ Common ground

⚠  IMPORTANT: All grounds must be connected together.
   USB GND, 24V PSU GND, and 5V regulator GND share a common rail.
```

## USB Serial (Alternative to Ethernet)

When DIP switch is in USB mode (MCP23017 GPA4 = LOW):
- Master Pico uses USB CDC serial at 115200 baud
- Same commands as Ethernet (GRBL-style config, #M, #JC, etc.)
- No W5500 needed

## Quick Reference — Complete Pin Table

### Master Pico (RP2040)

| GPIO | Function | Connected To |
|---|---|---|
| GP0 | I2C0 SDA | MCP23017 SDA |
| GP1 | I2C0 SCL | MCP23017 SCL |
| GP9 | SPI1 CS0 | Slave 1 (bit-bang CS) |
| GP10 | SPI1 SCK | Slave 1 SCK, Slave 2 SCK |
| GP11 | SPI1 MOSI | Slave 1 MOSI, Slave 2 MOSI |
| GP12 | SPI1 MISO | Slave 1 MISO, Slave 2 MISO |
| GP13 | SPI1 CS1 | Slave 2 CS |
| GP14 | SYNC pulse | Slave 1 + Slave 2 SYNC |
| GP15 | W5500 RST | W5500 RST pin |
| GP16 | SPI0 MISO | W5500 MISO |
| GP17 | SPI0 CS | W5500 SCS |
| GP18 | SPI0 SCK | W5500 SCK |
| GP19 | SPI0 MOSI | W5500 MOSI |
| GP25 | Onboard LED | Status heartbeat (1Hz blink) |
