# Nite369 — 6-Axis Robot Arm Hardware

## Overview

| Component | Details |
|---|---|
| Controller | 3× RP2040 Pico (Master + Slave 1 + Slave 2) |
| Drivers | TMC2209 UART stepper drivers (6 axes + gripper) |
| Encoders | AS5600 magnetic encoders (closed-loop) |
| Network | W5500 Ethernet TCP/UDP |
| I/O Expander | MCP23017 I2C GPIO (LEDs, DIP switch) |
| Power | 24V DC (stepper drivers), 5V (logic/encoders) |

## Architecture

```
┌──────────────────────────────────────────────────────┐
│                   HOST (PC / Uno Q)                   │
│          Ethernet UDP:5000  or  USB Serial            │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────┐
│                 MASTER PICO (RP2040)                  │
│  ┌─────────┐  ┌──────────┐  ┌────────────────────┐  │
│  │ W5500   │  │ MCP23017 │  │ SPI1 → Slaves      │  │
│  │ Ethernet│  │ GPIO Exp │  │ CS0→S1, CS1→S2     │  │
│  └────┬────┘  └────┬─────┘  └────────┬───────────┘  │
│       │             │                 │               │
│    SPI0:GP15-19  I2C0:GP0-1     SPI1:GP9-13         │
└───────┼─────────────┼─────────────┼──────────────────┘
        │             │             │
   Ethernet       LEDs/DIP    ┌────┴────┐
   UDP:5000                   │         │
                         ┌────┴──┐  ┌──┴────┐
                         │SLAVE 1│  │SLAVE 2│
                         │ J1-J3 │  │J4-J6+G│
                         └───────┘  └───────┘
```

## Master Pico Pin Map

### W5500 Ethernet (SPI0)

| Pin | GPIO | Function |
|---|---|---|
| RST | GP15 | W5500 hardware reset |
| MISO | GP16 | SPI0 data in |
| CS | GP17 | SPI0 chip select |
| SCK | GP18 | SPI0 clock |
| MOSI | GP19 | SPI0 data out |

### Slave Communication (SPI1)

| Pin | GPIO | Function |
|---|---|---|
| SCK | GP10 | SPI1 clock |
| MOSI | GP11 | SPI1 data out |
| MISO | GP12 | SPI1 data in |
| CS0 | GP9 | Slave 1 chip select |
| CS1 | GP13 | Slave 2 chip select |
| SYNC | GP14 | Synchronized motion pulse |

### MCP23017 GPIO Expander (I2C0)

| Pin | GPIO | Function |
|---|---|---|
| SDA | GP0 | I2C data |
| SCL | GP1 | I2C clock |

MCP23017 I2C address: `0x20`

| MCP Pin | Function |
|---|---|
| GPA0 | LED_POWER (green) |
| GPA1 | LED_MODE (yellow) |
| GPA2 | LED_LINK (green, UDP activity) |
| GPA3 | LED_FAULT (red, error/E-stop) |
| GPB4 | DIP_MODE (USB vs LAN select) |

### Network Config (Default)

| Parameter | Value |
|---|---|
| IP | 192.168.1.100 |
| Subnet | 255.255.255.0 |
| Gateway | 192.168.1.1 |
| DNS | 8.8.8.8 |
| MAC | 00:08:DC:11:22:33 |
| UDP Port | 5000 (configurable via `$MASTER.port`) |
| Protocol | UDP (binary `astra_udp_cmd_t` / text GRBL-style) |

## Slave 1 — Arm Base (J1, J2, J3)

| Axis | Joint | Motor Type | Encoder |
|---|---|---|---|
| 0 | J1 (Base Rotate) | Stepper + TMC2209 | AS5600 |
| 1 | J2A+J2B (Shoulder) | 2× Stepper (paired) | AS5600 |
| 2 | J3 (Elbow) | Stepper + TMC2209 | AS5600 |

### Slave 1 Pin Map

| Pin | GPIO | Function |
|---|---|---|
| SPI SCK | GP2 | Bit-bang SPI input |
| SPI MISO | GP3 | Bit-bang SPI output |
| SPI MOSI | GP4 | Bit-bang SPI input |
| SPI CS | GP5 | Bit-bang SPI chip select |

### Step/Dir Pins (Slave 1)

| Joint | STEP | DIR | EN |
|---|---|---|---|
| J1 | GP6 | GP7 | GP8 |
| J2 (paired) | GP10/GP11 | GP12/GP13 | GP14 |
| J3 | GP15 | GP16 | GP17 |

### AS5600 Encoders (Slave 1)

I2C bus (bit-bang) on GP18 (SDA) / GP19 (SCL):
- Encoder 0 (J1): Address `0x36`
- Encoder 1 (J2): Address `0x37` (or via TCA9548A mux)
- Encoder 2 (J3): Address `0x38`

## Slave 2 — Wrist & Gripper (J4, J5, J6, Gripper)

### TMC2209 Drivers (Shared UART Bus)

| Driver | Addr | Joint | MS1 | MS2 |
|---|---|---|---|---|
| TMC2209#0 | 0x00 | J4 (Forearm Roll) | GND | GND |
| TMC2209#1 | 0x01 | J5 (Wrist Pitch) | VCC | GND |
| TMC2209#2 | 0x02 | J6 (Wrist Roll) | GND | VCC |
| TMC2209#3 | 0x03 | Gripper | VCC | VCC |

### Slave 2 Pin Map

| Pin | GPIO | Function |
|---|---|---|
| UART TX | GP0 | Half-duplex UART (115200 baud) |
| UART RX | GP1 | Half-duplex UART (shared TX/RX via 1kΩ) |
| ENABLE | GP28 | All TMC2209 EN pins (active LOW) |

### TMC2209 UART Wiring

```
GP0 (UART TX) ──[1kΩ]──┬── TMC2209#0 (J4,  ADDR 0)
                        ├── TMC2209#1 (J5,  ADDR 1)
GP1 (UART RX) ─────────┤├── TMC2209#2 (J6,  ADDR 2)
                        │└── TMC2209#3 (GRIP, ADDR 3)
GP28 (ENABLE) ─────────┘── All drivers EN (active LOW)
```

Protocol: 115200 baud, 8N1, half-duplex
- Read: `[0x05, addr, reg, CRC]` → `[0x05, 0xFF, reg, d3..d0, CRC]`
- Write: `[0x05, addr|0x80, reg, d3..d0, CRC]`

### TMC2209 Default Config

| Register | Value | Meaning |
|---|---|---|
| GCONF | SpreadCycle + UART microstep | EN_SPREADCYCLE + MSTEP_REG_SELECT |
| CHOPCONF | 1/8 microsteps, 256x interpolation | MRES=3, INTPOL |
| IHOLD_IRUN | IRUN=16, IHOLD=8, delay=4 | ~1.0A run, 50% hold |
| VACTUAL | Controlled via firmware | Direct speed control |

## Stepper Motor Specs (Estimated)

| Joint | Motor | Steps/Rev | Gear Ratio | Microsteps | Steps/Deg |
|---|---|---|---|---|---|
| J1 | NEMA 17 | 200 | TBD | 1/8 | ~4.4 |
| J2A+J2B | NEMA 23 | 200 | TBD | 1/8 | ~8.9 (paired) |
| J3 | NEMA 17 | 200 | TBD | 1/8 | ~4.4 |
| J4 | NEMA 17 | 200 | TBD | 1/8 | ~4.4 |
| J5 | NEMA 17 | 200 | TBD | 1/8 | ~4.4 |
| J6 | NEMA 17 | 200 | TBD | 1/8 | ~4.4 |

*Steps/rev and gear ratios are configurable via `$M92` and stored in flash.*

## Power Distribution

```
24V DC Supply
  ├── Stepper Driver VM (all TMC2209)
  └── 5V Regulator (Buck)
       ├── Pico × 3 (VBUS/VSYS)
       ├── MCP23017 (VCC)
       ├── W5500 (3.3V via Pico)
       └── AS5600 encoders (VCC)
```

## Safety

- **Watchdog timer**: 500μs ISR, 2s heartbeat timeout → emergency stop
- **E-Stop**: Zeroes all targets on watchdog trip
- **Soft limits**: Configurable per-joint position limits
- **DIP switch**: Hardware USB/LAN mode selection via MCP23017
