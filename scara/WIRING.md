# Parallel SCARA — Wiring Guide

## MCP2515 CAN Module → Arduino Uno Q

```
MCP2515 Module              Arduino Uno Q
──────────────              ─────────────
VCC              ────→      3.3V
GND              ────→      GND
CS               ────→      D10 (chip select)
SO (MISO)        ←────       D12 (SPI data in)
SI (MOSI)        ────→      D11 (SPI data out)
SCK              ────→      D13 (SPI clock)
INT              ────→      D2  (interrupt, optional)
```

## CAN Bus — Motor Wiring

```
MCP2515                    Harmonic Drive Motors
────────                   ─────────────────────
CANH ──[twisted pair]──┬──→ Motor L1 CANH (ID 0x10)
CANL ──[twisted pair]──┼──→ Motor L1 CANL
                       │
                       ├──→ Motor L2 CANH (ID 0x11)
                       └──→ Motor L2 CANL
```

### CAN Bus Termination

```
CANH ──[120Ω]── CANL    (at MCP2515 end)
CANH ──[120Ω]── CANL    (at last motor end)

⚠  Always place 120Ω termination at BOTH ends of the bus.
   Without termination, reflections cause communication errors.
```

### CAN Bus Cable

- Use **twisted pair** cable (CAT5e works well)
- Keep bus length under 40m at 500kbps
- Connect shield to GND at one end only (to avoid ground loops)

## Harmonic Drive Motor Wiring

```
Motor Connector          Description
───────────────          ───────────
CANH          ────→      CAN Bus High
CANL          ────→      CAN Bus Low
V+            ←────       Motor power supply (12-48V, check motor spec)
GND           ←────       Common ground
```

### Motor CAN ID Configuration

| Motor | CAN ID | Joint | Link |
|---|---|---|---|
| Harmonic Drive #1 | 0x10 | J1 (Base) | L1 = 300mm |
| Harmonic Drive #2 | 0x11 | J2 (Elbow) | L2 = 390mm |

Motor CAN IDs are typically set via:
- DIP switches on the motor driver board
- Software configuration over CAN
- Check your motor's documentation for the exact method

## C920 Webcam

```
Logitech C920             Arduino Uno Q
─────────────             ─────────────
USB           ────→       USB Host Port
```

- Auto-detected as `/dev/video2` (Linux)
- Powered via USB (no external power needed)
- Default resolution: 640×480 @ 15fps

## Feedback Potentiometers (Optional)

If using potentiometers for joint position feedback:

```
3.3V ──→ Pot pin 1 (power)
Arduino A0 ←── Pot wiper (pin 2) — J1 feedback
GND ──→ Pot pin 3 (ground)

3.3V ──→ Pot pin 1
Arduino A1 ←── Pot wiper (pin 2) — J2 feedback
GND ──→ Pot pin 3
```

## Power Distribution

```
12V DC PSU (or higher, per motor spec)
  │
  ├── Motor V+ ──→ Harmonic Drive #1 V+
  │                 Harmonic Drive #2 V+
  │
  ├── 5V Buck ──→ Arduino Uno Q (barrel jack or VIN)
  │
  └── GND ──────→ Common ground (MCP2515, Motors, Arduino)

⚠  IMPORTANT:
   - Harmonic drives draw significant current (check motor datasheet)
   - Use appropriately rated power supply and wiring
   - All grounds must be connected together
   - Keep motor power wires separate from signal wires
     (to avoid EMI on CAN bus)
```

## Complete Wiring Diagram

```
                    ┌──────────────┐
                    │  ARDUINO     │
                    │  UNO Q       │
                    │              │
                    │  D10 (CS) ───┼──→ MCP2515 CS
                    │  D11 (MOSI)──┼──→ MCP2515 SI
                    │  D12 (MISO)←─┼──── MCP2515 SO
                    │  D13 (SCK) ──┼──→ MCP2515 SCK
                    │              │
                    │  D2 (INT) ←──┼──── MCP2515 INT
                    │              │
                    │  USB ────────┼──→ C920 Webcam
                    │              │
                    │  3.3V ───────┼──→ MCP2515 VCC
                    │  GND ────────┼──→ MCP2515 GND
                    │              │
                    │  Barrel Jack ←── 5V from buck converter
                    └──────────────┘
                           │
                    ┌──────┴──────┐
                    │  MCP2515    │
                    │  CAN Module │
                    │             │
                    │  CANH ──────┼──→ CAN Bus High
                    │  CANL ──────┼──→ CAN Bus Low
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
         ┌────┴────┐  ┌───┴───┐    ┌───┴───┐
         │ Motor L1│  │Motor L2│    │ 120Ω  │
         │ (0x10)  │  │(0x11)  │    │ term. │
         │ J1 Base │  │J2 Elbow│    └───────┘
         │ L1=300mm│  │L2=390mm│
         └────┬────┘  └───┬───┘
              │            │
         ┌────┴────┐  ┌───┴───┐
         │  120Ω   │  │       │
         │  term.  │  │       │
         └─────────┘  └───────┘
```

## Serial Debug Output

Connect to the Arduino's serial console (115200 baud) to see:
- Boot messages (`CAN OK`, `SCARA READY`)
- Command responses (`OK`, `ERR UNREACHABLE`)
- Position reports (`J1=45.2 J2=30.1 X=520.3 Y=180.7`)

## Debugging Tips

1. **No CAN communication**: Check 120Ω termination at both ends
2. **Motor doesn't move**: Verify CAN ID matches (0x10/0x11)
3. **Wrong direction**: Check CANH/CANL aren't swapped
4. **Camera not found**: Check `ls /dev/video*` — C920 may be video0, video1, or video2
5. **IK unreachable**: Position must be within 90mm–690mm from base
