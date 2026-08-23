# nite369_v2 — Restructured Nite369 Firmware

The v2 firmware tree. The single structural change vs v1: **the SPI frame
format now lives in one place** — `shared/spi_protocol/` — and the master
and both slaves include it directly instead of each carrying their own
copy of the protocol constants.

```
nite369_v2/
├── shared/spi_protocol/       ← SINGLE SOURCE OF TRUTH for the SPI frame
│   ├── frame.h                  format: 9-byte frames, CRC8, opcodes,
│   ├── frame.c                  v1 pack/unpack + v2 structured format
│   ├── common_types.h           joint IDs, status codes, fault codes, cfg fields
│   └── protocol_spec.md         human-readable wire spec, versioned
├── master/                    ← grblHAL target dir (currently the v1
│   ├── spi_cmd_master_eth.c     W5500 TCP master, moved here)
│   ├── gcode_parser.c/h
│   ├── w5500_tcp.c/h
│   └── nite_tmc2209.c/h
├── slave_common/              ← shared closed-loop stack, both slaves link
│   ├── motion_profile.c/h       this: motion planner (Marlin trapezoid),
│   ├── motion_marlin.c/h        homing, config_store, TMC UART, WS2812,
│   ├── homing.c/h               + the legacy nite_spi_proto.h shim
│   ├── config_store.c/h
│   ├── tmc_uart.c/h
│   ├── ws2812.c/h
│   └── nite_spi_proto.h         (compat shim over frame.h)
├── slave1_arm_base/           ← J1-J3 (GPIO step, bit-bang SPI GP2/3/4/5)
│   └── spi_cmd_slave_s1.c
├── slave2_wrist_gripper/      ← J4-J6 + gripper (TMC2209 UART, WS2812)
│   └── spi_cmd_slave_s2.c
├── tools/                     ← build/flash helpers
├── tests/                     ← host-side protocol unit tests (no Pico SDK)
└── docs/                      ← PROGRESS.md, JOURNAL.md, hardening log
```

## Why this layout

1. **`shared/spi_protocol/` is the only place the frame format is defined.**
   The master driver and both slaves `#include` it directly (via CMake
   include paths). If the frame ever changes, it changes once.
2. **`slave_common/`** holds code both slave boards link — the motion
   planner, homing state machine, and config store are shared verbatim.
3. **`master/`, `slave1_arm_base/`, `slave2_wrist_gripper/`** hold only
   what is specific to each board.

## Wire format (current)

9 bytes, CS-per-byte framing at 50kHz, two frames per command (v1 layout,
hardware-verified — unchanged from the working v1 firmware):

```
Command:  [cmd, a1, a2, a3, a4, a5, a6, a7, crc8]
Response: [status, d1, d2, d3, d4, d5, d6, d7, crc8]
```

CRC8 (poly 0x07, init 0xFF) covers bytes [0..7]. A slave never executes a
frame whose CRC fails. **Master and slaves MUST be flashed together.**

The `frame.h` header also defines the **v2 structured format** (sync byte
0xA5 + opcode + int32 payload + seq) as the migration target — see
`shared/spi_protocol/protocol_spec.md`. It is unit-tested and ready, but
not yet on the wire.

## Build

```bash
cmake -B build -GNinja -DPICO_SDK_PATH=C:/pico/pico-sdk -DPICO_NO_PICOTOOL=1
cmake --build build --target spi_cmd_master_eth spi_cmd_slave_s1 spi_cmd_slave_s2
```

This produces `*.elf`/`*.bin`/`*.hex`. Convert to `.uf2` with picotool:

```bash
picotool uf2 convert build/spi_cmd_master_eth.elf build/spi_cmd_master_eth.uf2
picotool uf2 convert build/spi_cmd_slave_s1.elf    build/spi_cmd_slave_s1.uf2
picotool uf2 convert build/spi_cmd_slave_s2.elf    build/spi_cmd_slave_s2.uf2
```

or run `tools/flash_all.py` (builds + converts all three).

## Test (no Pico SDK needed)

The shared protocol is pure C — verify it on any host:

```bash
python tests/test_frame_protocol.py      # Python mirror of frame.c
# or with a host gcc:
gcc -I shared/spi_protocol -o /tmp/frame_test \
    tests/test_frame_protocol.c shared/spi_protocol/frame.c && /tmp/frame_test
```

## Flash order

1. Slave 1 → 2. Slave 2 → 3. Master (then the master finds both slaves).
