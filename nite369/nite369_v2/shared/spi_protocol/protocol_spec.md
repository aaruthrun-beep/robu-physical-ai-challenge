# Nite369 v2 — SPI Frame Protocol Spec

Version: 2.1 (structured frames with sync + CRC8 — LIVE on all three Picos)

> Migration complete 2026-08-17: the v1 byte-layout is retired. Master and
> both slaves exclusively use the structured frame below. The legacy
> `frame_v1_pack/unpack` remain in frame.c for host-side decoding of old
> captures only.

## Wire format

Every frame is exactly 9 bytes, sent with CS-per-byte framing at 50kHz
(unchanged from v1 — the RP2040 SPI slave reloads its TX shift register
only when CS re-asserts, so CS must toggle HIGH between each byte).

```
 byte:   0       1         2        3     4     5     6     7       8
        sync   joint_id  opcode   payload (int32, little-endian)   seq   crc8
        0xA5    0-5/0xFF   enum    3  4  5  6                     7     over [0..7]
```

- `sync` (0xA5) is a fixed resync anchor. On a CRC failure the receiver
  scans forward for 0xA5 + valid CRC instead of the ad-hoc v1 resync logic.
- `seq` dedupes retried commands: a slave that already executed `seq` for a
  given opcode/joint answers OK without re-executing.
- `crc8` is CRC8-ATM (poly 0x07, init 0xFF) — identical to v1's
  `astra_crc8`, so host tools that verify frames keep working.

## Two-frame transaction (unchanged from v1)

1. Master sends frame 1 (command). Slave validates sync+CRC; on failure
   replies `NITE_RSP_BAD_FRAME` and NEVER executes.
2. Master sends frame 2 (dummy zeros) to clock out the slave's response.

## Opcodes

See `frame.h` — the enum is the single source of truth. Key categories:

| Direction | Opcodes |
|-----------|---------|
| Master → slave | OP_STEP_DELTA, OP_PING, OP_ENABLE, OP_HOME, OP_CFG_WRITE, OP_CFG_READ, OP_MOTION_STATUS, OP_ENCODER_READ, OP_LIMIT_READ, OP_TMC_READ, OP_TMC_WRITE, OP_LED, OP_GRIPPER, OP_GO, OP_HALT, OP_CONT_JOG, OP_CFG_SAVE, OP_CFG_RESET |
| Slave → master | OP_ACK, OP_BUSY, OP_FAULT, OP_CFG_REPLY, OP_PONG, OP_LIMIT_REPLY, OP_HOMING_REPLY, OP_TMC_REPLY, OP_MOTION_REPLY, OP_ENCODER_REPLY |

## Rules

- `shared/spi_protocol/` is the ONLY place the frame format is defined.
  Master driver and both slaves include these files directly — never copy
  the struct into a per-project file.
- A frame with a bad sync byte or bad CRC is dropped, never executed.
- Joint IDs 0-5 map J1-J6; axis on slave = joint_id % 3; slave =
  (joint_id < 3) ? 1 : 2 — identical to v1's mapping.
