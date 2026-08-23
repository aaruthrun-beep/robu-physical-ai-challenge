#ifndef NITE_SPI_FRAME_H
#define NITE_SPI_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "common_types.h"

/**
 * Nite 369 v2 — SPI frame protocol (Master <-> Slave 1 / Slave 2).
 *
 * THIS HEADER IS THE SINGLE SOURCE OF TRUTH for the SPI frame format.
 * The master driver and BOTH slaves include these files directly; never
 * copy the constants or structs into a per-project file.
 *
 * ── Wire format (v1 layout, PROVEN on hardware) ──────────────────────
 *   9 bytes, CS-per-byte at 50kHz, two frames per command:
 *     Frame 1 (command): [cmd, a1, a2, a3, a4, a5, a6, a7, crc8]
 *     Frame 2 (dummy)  : all zeros -> slave transmits its response:
 *     Response         : [status, d1, d2, d3, d4, d5, d6, d7, crc8]
 *
 *   CRC8 (poly 0x07, init 0xFF) covers bytes [0..7]; byte 8 carries the
 *   checksum. A slave NEVER executes a command whose CRC fails — it
 *   answers NITE_RSP_BAD_FRAME and the master retries. Master and slaves
 *   MUST be flashed together.
 *
 * ── v2 structured format (migration target, see protocol_spec.md) ────
 *   The v1 byte-layout above is kept because it is hardware-verified and
 *   carries richer payloads (speed+steps+accel+decel) than the int32-only
 *   structured frame. The v2 struct + opcode enum below are the planned
 *   replacement, unit-tested and ready to flip during a bench session
 *   (master + both slaves reflashed together).
 */

#define FRAME_LEN         9   // total bytes on the wire
#define FRAME_CRC_IDX     8   // index of the CRC8 byte
#define FRAME_ARGS        7   // number of argument bytes [1..7]

/**
 * Response status codes live in common_types.h (included above):
 *   NITE_RSP_OK 0x00 / STALE 0x4F / BAD_FRAME 0xF0 / BUSY 0xFE /
 *   ERR 0xFF / LIMIT 0xFD.
 */

/** CRC8 over the payload bytes of a frame (everything except the CRC byte). */
uint8_t frame_crc8(const uint8_t *data, size_t len);

// ── v1 pack/unpack (current wire format) ─────────────────────────────
// Build a 9-byte command frame from opcode + 7 argument bytes, filling
// the CRC. Parse a received frame back into opcode + args after verifying
// CRC. These are the ONLY functions master/slave use to touch the wire.
void frame_v1_pack(uint8_t out9[FRAME_LEN],
                   uint8_t cmd, const uint8_t args[FRAME_ARGS]);
bool frame_v1_unpack(const uint8_t in9[FRAME_LEN],
                     uint8_t *cmd, uint8_t args[FRAME_ARGS]);

// ── v2 structured format (migration target) ──────────────────────────
#define FRAME_SYNC_BYTE  0xA5   // resync anchor after noise events

// v2 opcodes — replace the v1 command bytes during migration.
typedef enum {
    OP_STEP_DELTA   = 0x01,   // master -> slave: step count + direction
    OP_PING         = 0x02,   // master -> slave: liveness check
    OP_STATUS_REQ   = 0x03,   // master -> slave: request position/state
    OP_ENABLE       = 0x04,   // master -> slave: enable/disable driver
    OP_HOME         = 0x05,   // master -> slave: begin homing sequence
    OP_LIMIT_CFG    = 0x06,   // master -> slave: soft-limit values
    OP_CFG_WRITE    = 0x07,   // master -> slave: write axis config
    OP_CFG_READ     = 0x08,   // master -> slave: read axis config
    OP_MOTION_STATUS= 0x09,   // master -> slave: request motion status
    OP_ENCODER_READ = 0x0A,   // master -> slave: read encoder value
    OP_LIMIT_READ   = 0x0B,   // master -> slave: read limit switch state
    OP_HOMING_CFG   = 0x0C,   // master -> slave: write homing configuration
    OP_HOMING_STATUS= 0x0D,   // master -> slave: read homing status
    OP_TMC_READ     = 0x10,   // master -> slave: read TMC register
    OP_TMC_WRITE    = 0x11,   // master -> slave: write TMC register (seq = register)
    OP_LED          = 0x12,   // master -> slave: set WS2812 LED color
    OP_GRIPPER      = 0x13,   // master -> slave: gripper servo position
    OP_GO           = 0x14,   // master -> slave: start all staged (HOLD) moves
    OP_HALT         = 0x15,   // master -> slave: stop axis (or all with 0xFF)
    OP_CONT_JOG     = 0x16,   // master -> slave: continuous jog, hold-to-run
                              //   (payload = V2_PAYLOAD_MOVE(speed, dir +/-1))
    OP_CFG_SAVE     = 0x17,   // master -> slave: persist config to flash
                              //   (replaces v1 0x47; slave blocks ~100-200ms)
    OP_CFG_RESET    = 0x18,   // master -> slave: reset config to defaults
                              //   (replaces v1 0x49)
    // Slave -> Master responses
    OP_ACK          = 0x81,   // command accepted
    OP_BUSY         = 0x82,   // still executing, do not send next
    OP_STATUS_REPLY = 0x83,   // position, state, fault flags
    OP_FAULT        = 0x84,   // TMC/encoder fault, includes reason code
    OP_CFG_REPLY    = 0x85,   // config read reply
    OP_PONG         = 0x86,   // ping response (alive marker)
    OP_LIMIT_REPLY  = 0x87,   // limit switch state reply
    OP_HOMING_REPLY = 0x88,   // homing status reply
    OP_TMC_REPLY    = 0x89,   // TMC register read reply
    OP_MOTION_REPLY = 0x8A,   // motion status reply (pos, spd, mov)
    OP_ENCODER_REPLY= 0x8B,   // encoder read reply
} frame_opcode_t;

// ── Payload / joint_id packing (per-opcode contract) ────────────────
// These are the ONLY field layouts used on the v2 wire. Kept here so the
// master and both slaves agree without copying.

// OP_STEP_DELTA payload: [31:16] speed (steps/sec, u16), [15:0] steps (int16)
#define V2_PAYLOAD_MOVE(speed, steps) \
    ((int32_t)((((uint32_t)(speed) & 0xFFFFu) << 16) | ((uint32_t)(steps) & 0xFFFFu)))
#define V2_MOVE_SPEED(p)  (uint16_t)(((uint32_t)(p) >> 16) & 0xFFFFu)
#define V2_MOVE_STEPS(p)  (int16_t)((uint16_t)(p) & 0xFFFFu)

// OP_STEP_DELTA joint_id HOLD flag (bit 7): set = stage for OP_GO,
// clear = execute immediately (or queue if the axis is busy).
#define V2_JID_HOLD   0x80u

// OP_CFG_READ / OP_CFG_WRITE joint_id: (field << 3) | axis
// (field = cfg_field_t, axis 0-2). payload = the 32-bit value.
#define V2_JID_CFG(field, axis) (uint8_t)(((uint8_t)(field) << 3) | ((uint8_t)(axis) & 0x07u))
#define V2_CFG_FIELD(jid) ((uint8_t)((jid) >> 3))
#define V2_CFG_AXIS(jid)  ((uint8_t)((jid) & 0x07u))

// OP_MOTION_REPLY payload: [31:16] pos (steps, int16), [15:0] spd (steps/sec, u16);
// the reply seq byte carries the moving flag (0/1).
#define V2_PAYLOAD_STATUS(pos, spd) \
    ((int32_t)((((uint32_t)(pos) & 0xFFFFu) << 16) | ((uint32_t)(spd) & 0xFFFFu)))
#define V2_STATUS_POS(p)  (int16_t)((uint16_t)(((uint32_t)(p) >> 16) & 0xFFFFu))
#define V2_STATUS_SPD(p)  (uint16_t)((uint32_t)(p) & 0xFFFFu)

// OP_LED payload: [31:24] r, [23:16] g, [15:8] b, [7:0] mode
#define V2_PAYLOAD_LED(r, g, b, mode) \
    ((int32_t)((((uint32_t)(r) & 0xFFu) << 24) | (((uint32_t)(g) & 0xFFu) << 16) | \
               (((uint32_t)(b) & 0xFFu) << 8) | ((uint32_t)(mode) & 0xFFu)))

// v2 packed struct view of the 9 bytes:
//   [0] sync, [1] joint_id, [2] opcode, [3-6] payload int32 LE, [7] seq, [8] crc8
typedef struct __attribute__((packed)) {
    uint8_t  sync;        // [0] FRAME_SYNC_BYTE — resync point
    uint8_t  joint_id;    // [1] 0-5 (J1-J6) or 0xFF for broadcast
    uint8_t  opcode;      // [2] frame_opcode_t
    int32_t  payload;     // [3-6] step_delta, position, or config value
    uint8_t  seq;         // [7] sequence number — dedupe
    uint8_t  crc8;        // [8] CRC8 over bytes [0..7]
} spi_frame_t;

// v2 pack/unpack (CURRENT wire format — migrated). frame_v2_pack fills the
// CRC; frame_v2_unpack verifies sync + CRC before returning.
bool frame_v2_pack(spi_frame_t *f, uint8_t out9[FRAME_LEN]);
bool frame_v2_unpack(const uint8_t in9[FRAME_LEN], spi_frame_t *f);

// v2 resync helper: scan a buffer for the next valid frame start.
// Returns the byte offset of the next valid frame, or -1 if none found.
int frame_v2_find_resync(const uint8_t *buf, size_t len);

#endif // NITE_SPI_FRAME_H