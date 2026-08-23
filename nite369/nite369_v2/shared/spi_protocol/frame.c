#include "frame.h"
#include <string.h>

// CRC8-ATM: polynomial 0x07, init 0xFF — identical to the legacy astra_crc8
// used across the Nite369 stack (LinuxCNC bridge, config upload tools,
// old firmware). Keeping the same CRC means tools that already verify
// frames keep working during the transition.
uint8_t frame_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

// ── v1 wire format (current, proven) ─────────────────────────────────
void frame_v1_pack(uint8_t out9[FRAME_LEN],
                   uint8_t cmd, const uint8_t args[FRAME_ARGS]) {
    out9[0] = cmd;
    for (int i = 0; i < FRAME_ARGS; i++) {
        out9[1 + i] = args[i];
    }
    out9[FRAME_CRC_IDX] = frame_crc8(out9, FRAME_CRC_IDX);
}

bool frame_v1_unpack(const uint8_t in9[FRAME_LEN],
                     uint8_t *cmd, uint8_t args[FRAME_ARGS]) {
    if (!in9 || !cmd || !args) return false;
    if (frame_crc8(in9, FRAME_CRC_IDX) != in9[FRAME_CRC_IDX]) return false;
    *cmd = in9[0];
    for (int i = 0; i < FRAME_ARGS; i++) {
        args[i] = in9[1 + i];
    }
    return true;
}

// ── v2 structured format (migration target) ──────────────────────────
bool frame_v2_pack(spi_frame_t *f, uint8_t out9[FRAME_LEN]) {
    if (!f || !out9) return false;
    f->sync = FRAME_SYNC_BYTE;
    out9[0] = f->sync;
    out9[1] = f->joint_id;
    out9[2] = f->opcode;
    out9[3] = (uint8_t)(f->payload & 0xFF);
    out9[4] = (uint8_t)((f->payload >> 8) & 0xFF);
    out9[5] = (uint8_t)((f->payload >> 16) & 0xFF);
    out9[6] = (uint8_t)((f->payload >> 24) & 0xFF);
    out9[7] = f->seq;
    out9[8] = frame_crc8(out9, 8);
    return true;
}

bool frame_v2_unpack(const uint8_t in9[FRAME_LEN], spi_frame_t *f) {
    if (!in9 || !f) return false;
    if (in9[0] != FRAME_SYNC_BYTE) return false;
    if (frame_crc8(in9, 8) != in9[8]) return false;
    f->sync     = in9[0];
    f->joint_id = in9[1];
    f->opcode   = in9[2];
    f->payload  = (int32_t)((uint32_t)in9[3] |
                            ((uint32_t)in9[4] << 8) |
                            ((uint32_t)in9[5] << 16) |
                            ((uint32_t)in9[6] << 24));
    f->seq      = in9[7];
    f->crc8     = in9[8];
    return true;
}

int frame_v2_find_resync(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i + FRAME_LEN <= len; i++) {
        if (buf[i] == FRAME_SYNC_BYTE) {
            if (frame_crc8(buf + i, 8) == buf[i + 8]) {
                return (int)i;
            }
        }
    }
    return -1;
}
