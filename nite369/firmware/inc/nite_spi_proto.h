#ifndef NITE_SPI_PROTO_H
#define NITE_SPI_PROTO_H

#include "nite_transmission.h"

/**
 * Nite 369 — SPI command protocol (Master <-> Slave 1 / Slave 2).
 *
 * CS-per-byte at 100kHz, two frames per command:
 *   Frame 1 (command): [cmd, a1, a2, a3, a4, a5, a6, a7, crc8]
 *   Frame 2 (dummy)  : all zeros -> slave transmits its response:
 *   Response         : [status, d1, d2, d3, d4, d5, d6, d7, crc8]
 *
 * CRC8 (astra_crc8: poly 0x07, init 0xFF — same as the LinuxCNC bridge and
 * config upload tools) covers bytes [0..7]; byte 8 carries the checksum.
 *
 * Integrity guarantee: a slave NEVER executes a command whose CRC fails.
 * It answers NITE_RSP_BAD_FRAME instead and the master retries. This makes
 * the link immune to the wire/EMI corruption observed on #MV frames
 * (see PROGRESS.md — SPI data corruption on motion commands), where a
 * corrupted 0x4D frame previously made the arm move the wrong distance.
 *
 * NOTE: master and slaves MUST be flashed together — frame length changed
 * from 8 to 9 bytes when CRC protection was added.
 */

#define NITE_SPI_BUF_LEN   9
#define NITE_SPI_CRC_IDX   8

// Response status codes (byte 0 of the response frame)
#define NITE_RSP_OK        0x00  // command executed OK
#define NITE_RSP_STALE     0x4F  // idle/PONG — slave wasn't processing (retry)
#define NITE_RSP_BAD_FRAME 0xF0  // CRC mismatch — command NOT executed (retry)
#define NITE_RSP_BUSY      0xFE  // axis busy (retry)
#define NITE_RSP_ERR       0xFF  // hard error (do not retry)

/** CRC8 over the payload bytes of a frame (everything except the CRC byte). */
static inline uint8_t nite_spi_crc(const uint8_t *frame) {
    return astra_crc8(frame, NITE_SPI_CRC_IDX);
}

#endif // NITE_SPI_PROTO_H
