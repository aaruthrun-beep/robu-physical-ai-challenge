#ifndef NITE_SPI_PROTO_H
#define NITE_SPI_PROTO_H

/**
 * Compatibility shim — v2 restructure.
 *
 * The single source of truth for the SPI frame format now lives in
 * shared/spi_protocol/frame.h (+ common_types.h). This header maps the
 * legacy v1 names onto the shared definitions so the proven master/slave
 * command processors compile unchanged against the shared protocol.
 *
 * New code should include "frame.h" directly and use frame_crc8(),
 * FRAME_LEN, FRAME_CRC_IDX, NITE_RSP_* etc.
 */

#include "frame.h"

// Legacy aliases -> shared protocol names
#define NITE_SPI_BUF_LEN   FRAME_LEN
#define NITE_SPI_CRC_IDX   FRAME_CRC_IDX

/** CRC8 over the payload bytes of a frame (everything except the CRC byte). */
static inline uint8_t nite_spi_crc(const uint8_t *frame) {
    return frame_crc8(frame, FRAME_CRC_IDX);
}

#endif // NITE_SPI_PROTO_H