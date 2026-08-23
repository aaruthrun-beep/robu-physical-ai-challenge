#ifndef NITE_TRANSMISSION_H
#define NITE_TRANSMISSION_H

#include <stdint.h>

/**
 * @brief High-Performance Deterministic Protocol for the Nite 369 System.
 * 
 * Optimized for minimal latency and high integrity across long wire runs.
 */

#define ASTRA_MAGIC 0xAC
#define MAX_AXES 8
#define AXES_PER_SLAVE 4
#define SPI_XFER_SIZE 26  // Fixed full-duplex transfer size (must match on Master and Slave)

static inline uint8_t astra_crc8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // ATM polynomial x^8 + x^2 + x + 1 (0x07)
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// --- 1. LinuxCNC <-> Astra-Master (UDP) ---

typedef struct __attribute__((packed)) {
    uint8_t magic;           // 0xAC
    uint32_t sequence;       // Frame counter
    int32_t target[MAX_AXES]; // Joint targets in raw counts
    uint8_t enable_bits;     // Global/Local enables
    uint8_t reserve[2];
    uint8_t crc;
} astra_udp_cmd_t;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint32_t sequence_ack;
    int32_t actual[MAX_AXES]; // Feedback from encoders
    uint16_t current_ma[MAX_AXES]; // Estimated motor current
    uint8_t status_flags;     // Error/Limit/Limit
    uint8_t crc;
} astra_udp_telemetry_t;

// --- 2. Astra-Master <-> Astra-Slave (SPI) ---

typedef struct __attribute__((packed)) {
    int32_t target[AXES_PER_SLAVE];   // 16 bytes
    uint8_t control_word;              // 1 byte
    uint8_t reserved[8];               // Pad to match feedback size for full-duplex SPI
    uint8_t crc;                       // 1 byte  -> Total: 26 bytes
} astra_spi_cmd_t;

typedef struct __attribute__((packed)) {
    int32_t actual[AXES_PER_SLAVE];   // 16 bytes
    int16_t error[AXES_PER_SLAVE];    // 8 bytes - Following error for real-time graphs
    uint8_t health_bits;              // 1 byte - Thermal/Stall warnings
    uint8_t crc;                      // 1 byte  -> Total: 26 bytes
} astra_spi_feedback_t;

#endif // NITE_TRANSMISSION_H
