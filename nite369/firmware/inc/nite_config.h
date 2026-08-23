#ifndef NITE_CONFIG_H
#define NITE_CONFIG_H

/**
 * @file nite_config.h
 * @brief Nite 369 — GRBL-style single-file configuration system.
 *
 * One text file (nite.cfg) configures ALL three Picos:
 *
 *   [MASTER]  ip, mac, port, heartbeat timeout      -> Master Pico
 *   [SLAVE1]  steps/unit, dir, limits, LADRC, TMC    -> Arm Base Pico
 *   [SLAVE2]  steps/unit, dir, limits, LADRC, TMC    -> Wrist Pico
 *
 * Transport:
 *   - Uploaded to the Master over USB serial or UDP (CONFIG mode).
 *   - Master re-broadcasts the blob to both slaves over the existing
 *     26-byte SPI full-duplex frames (config frames).
 *   - Every Pico persists the FULL blob to its own reserved flash sector,
 *     so a single Pico can be re-flashed and still self-configure.
 *
 * Flash layout (last 64KB of the 2MB flash, app linker script is shrunk):
 *   NITE_CFG_FLASH_BASE + 0x0000  -> Master's copy
 *   NITE_CFG_FLASH_BASE + 0x1000  -> Slave 1's copy
 *   NITE_CFG_FLASH_BASE + 0x2000  -> Slave 2's copy
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nite_transmission.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================================
// Constants
// ==========================================================================

#define NITE_CFG_MAGIC          0x4E495445u   // "NITE"
#define NITE_CFG_VERSION        1u

#define NITE_CFG_RESERVED       (64u * 1024u) // 64KB flash reserved at end
#define NITE_CFG_BLOB_SIZE      256u          // padded blob (one flash page)
#define NITE_CFG_TEXT_MAX       2048u         // text config buffer size

// Flash offsets (relative to NITE_CFG_FLASH_BASE), one 4KB sector each
#define NITE_CFG_OFFSET_MASTER  0u
#define NITE_CFG_OFFSET_SLAVE1  4096u
#define NITE_CFG_OFFSET_SLAVE2  8192u

// SPI config-frame markers (carried inside astra_spi_cmd_t)
#define ASTRA_CFG_TAG           0xCFu   // control_word == TAG  -> config frame
#define ASTRA_CFG_ACK_BIT       0x80u   // feedback.health_bits bit 7 = applied
#define ASTRA_CFG_FRAME_PAYLOAD 6u      // payload bytes per frame (reserved[2..7])

// ==========================================================================
// Config structures (packed for deterministic CRC + serialization)
// ==========================================================================

typedef struct __attribute__((packed)) {
    uint8_t  ip[4];                // e.g. 192.168.1.100
    uint8_t  mac[6];               // e.g. 00:08:DC:11:22:33
    uint16_t port;                 // UDP port (default 5000)
    uint32_t heartbeat_timeout_us; // link-loss e-stop timeout
} nite_master_cfg_t;

typedef struct __attribute__((packed)) {
    float    steps_per_unit[4];    // counts per degree (LinuxCNC SCALE)
    uint8_t  dir_invert[4];        // 1 = reverse motor direction
    uint8_t  limit_polarity[3];    // 0 = active-low (NC->GND), 1 = active-high
    float    ladrc_wc;             // LADRC controller bandwidth
    float    ladrc_wo;             // LADRC observer bandwidth
    float    ladrc_b0;             // LADRC plant gain estimate
    float    max_speed;            // max speed (counts/s)
    float    output_slew;          // speed slew limit per cycle
    float    deadband;             // position deadband (counts)
    uint16_t tmc_current_ma;       // TMC2209 run current (mA)
    uint8_t  tmc_microsteps;       // 8 / 16 / 32 ...
    uint8_t  tmc_stealthchop;      // 0 = SpreadCycle, 1 = StealthChop
} nite_slave_cfg_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;                // NITE_CFG_MAGIC
    uint32_t version;              // NITE_CFG_VERSION
    uint32_t crc32;                // CRC32 over everything after this field
    nite_master_cfg_t master;
    nite_slave_cfg_t  slave1;
    nite_slave_cfg_t  slave2;
} nite_config_t;

// ==========================================================================
// API
// ==========================================================================

/** Fill cfg with the built-in factory defaults (matches current firmware). */
void nite_cfg_defaults(nite_config_t *cfg);

/** True if magic + version + crc32 all check out. */
bool nite_cfg_valid(const nite_config_t *cfg);

/** Parse an INI-style text config (ignores # comments, blank lines). */
bool nite_cfg_parse_text(nite_config_t *cfg, const char *text);

/** Format cfg as INI text (the $$ dump / the uploadable file). Returns bytes written. */
size_t nite_cfg_format_text(const nite_config_t *cfg, char *buf, size_t cap);

/** Set a single "SECTION.key" to a value (for $ commands). */
bool nite_cfg_set_key(nite_config_t *cfg, const char *key, const char *value);

/** Load the full blob from flash at the given offset (0/SLAVE1/SLAVE2). */
bool nite_cfg_load(nite_config_t *cfg, uint32_t flash_offset);

/** Save the full blob to flash at the given offset (0/SLAVE1/SLAVE2). */
bool nite_cfg_save(const nite_config_t *cfg, uint32_t flash_offset);

/** Number of SPI config frames needed to transmit the full blob. */
uint16_t nite_cfg_frame_count(void);

/**
 * Build SPI config frame #idx for both slaves (same payload).
 * @param idx     frame index (0..count-1)
 * @param cfg     blob to transmit
 * @param out     astra_spi_cmd_t filled in config mode (control_word=TAG)
 */
void nite_cfg_make_frame(uint16_t idx, const nite_config_t *cfg, astra_spi_cmd_t *out);

/** Mark whether core1 is running (slaves only; Master leaves false). */
void nite_cfg_set_core1_running(bool running);

/** Core1: call every control cycle to participate in the flash-write handshake. */
void nite_cfg_flash_guard(void);

/** Reset the slave-side incremental blob reassembly state. */
void nite_cfg_rx_reset(void);

/**
 * Feed one received config frame into the reassembly buffer (slave side).
 * Clears the "applied" flag on frame 0. Returns true when the full blob
 * has been received (call nite_cfg_rx_result() afterwards).
 */
bool nite_cfg_rx_frame(const astra_spi_cmd_t *cmd);

/** Get the reassembled, validated blob (only valid if rx_frame returned true). */
bool nite_cfg_rx_result(nite_config_t *cfg_out);

/** Apply TMC2209 settings from a slave config (no-op unless USE_TMC2209_UART). */
void nite_cfg_apply_tmc(const nite_slave_cfg_t *s);

#ifdef __cplusplus
}
#endif

#endif // NITE_CONFIG_H
