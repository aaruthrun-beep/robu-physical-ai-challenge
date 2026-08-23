/**
 * @file nite_config.c
 * @brief Nite 369 — GRBL-style single-file configuration system (impl).
 *
 * One text file (nite.cfg) configures all three Picos:
 *   - Uploaded to the Master over USB serial or UDP (CONFIG mode).
 *   - Master re-broadcasts the full blob to both slaves over SPI.
 *   - Every Pico persists the FULL blob to its own reserved flash sector.
 *
 * Flash safety: RP2040 flash erase/program stalls XIP for BOTH cores, so
 * the write is executed from RAM (__no_inline_not_in_flash_func) while the
 * other core is parked in a RAM spin via multicore_lockout (safe on the
 * single-core Master too).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "nite_config.h"

#ifdef USE_TMC2209_UART
#include "nite_tmc2209.h"
#endif

// ==========================================================================
// Flash layout
// ==========================================================================
#ifndef PICO_FLASH_SIZE_BYTES
// Must match CMake's PICO_FLASH_SIZE_BYTES (2MB minus the config reserve),
// otherwise the static asserts below would fail on a standalone compile.
#define PICO_FLASH_SIZE_BYTES (2u * 1024u * 1024u - NITE_CFG_RESERVED)
#endif

#define NITE_XIP_BASE 0x10000000u
// Config region = last 64KB of the PHYSICAL 2MB flash (0x1F0000).
// PICO_FLASH_SIZE_BYTES is already shrunk to 2MB-64KB by CMake so the app
// never overlaps, but the base must be derived from the physical size, NOT
// from PICO_FLASH_SIZE_BYTES (that would double-subtract and land the config
// 64KB INSIDE the app region). flash_range_erase/program take a 0-based
// offset FROM FLASH START (not XIP address), so we keep both:
//   NITE_CFG_FLASH_BASE      = XIP address for memcpy reads
//   NITE_CFG_FLASH_OFF_BASE  = 0-based offset for flash API writes
#define NITE_CFG_FLASH_OFF_BASE (2u * 1024u * 1024u - NITE_CFG_RESERVED)  // 0x1F0000
#define NITE_CFG_FLASH_BASE (NITE_XIP_BASE + NITE_CFG_FLASH_OFF_BASE)

// Blob sizes (packed, little-endian layout, CRC over payload after crc field)
#define CFG_HDR_SIZE    12u  // magic(4) + version(4) + crc32(4)
#define CFG_MASTER_SIZE 16u  // ip(4) + mac(6) + port(2) + timeout(4)
#define CFG_SLAVE_SIZE  51u  // steps(16) + dir(4) + pol(3) + wc/wo/b0/max/slew/db(24) + cur(2) + ms(1) + chop(1)
#define CFG_BLOB_LEN    (CFG_HDR_SIZE + CFG_MASTER_SIZE + 2u * CFG_SLAVE_SIZE)  // 130

// Harden: the blob layout is hand-rolled byte-by-byte; these asserts catch any
// struct-layout drift (padding would silently corrupt the CRC over the wire).
_Static_assert(sizeof(nite_master_cfg_t) == CFG_MASTER_SIZE, "master cfg size drift");
_Static_assert(sizeof(nite_slave_cfg_t)  == CFG_SLAVE_SIZE,  "slave cfg size drift");
_Static_assert(sizeof(nite_config_t)     == CFG_BLOB_LEN,    "config blob size drift");
// Config must start at/after the app region end and stay within physical flash.
_Static_assert(NITE_CFG_FLASH_OFF_BASE >= PICO_FLASH_SIZE_BYTES,
               "config region overlaps app flash region");
_Static_assert(NITE_CFG_FLASH_OFF_BASE + 3u * 4096u <= 2u * 1024u * 1024u,
               "config sectors overflow physical flash");

// ==========================================================================
// Little-endian helpers
// ==========================================================================
static inline void put_u8(uint8_t **p, uint8_t v)   { *(*p)++ = v; }
static inline void put_u16(uint8_t **p, uint16_t v) { *(*p)++ = v & 0xFF; *(*p)++ = (v >> 8) & 0xFF; }
static inline void put_u32(uint8_t **p, uint32_t v) {
    *(*p)++ = v & 0xFF; *(*p)++ = (v >> 8) & 0xFF; *(*p)++ = (v >> 16) & 0xFF; *(*p)++ = (v >> 24) & 0xFF;
}
static inline uint8_t  get_u8(const uint8_t **p)  { return *(*p)++; }
static inline uint16_t get_u16(const uint8_t **p) { uint16_t v = *(*p)++; v |= (uint16_t)(*(*p)++) << 8; return v; }
static inline uint32_t get_u32(const uint8_t **p) {
    uint32_t v = *(*p)++; v |= (uint32_t)(*(*p)++) << 8; v |= (uint32_t)(*(*p)++) << 16; v |= (uint32_t)(*(*p)++) << 24;
    return v;
}

// ==========================================================================
// Blob serialize / deserialize (explicit layout, no padding surprises)
// ==========================================================================
static size_t cfg_serialize(const nite_config_t *cfg, uint8_t *out, size_t cap) {
    if (cap < CFG_BLOB_LEN) return 0;
    uint8_t *p = out;
    put_u32(&p, NITE_CFG_MAGIC);
    put_u32(&p, NITE_CFG_VERSION);
    put_u32(&p, 0); // crc placeholder
    for (int i = 0; i < 4; i++) put_u8(&p, cfg->master.ip[i]);
    for (int i = 0; i < 6; i++) put_u8(&p, cfg->master.mac[i]);
    put_u16(&p, cfg->master.port);
    put_u32(&p, cfg->master.heartbeat_timeout_us);
    for (int sl = 0; sl < 2; sl++) {
        const nite_slave_cfg_t *s = (sl == 0) ? &cfg->slave1 : &cfg->slave2;
        for (int i = 0; i < 4; i++) { uint32_t u; memcpy(&u, &s->steps_per_unit[i], 4); put_u32(&p, u); }
        for (int i = 0; i < 4; i++) put_u8(&p, s->dir_invert[i]);
        for (int i = 0; i < 3; i++) put_u8(&p, s->limit_polarity[i]);
        uint32_t f[5] = {0};
        memcpy(&f[0], &s->ladrc_wc, 4); memcpy(&f[1], &s->ladrc_wo, 4);
        memcpy(&f[2], &s->ladrc_b0, 4); memcpy(&f[3], &s->max_speed, 4);
        memcpy(&f[4], &s->output_slew, 4);
        for (int i = 0; i < 5; i++) put_u32(&p, f[i]);
        uint32_t db; memcpy(&db, &s->deadband, 4); put_u32(&p, db);
        put_u16(&p, s->tmc_current_ma);
        put_u8(&p, s->tmc_microsteps);
        put_u8(&p, s->tmc_stealthchop);
    }
    // Now compute CRC over the payload (everything after the crc field)
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = CFG_HDR_SIZE; i < CFG_BLOB_LEN; i++) {
        crc ^= out[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    crc = ~crc;
    out[8] = crc & 0xFF; out[9] = (crc >> 8) & 0xFF; out[10] = (crc >> 16) & 0xFF; out[11] = (crc >> 24) & 0xFF;
    return CFG_BLOB_LEN;
}

static bool cfg_deserialize(nite_config_t *cfg, const uint8_t *in, size_t len) {
    if (len < CFG_BLOB_LEN) return false;
    const uint8_t *p = in;
    uint32_t magic = get_u32(&p), version = get_u32(&p), crc_stored = get_u32(&p);
    if (magic != NITE_CFG_MAGIC || version != NITE_CFG_VERSION) return false;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = CFG_HDR_SIZE; i < CFG_BLOB_LEN; i++) {
        crc ^= in[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    if (~crc != crc_stored) return false;
    for (int i = 0; i < 4; i++) cfg->master.ip[i] = get_u8(&p);
    for (int i = 0; i < 6; i++) cfg->master.mac[i] = get_u8(&p);
    cfg->master.port = get_u16(&p);
    cfg->master.heartbeat_timeout_us = get_u32(&p);
    for (int sl = 0; sl < 2; sl++) {
        nite_slave_cfg_t *s = (sl == 0) ? &cfg->slave1 : &cfg->slave2;
        for (int i = 0; i < 4; i++) { uint32_t u = get_u32(&p); memcpy(&s->steps_per_unit[i], &u, 4); }
        for (int i = 0; i < 4; i++) s->dir_invert[i] = get_u8(&p);
        for (int i = 0; i < 3; i++) s->limit_polarity[i] = get_u8(&p);
        uint32_t f[6] = {0};
        for (int i = 0; i < 6; i++) f[i] = get_u32(&p);
        memcpy(&s->ladrc_wc, &f[0], 4); memcpy(&s->ladrc_wo, &f[1], 4);
        memcpy(&s->ladrc_b0, &f[2], 4); memcpy(&s->max_speed, &f[3], 4);
        memcpy(&s->output_slew, &f[4], 4); memcpy(&s->deadband, &f[5], 4);
        s->tmc_current_ma = get_u16(&p);
        s->tmc_microsteps = get_u8(&p);
        s->tmc_stealthchop = get_u8(&p);
    }
    return true;
}

// ==========================================================================
// Factory defaults (match the pre-config firmware behaviour)
// ==========================================================================
void nite_cfg_defaults(nite_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->master.ip[0] = 192; cfg->master.ip[1] = 168; cfg->master.ip[2] = 1; cfg->master.ip[3] = 100;
    cfg->master.mac[0] = 0x00; cfg->master.mac[1] = 0x08; cfg->master.mac[2] = 0xDC;
    cfg->master.mac[3] = 0x11; cfg->master.mac[4] = 0x22; cfg->master.mac[5] = 0x33;
    cfg->master.port = 5000;
    cfg->master.heartbeat_timeout_us = 100000;

    // SLAVE1 (arm base): J0 211.56, J1 400.0, J1-tandem 400.0, J2 266.67
    static const float s1_steps[4] = { 211.56f, 400.0f, 400.0f, 266.67f };
    // SLAVE2 (wrist): J3 142.22, J4 222.22, J5 222.22, gripper 1.0
    static const float s2_steps[4] = { 142.22f, 222.22f, 222.22f, 1.0f };
    nite_slave_cfg_t *slaves[2] = { &cfg->slave1, &cfg->slave2 };
    const float *steps[2] = { s1_steps, s2_steps };
    for (int sl = 0; sl < 2; sl++) {
        nite_slave_cfg_t *s = slaves[sl];
        for (int i = 0; i < 4; i++) s->steps_per_unit[i] = steps[sl][i];
        s->ladrc_wc = 20.0f;
        s->ladrc_wo = 100.0f;
        s->ladrc_b0 = 120.0f;
        s->max_speed = 1500.0f;
        s->output_slew = 10.0f;
        s->deadband = 25.0f;
        s->tmc_current_ma = 1000;
        s->tmc_microsteps = 8;
        s->tmc_stealthchop = 0;
    }
}

bool nite_cfg_valid(const nite_config_t *cfg) {
    uint8_t blob[CFG_BLOB_LEN];
    if (cfg_serialize(cfg, blob, sizeof(blob)) != CFG_BLOB_LEN) return false;
    nite_config_t tmp;
    return cfg_deserialize(&tmp, blob, sizeof(blob));
}

// ==========================================================================
// Flash load / save
// ==========================================================================
//
// FIXED (CRITICAL): the RP2040 flash API takes a 0-based offset from flash
// start, NOT a relative sector index and NOT an XIP address. Passing 0 here
// would erase the boot2 loader + vector table + firmware start! We now add
// NITE_CFG_FLASH_OFF_BASE (0x1F0000 = 2MB - 64KB).
//
// FIXED (CRITICAL): SDK multicore_lockout_start_blocking() spins forever
// unless the OTHER core is already executing the lockout-victim loop. Our
// slaves run the LADRC control loop on Core 1 (no victim) and the Master
// never launches Core 1 — so the SDK call would deadlock on the first save.
// We use an explicit handshake instead:
//   - Core 0 sets cfg_flash_active, waits for Core 1 to reach the RAM spin
//     (cfg_flash_entered), disables IRQs, does erase+program, clears flag.
//   - Core 1 checks cfg_flash_active once per control cycle; if set it jumps
//     into the RAM-resident spin below (which never fetches from flash).
//   - On single-core builds (Master) core1 is never marked running, so the
//     handshake is skipped entirely.

static volatile bool cfg_core1_running = false;
static volatile bool cfg_flash_active = false;
static volatile bool cfg_flash_entered = false;

void nite_cfg_set_core1_running(bool running) {
    cfg_core1_running = running;
}

/** Core 1: RAM-resident spin — safe to run while the other core programs flash. */
void __no_inline_not_in_flash_func(nite_cfg_flash_guard)(void) {
    if (cfg_flash_active) {
        cfg_flash_entered = true;
        while (cfg_flash_active) {
            tight_loop_contents();
        }
        cfg_flash_entered = false;
    }
}

static void __no_inline_not_in_flash_func(cfg_flash_write)(uint32_t flash_off, const uint8_t *blob) {
    // flash_off is the 0-based offset from flash start (already base-adjusted)
    if (cfg_core1_running) {
        // Ask Core 1 to enter the RAM spin, then wait until it confirms
        cfg_flash_active = true;
        while (!cfg_flash_entered) {
            tight_loop_contents();
        }
    }
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(flash_off, FLASH_SECTOR_SIZE);
    flash_range_program(flash_off, blob, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    if (cfg_core1_running) {
        cfg_flash_active = false;          // release Core 1
        while (cfg_flash_entered) {
            tight_loop_contents();         // wait until it leaves the spin
        }
    }
}

bool nite_cfg_load(nite_config_t *cfg, uint32_t flash_offset) {
    uint8_t blob[CFG_BLOB_LEN];
    memcpy(blob, (const void *)(NITE_CFG_FLASH_BASE + flash_offset), CFG_BLOB_LEN);
    return cfg_deserialize(cfg, blob, sizeof(blob));
}

bool nite_cfg_save(const nite_config_t *cfg, uint32_t flash_offset) {
    uint8_t blob[FLASH_PAGE_SIZE];   // 256 bytes, zero-padded
    memset(blob, 0xFF, sizeof(blob));
    size_t len = cfg_serialize(cfg, blob, sizeof(blob));
    if (len != CFG_BLOB_LEN) return false;
    cfg_flash_write(NITE_CFG_FLASH_OFF_BASE + flash_offset, blob);
    nite_config_t verify;
    return nite_cfg_load(&verify, flash_offset);
}

// ==========================================================================
// Text parsing / formatting (the uploadable file / the $$ dump)
// ==========================================================================
static char *trim_str(char *s) {
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) e--;
    *e = '\0';
    return s;
}

static bool parse_u8_list(const char *v, uint8_t *out, int n) {
    int got = 0;
    char buf[64];
    snprintf(buf, sizeof(buf), "%s", v);
    char *tok = strtok(buf, " ,");
    while (tok && got < n) {
        int val = atoi(tok);
        if (val < 0 || val > 255) return false;
        out[got++] = (uint8_t)val;
        tok = strtok(NULL, " ,");
    }
    return got == n;
}

static bool parse_float_list(const char *v, float *out, int n) {
    int got = 0;
    char buf[128];
    snprintf(buf, sizeof(buf), "%s", v);
    char *tok = strtok(buf, " ,");
    while (tok && got < n) {
        out[got++] = strtof(tok, NULL);
        tok = strtok(NULL, " ,");
    }
    return got == n;
}

// Parse one key=value into a config section. section: 1=master, 2=slave1, 3=slave2
static bool cfg_set_field(nite_config_t *cfg, int section, const char *key, const char *value) {
    nite_slave_cfg_t *s = (section == 2) ? &cfg->slave1 : &cfg->slave2;
    float f[4];

    if (section == 1) {
        if (strcmp(key, "ip") == 0) {
            return sscanf(value, "%hhu.%hhu.%hhu.%hhu",
                          &cfg->master.ip[0], &cfg->master.ip[1],
                          &cfg->master.ip[2], &cfg->master.ip[3]) == 4;
        }
        if (strcmp(key, "mac") == 0) {
            unsigned m[6];
            if (sscanf(value, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
                return false;
            for (int i = 0; i < 6; i++) cfg->master.mac[i] = (uint8_t)m[i];
            return true;
        }
        if (strcmp(key, "port") == 0) { int p = atoi(value); if (p < 0 || p > 65535) return false; cfg->master.port = (uint16_t)p; return true; }
        if (strcmp(key, "heartbeat_timeout_us") == 0) { long t = atol(value); if (t < 0) return false; cfg->master.heartbeat_timeout_us = (uint32_t)t; return true; }
        return false;
    }

    if (strcmp(key, "steps_per_unit") == 0) {
        if (!parse_float_list(value, f, 4)) return false;
        for (int i = 0; i < 4; i++) s->steps_per_unit[i] = f[i];
        return true;
    }
    if (strcmp(key, "dir_invert") == 0) return parse_u8_list(value, s->dir_invert, 4);
    if (strcmp(key, "limit_polarity") == 0) return parse_u8_list(value, s->limit_polarity, 3);
    if (strcmp(key, "ladrc_wc") == 0) { s->ladrc_wc = strtof(value, NULL); return true; }
    if (strcmp(key, "ladrc_wo") == 0) { s->ladrc_wo = strtof(value, NULL); return true; }
    if (strcmp(key, "ladrc_b0") == 0) { s->ladrc_b0 = strtof(value, NULL); return true; }
    if (strcmp(key, "max_speed") == 0) { s->max_speed = strtof(value, NULL); return true; }
    if (strcmp(key, "output_slew") == 0) { s->output_slew = strtof(value, NULL); return true; }
    if (strcmp(key, "deadband") == 0) { s->deadband = strtof(value, NULL); return true; }
    if (strcmp(key, "tmc_current_ma") == 0) { int v = atoi(value); if (v < 0 || v > 65535) return false; s->tmc_current_ma = (uint16_t)v; return true; }
    if (strcmp(key, "tmc_microsteps") == 0) { int v = atoi(value); if (v < 1 || v > 256) return false; s->tmc_microsteps = (uint8_t)v; return true; }
    if (strcmp(key, "tmc_stealthchop") == 0) { int v = atoi(value); if (v < 0 || v > 1) return false; s->tmc_stealthchop = (uint8_t)v; return true; }
    return false;
}

bool nite_cfg_set_key(nite_config_t *cfg, const char *key, const char *value) {
    char k[48];
    snprintf(k, sizeof(k), "%s", key);
    char *dot = strchr(k, '.');
    if (!dot) return false;
    *dot = '\0';
    int section = 0;
    if (strcmp(k, "MASTER") == 0) section = 1;
    else if (strcmp(k, "SLAVE1") == 0) section = 2;
    else if (strcmp(k, "SLAVE2") == 0) section = 3;
    if (!section) return false;
    return cfg_set_field(cfg, section, dot + 1, value);
}

bool nite_cfg_parse_text(nite_config_t *cfg, const char *text) {
    nite_config_t tmp;
    nite_cfg_defaults(&tmp);
    int section = 0;
    char line[160];
    const char *p = text;
    while (*p) {
        // extract one line
        size_t i = 0;
        while (*p && *p != '\n' && i < sizeof(line) - 1) line[i++] = *p++;
        line[i] = '\0';
        if (*p == '\n') p++;
        char *l = trim_str(line);
        if (*l == '\0' || *l == '#') continue;
        if (*l == '[') {
            char *cl = strchr(l, ']');
            if (!cl) return false;
            *cl = '\0';
            if (strcmp(l + 1, "MASTER") == 0) section = 1;
            else if (strcmp(l + 1, "SLAVE1") == 0) section = 2;
            else if (strcmp(l + 1, "SLAVE2") == 0) section = 3;
            else return false;
            continue;
        }
        char *eq = strchr(l, '=');
        if (!eq || section == 0) return false;
        *eq = '\0';
        char *key = trim_str(l);
        char *val = trim_str(eq + 1);
        if (!cfg_set_field(&tmp, section, key, val)) return false;
    }
    *cfg = tmp;
    return true;
}

size_t nite_cfg_format_text(const nite_config_t *cfg, char *buf, size_t cap) {
    size_t n = 0;
    n += snprintf(buf + n, cap - n, "# Nite 369 configuration v%lu (GRBL-style single file)\n", (unsigned long)NITE_CFG_VERSION);
    n += snprintf(buf + n, cap - n, "\n[MASTER]\n");
    n += snprintf(buf + n, cap - n, "ip = %u.%u.%u.%u\n", cfg->master.ip[0], cfg->master.ip[1], cfg->master.ip[2], cfg->master.ip[3]);
    n += snprintf(buf + n, cap - n, "mac = %02X:%02X:%02X:%02X:%02X:%02X\n",
                  cfg->master.mac[0], cfg->master.mac[1], cfg->master.mac[2],
                  cfg->master.mac[3], cfg->master.mac[4], cfg->master.mac[5]);
    n += snprintf(buf + n, cap - n, "port = %u\n", cfg->master.port);
    n += snprintf(buf + n, cap - n, "heartbeat_timeout_us = %lu\n", (unsigned long)cfg->master.heartbeat_timeout_us);

    const nite_slave_cfg_t *slaves[2] = { &cfg->slave1, &cfg->slave2 };
    const char *names[2] = { "SLAVE1", "SLAVE2" };
    for (int sl = 0; sl < 2; sl++) {
        const nite_slave_cfg_t *s = slaves[sl];
        n += snprintf(buf + n, cap - n, "\n[%s]\n", names[sl]);
        n += snprintf(buf + n, cap - n, "steps_per_unit = %.2f %.2f %.2f %.2f\n",
                      s->steps_per_unit[0], s->steps_per_unit[1], s->steps_per_unit[2], s->steps_per_unit[3]);
        n += snprintf(buf + n, cap - n, "dir_invert = %u %u %u %u\n",
                      s->dir_invert[0], s->dir_invert[1], s->dir_invert[2], s->dir_invert[3]);
        n += snprintf(buf + n, cap - n, "limit_polarity = %u %u %u\n",
                      s->limit_polarity[0], s->limit_polarity[1], s->limit_polarity[2]);
        n += snprintf(buf + n, cap - n, "ladrc_wc = %.2f\n", s->ladrc_wc);
        n += snprintf(buf + n, cap - n, "ladrc_wo = %.2f\n", s->ladrc_wo);
        n += snprintf(buf + n, cap - n, "ladrc_b0 = %.2f\n", s->ladrc_b0);
        n += snprintf(buf + n, cap - n, "max_speed = %.2f\n", s->max_speed);
        n += snprintf(buf + n, cap - n, "output_slew = %.2f\n", s->output_slew);
        n += snprintf(buf + n, cap - n, "deadband = %.2f\n", s->deadband);
        n += snprintf(buf + n, cap - n, "tmc_current_ma = %u\n", s->tmc_current_ma);
        n += snprintf(buf + n, cap - n, "tmc_microsteps = %u\n", s->tmc_microsteps);
        n += snprintf(buf + n, cap - n, "tmc_stealthchop = %u\n", s->tmc_stealthchop);
    }
    return n;
}

// ==========================================================================
// SPI config-frame protocol (Master -> both slaves)
// ==========================================================================
uint16_t nite_cfg_frame_count(void) {
    return (uint16_t)((CFG_BLOB_LEN + ASTRA_CFG_FRAME_PAYLOAD - 1) / ASTRA_CFG_FRAME_PAYLOAD);
}

void nite_cfg_make_frame(uint16_t idx, const nite_config_t *cfg, astra_spi_cmd_t *out) {
    uint8_t blob[CFG_BLOB_LEN];
    cfg_serialize(cfg, blob, sizeof(blob));
    memset(out, 0, sizeof(*out));
    out->control_word = ASTRA_CFG_TAG;
    out->reserved[0] = (uint8_t)idx;
    out->reserved[1] = (uint8_t)nite_cfg_frame_count();
    for (int i = 0; i < (int)ASTRA_CFG_FRAME_PAYLOAD; i++) {
        size_t pos = (size_t)idx * ASTRA_CFG_FRAME_PAYLOAD + i;
        out->reserved[2 + i] = (pos < CFG_BLOB_LEN) ? blob[pos] : 0xFF;
    }
    out->crc = astra_crc8((uint8_t *)out, sizeof(*out) - 1);
}

// Slave-side incremental reassembly state
static uint8_t  cfg_rx_buf[CFG_BLOB_LEN];
static uint16_t cfg_rx_total = 0;
static uint16_t cfg_rx_next = 0;

void nite_cfg_rx_reset(void) {
    cfg_rx_total = 0;
    cfg_rx_next = 0;
}

bool nite_cfg_rx_frame(const astra_spi_cmd_t *cmd) {
    if (cmd->control_word != ASTRA_CFG_TAG) return false;
    uint16_t idx = cmd->reserved[0];
    uint16_t total = cmd->reserved[1];
    uint16_t max_frames = (uint16_t)((CFG_BLOB_LEN + ASTRA_CFG_FRAME_PAYLOAD - 1) / ASTRA_CFG_FRAME_PAYLOAD);
    if (total == 0 || total > max_frames) return false;
    if (idx == 0) { cfg_rx_total = total; cfg_rx_next = 0; }
    if (cfg_rx_total != total || idx != cfg_rx_next) return false; // require in-order
    for (int i = 0; i < (int)ASTRA_CFG_FRAME_PAYLOAD; i++) {
        size_t pos = (size_t)idx * ASTRA_CFG_FRAME_PAYLOAD + i;
        if (pos < CFG_BLOB_LEN) cfg_rx_buf[pos] = cmd->reserved[2 + i];
    }
    cfg_rx_next++;
    return cfg_rx_next >= cfg_rx_total;
}

bool nite_cfg_rx_result(nite_config_t *cfg_out) {
    if (cfg_rx_next < cfg_rx_total) return false;
    return cfg_deserialize(cfg_out, cfg_rx_buf, sizeof(cfg_rx_buf));
}

// ==========================================================================
// TMC2209 application (Slave 2 wrist drivers)
// ==========================================================================
void nite_cfg_apply_tmc(const nite_slave_cfg_t *s) {
#ifdef USE_TMC2209_UART
    // Map config -> TMC register values
    uint8_t mres = 0;
    uint32_t ms = s->tmc_microsteps;
    while (ms > 1 && mres < 8) { ms >>= 1; mres++; }

    // IRUN: ~16/32 of full scale at 1000mA with 0.11R sense (rough linear map)
    int irun = (int)((uint32_t)s->tmc_current_ma * 16u / 1000u);
    if (irun < 1) irun = 1;
    if (irun > 31) irun = 31;
    int ihold = irun / 2;
    if (ihold < 1) ihold = 1;

    uint32_t gconf = TMC2209_GCONF_MSTEP_REG_SELECT;
    if (s->tmc_stealthchop) {
        gconf &= ~TMC2209_GCONF_EN_SPREADCYCLE;   // StealthChop
    } else {
        gconf |= TMC2209_GCONF_EN_SPREADCYCLE;    // SpreadCycle
    }
    uint32_t chopconf = TMC2209_TOFF(3) | TMC2209_HSTRT(5) | TMC2209_HEND(2) |
                        TMC2209_TBL(1) | TMC2209_MRES(mres) | TMC2209_INTPOL;
    uint32_t ihold_irun = TMC2209_IRUN(irun) | TMC2209_IHOLD(ihold) | TMC2209_IHOLDDELAY(4);

    printf("  TMC config: %umA, 1/%u, %s\n",
           s->tmc_current_ma, (unsigned)s->tmc_microsteps,
           s->tmc_stealthchop ? "StealthChop" : "SpreadCycle");
    tmc2209_configure_all(gconf, chopconf, ihold_irun);
#else
    (void)s;
#endif
}
