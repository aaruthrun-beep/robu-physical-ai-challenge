#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"

// WIZnet ioLibrary includes
#include "w5500.h"
#include "wizchip_conf.h"
#include "socket.h"

#include "../inc/nite_transmission.h"
#include "../inc/nite_config.h"

// ==========================================================================
// HW Pins
// ==========================================================================

// --- W5500 Ethernet (SPI0) ---
#define W5500_SPI      spi0
#define PIN_RST        15
#define PIN_MISO       16
#define PIN_CS         17
#define PIN_SCK        18
#define PIN_MOSI       19

// --- Servo Slaves (SPI1) ---
#define SLAVE_SPI      spi1
#define SLAVE_SCK      10
#define SLAVE_MOSI     11
#define SLAVE_MISO     12
#define SLAVE_CS0      9
#define SLAVE_CS1      13
#define SYNC_PULSE_PIN 14

#define SOCKET_UDP 0
// UDP port is now config-driven (g_cfg.master.port, default 5000)

// --- MCP23017 I2C GPIO Expander (I2C0, GP0=SDA, GP1=SCL) ---
#define MCP_I2C_ADDR    0x20
#define MCP_LED_POWER   0
#define MCP_LED_MODE    1
#define MCP_LED_LINK    2
#define MCP_LED_FAULT   3
#define MCP_DIP_MODE    4

#define MCP_REG_IODIRA  0x00
#define MCP_REG_IODIRB  0x01
#define MCP_REG_GPPUA   0x0C
#define MCP_REG_GPPUB   0x0D
#define MCP_REG_GPIOA   0x12
#define MCP_REG_GPIOB   0x13

#define MCP_I2C_TIMEOUT_US 10000

// ==========================================================================
// MCP23017 Driver (fixed: uses i2c_write_timeout_us / i2c_read_timeout_us)
// ==========================================================================

static uint8_t mcp_led_shadow = 0;
static bool mcp_is_initialized = false;

static int mcp_read_reg(uint8_t reg) {
    uint8_t data;
    int ret = i2c_write_timeout_us(i2c0, MCP_I2C_ADDR, &reg, 1, true,
                                   MCP_I2C_TIMEOUT_US);
    if (ret < 0) return ret;
    ret = i2c_read_timeout_us(i2c0, MCP_I2C_ADDR, &data, 1, false,
                              MCP_I2C_TIMEOUT_US);
    if (ret < 0) return ret;
    return data;
}

static bool mcp_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_timeout_us(i2c0, MCP_I2C_ADDR, buf, 2, false,
                                MCP_I2C_TIMEOUT_US) == 2;
}

static bool mcp23017_init(void) {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
    sleep_ms(10);

    // Verify MCP presence: write IODIRA=0x00, read back
    if (!mcp_write_reg(MCP_REG_IODIRA, 0x00)) {
        printf("MCP23017 WRITE FAILED at 0x%02X\n", MCP_I2C_ADDR);
        mcp_is_initialized = false;
        return false;
    }
    int check = mcp_read_reg(MCP_REG_IODIRA);
    if (check < 0 || check != 0x00) {
        printf("MCP23017 NOT DETECTED at 0x%02X (read=%d)\n", MCP_I2C_ADDR, check);
        mcp_is_initialized = false;
        return false;
    }

    // Configure: Port A = all inputs, Port B = upper nibble inputs, lower nibble outputs
    mcp_write_reg(MCP_REG_IODIRA, 0xFF);
    mcp_write_reg(MCP_REG_GPPUA,  0xFF);
    mcp_write_reg(MCP_REG_IODIRB, 0xF0);
    mcp_write_reg(MCP_REG_GPPUB,  0x10);
    mcp_led_shadow = 0x00;
    mcp_write_reg(MCP_REG_GPIOB, mcp_led_shadow);
    mcp_is_initialized = true;
    return true;
}

static inline bool mcp_read_dip_mode(void) {
    if (!mcp_is_initialized) return false;
    int gpio_b = mcp_read_reg(MCP_REG_GPIOB);
    if (gpio_b < 0) return false;
    return (gpio_b & (1 << MCP_DIP_MODE)) == 0;
}

static inline void mcp_set_led(uint8_t led_pin, bool on) {
    if (!mcp_is_initialized) return;
    if (on) {
        mcp_led_shadow |= (1 << led_pin);
    } else {
        mcp_led_shadow &= ~(1 << led_pin);
    }
    mcp_write_reg(MCP_REG_GPIOB, mcp_led_shadow);
}

static inline void mcp_toggle_led(uint8_t led_pin) {
    if (!mcp_is_initialized) return;
    mcp_led_shadow ^= (1 << led_pin);
    mcp_write_reg(MCP_REG_GPIOB, mcp_led_shadow);
}

// ==========================================================================
// Global State
// ==========================================================================
volatile astra_udp_cmd_t current_targets;
volatile astra_udp_telemetry_t telemetry_data;
static volatile uint64_t last_heartbeat_us = 0;
static volatile bool link_active = false;
static bool usb_mode = false;
static uint32_t udp_pkt_count = 0;
// FIXED: printf() from an ISR can deadlock/corrupt USB stdio. The ISR only
// sets this flag; the message is printed from the main loop instead.
static volatile bool watchdog_tripped = false;

// ==========================================================================
// Config (GRBL-style single-file configuration)
// ==========================================================================
static nite_config_t g_cfg;
static bool cfg_valid = false;
static uint16_t cfg_upload_port = 5000;  // last-used UDP port (for reopen detect)

// Text config upload state (shared by USB serial and UDP text packets)
static char cfg_upload_buf[NITE_CFG_TEXT_MAX];
static uint32_t cfg_upload_len = 0;
static bool cfg_upload_active = false;

// SPI config distribution state machine
static bool cfg_send_active = false;
static uint16_t cfg_frame_idx = 0;
static uint16_t cfg_frame_total = 0;
static bool cfg_await_ack = false;
static bool cfg_s1_acked = false;
static bool cfg_s2_acked = false;
static uint32_t cfg_ack_deadline_us = 0;

// ==========================================================================
// W5500 SPI Callbacks
// ==========================================================================
static void spi_cs_select(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}
static void spi_cs_deselect(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}
static uint8_t spi_read_byte(void) {
    uint8_t rx = 0;
    spi_read_blocking(W5500_SPI, 0xFF, &rx, 1);
    return rx;
}
static void spi_write_byte(uint8_t wb) {
    spi_write_blocking(W5500_SPI, &wb, 1);
}

// ==========================================================================
// Network Configuration
// ==========================================================================
wiz_NetInfo gWIZNETINFO = {
    .mac = {0x00, 0x08, 0xDC, 0x11, 0x22, 0x33},
    .ip = {192, 168, 1, 100},
    .sn = {255, 255, 255, 0},
    .gw = {192, 168, 1, 1},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

static bool w5500_init(void) {
    // Hardware Reset
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 0);
    sleep_ms(10);
    gpio_put(PIN_RST, 1);
    sleep_ms(50);

    spi_init(W5500_SPI, 20 * 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    reg_wizchip_cs_cbfunc(spi_cs_select, spi_cs_deselect);
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);

    uint8_t memsize[2][8] = { {2,2,2,2,2,2,2,2}, {2,2,2,2,2,2,2,2} };
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1) {
        printf("ERROR: WIZCHIP Initialization failed.\n");
        return false;
    }

    wizchip_setnetinfo(&gWIZNETINFO);
    return true;
}

// ==========================================================================
// Watchdog Timer (runs every 500us from interrupt context)
// FIXED: Uses volatile reads and atomic-style writes for shared state
// ==========================================================================
static bool watchdog_timer_cb(struct repeating_timer *t) {
    (void)t;
    uint64_t now = time_us_64();
    if (link_active && (now - last_heartbeat_us > (uint64_t)g_cfg.master.heartbeat_timeout_us)) {
        // Trip! Zero all targets from interrupt context
        link_active = false;
        watchdog_tripped = true;   // printed from main loop (never from ISR)
        for (int i = 0; i < MAX_AXES; i++) {
            current_targets.target[i] = 0;
        }
        telemetry_data.status_flags |= 0x01;
    }
    return true;
}

// ==========================================================================
// SPI Sync Dispatcher
// ==========================================================================

/**
 * Full-duplex exchange with one slave, CS-PER-BYTE framing.
 *
 * CRITICAL (docs/spi_debug_issues.md Issue 8): the RP2040 SPI slave only
 * reloads its TX shift register from the TX FIFO when CS re-asserts
 * (HIGH→LOW edge). A multi-byte burst under a single CS assertion only
 * transfers byte 0 correctly in BOTH directions. Therefore the master
 * must toggle CS HIGH between every byte of the frame.
 *
 * Slave 0 (CS0=GP9) and Slave 1 (CS1=GP13) are both GPIO-driven here
 * (no hardware-framed CS), matching the proven spi_raw_test.c pattern.
 */
static void spi_exchange(uint8_t slave, const astra_spi_cmd_t *cmd, astra_spi_feedback_t *fb) {
    uint8_t cs = (slave == 1) ? SLAVE_CS1 : SLAVE_CS0;
    const uint8_t *tx = (const uint8_t*)cmd;
    uint8_t *rx = (uint8_t*)fb;

    // Deselect the non-selected slave so it never sees a spurious frame
    gpio_put((slave == 1) ? SLAVE_CS0 : SLAVE_CS1, 1);

    for (int i = 0; i < SPI_XFER_SIZE; i++) {
        gpio_put(cs, 0);
        busy_wait_us_32(10);
        spi_write_read_blocking(SLAVE_SPI, &tx[i], &rx[i], 1);
        busy_wait_us_32(10);
        gpio_put(cs, 1);
        busy_wait_us_32(50);
    }
}

/** Broadcast the config blob to both slaves over SPI (one frame per call). */
static void astra_distribute_config(void) {
    astra_spi_cmd_t cmd, cmd2;
    astra_spi_feedback_t fb, fb2;
    memset(&fb, 0, sizeof(fb));
    memset(&fb2, 0, sizeof(fb2));

    if (!cfg_await_ack) {
        // ── Frame phase: send one frame to each slave ──
        if (cfg_frame_idx < cfg_frame_total) {
            nite_cfg_make_frame(cfg_frame_idx, &g_cfg, &cmd);
            cmd2 = cmd;   // identical frame to both slaves
            spi_exchange(0, &cmd, &fb);
            spi_exchange(1, &cmd2, &fb2);
            cfg_frame_idx++;
            return;
        }
        cfg_await_ack = true;
        cfg_ack_deadline_us = time_us_64() + 2000000;  // 2s ack timeout
        return;
    }

    // ── Ack phase: poll with the CURRENT targets so slaves hold position ──
    // FIXED (review): sending zeroed targets here commanded all axes to 0
    // for the whole ack window (arm snapped to home on every config upload).
    memset(&cmd, 0, sizeof(cmd));
    memset(&cmd2, 0, sizeof(cmd2));
    memcpy(cmd.target, (void*)&current_targets.target[0], 16);
    memcpy(cmd2.target, (void*)&current_targets.target[4], 16);
    cmd.control_word = current_targets.enable_bits;
    cmd2.control_word = current_targets.enable_bits;
    cmd.crc = astra_crc8((uint8_t*)&cmd, sizeof(cmd) - 1);
    cmd2.crc = astra_crc8((uint8_t*)&cmd2, sizeof(cmd2) - 1);
    spi_exchange(0, &cmd, &fb);
    spi_exchange(1, &cmd2, &fb2);

    if (astra_crc8((uint8_t*)&fb, sizeof(fb) - 1) == fb.crc &&
        (fb.health_bits & ASTRA_CFG_ACK_BIT)) cfg_s1_acked = true;
    if (astra_crc8((uint8_t*)&fb2, sizeof(fb2) - 1) == fb2.crc &&
        (fb2.health_bits & ASTRA_CFG_ACK_BIT)) cfg_s2_acked = true;

    if (cfg_s1_acked && cfg_s2_acked) {
        cfg_send_active = false;
        printf("CFG-DONE both slaves acked\n");
    } else if (time_us_64() > cfg_ack_deadline_us) {
        cfg_send_active = false;
        printf("CFG-ERR ack timeout (S1=%d S2=%d)\n", cfg_s1_acked, cfg_s2_acked);
    }
}

static void astra_sync_slaves(void) {
    astra_spi_cmd_t s1_cmd, s2_cmd;
    astra_spi_feedback_t s1_fb, s2_fb;
    memset(&s1_fb, 0, sizeof(s1_fb));
    memset(&s2_fb, 0, sizeof(s2_fb));

    memcpy(s1_cmd.target, (void*)&current_targets.target[0], 16);
    memcpy(s2_cmd.target, (void*)&current_targets.target[4], 16);

    s1_cmd.control_word = current_targets.enable_bits;
    s2_cmd.control_word = current_targets.enable_bits;
    memset(s1_cmd.reserved, 0, sizeof(s1_cmd.reserved));
    memset(s2_cmd.reserved, 0, sizeof(s2_cmd.reserved));

    s1_cmd.crc = astra_crc8((uint8_t*)&s1_cmd, sizeof(s1_cmd) - 1);
    s2_cmd.crc = astra_crc8((uint8_t*)&s2_cmd, sizeof(s2_cmd) - 1);

    // CS-per-byte framing on both slaves (docs Issue 8)
    spi_exchange(0, &s1_cmd, &s1_fb);
    spi_exchange(1, &s2_cmd, &s2_fb);

    // SYNC pulse
    gpio_put(SYNC_PULSE_PIN, 1);
    sleep_us(5);
    gpio_put(SYNC_PULSE_PIN, 0);

    // Parse feedbacks with CRC validation
    if (astra_crc8((uint8_t*)&s1_fb, sizeof(s1_fb) - 1) == s1_fb.crc) {
        memcpy((void*)&telemetry_data.actual[0], s1_fb.actual, 16);
        for (int i = 0; i < 4; i++) {
            // FIXED: abs(error)*10 can exceed uint16_t range -> clamp.
            int32_t ma = abs(s1_fb.error[i]) * 10;
            telemetry_data.current_ma[i] = (ma > 65535) ? 65535 : (uint16_t)ma;
        }
        // FIXED: forward slave encoder health to UDP telemetry (bit1 = S1)
        if (s1_fb.health_bits) telemetry_data.status_flags |= 0x02;
        else                   telemetry_data.status_flags &= ~0x02;
    }

    if (astra_crc8((uint8_t*)&s2_fb, sizeof(s2_fb) - 1) == s2_fb.crc) {
        memcpy((void*)&telemetry_data.actual[4], s2_fb.actual, 16);
        for (int i = 0; i < 4; i++) {
            int32_t ma = abs(s2_fb.error[i]) * 10;
            telemetry_data.current_ma[i + 4] = (ma > 65535) ? 65535 : (uint16_t)ma;
        }
        // FIXED: forward slave encoder health to UDP telemetry (bit2 = S2)
        if (s2_fb.health_bits) telemetry_data.status_flags |= 0x04;
        else                   telemetry_data.status_flags &= ~0x04;
    }
}

// ==========================================================================
// USB Serial Command Handler
// ==========================================================================

#define SERIAL_BUF_SIZE    128
#define MAX_TOKENS         16

static char serial_buf[SERIAL_BUF_SIZE];
static int serial_buf_pos = 0;

static int tokenize(char *str, const char **tokens, int max_tokens) {
    int count = 0;
    char *p = str;
    while (*p && count < max_tokens) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tokens[count++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) { *p = '\0'; p++; }
    }
    return count;
}

static bool serial_read_line(void) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return false;
    if (c == '\n' || c == '\r') {
        if (serial_buf_pos > 0) {
            serial_buf[serial_buf_pos] = '\0';
            serial_buf_pos = 0;
            return true;
        }
        return false;
    }
    if (c < 32 && c != '\t') return false;
    if (serial_buf_pos < SERIAL_BUF_SIZE - 1) {
        serial_buf[serial_buf_pos++] = (char)c;
    }
    return false;
}

// ==========================================================================
// Config helpers (GRBL-style)
// ==========================================================================
static bool w5500_ready = false;

/** Apply the [MASTER] section to the running system (IP/MAC/port). */
static void cfg_apply_master(void) {
    memcpy(gWIZNETINFO.ip, g_cfg.master.ip, 4);
    memcpy(gWIZNETINFO.mac, g_cfg.master.mac, 6);
    if (w5500_ready) {
        wizchip_setnetinfo(&gWIZNETINFO);
    }
    // Port change -> close socket so the main loop reopens on the new port.
    // FIXED (review): only touch W5500 registers after it is initialized —
    // at boot the SPI for the W5500 is not configured yet.
    if (cfg_upload_port != g_cfg.master.port && w5500_ready) {
        if (getSn_SR(SOCKET_UDP) == SOCK_UDP) close(SOCKET_UDP);
        cfg_upload_port = g_cfg.master.port;
        printf("  UDP port -> %u\n", g_cfg.master.port);
    } else {
        cfg_upload_port = g_cfg.master.port;
    }
}

/** Persist the full blob to Master flash, then broadcast to both slaves. */
static void cfg_save_and_distribute(void) {
    if (!nite_cfg_save(&g_cfg, NITE_CFG_OFFSET_MASTER)) {
        printf("ERR flash save failed\n");
        return;
    }
    cfg_send_active = true;
    cfg_frame_idx = 0;
    cfg_frame_total = nite_cfg_frame_count();
    cfg_await_ack = false;
    cfg_s1_acked = false;
    cfg_s2_acked = false;
    cfg_ack_deadline_us = 0;
    printf("CFG-BROADCAST %u frames\n", cfg_frame_total);
}

/** Set a single $SECTION.key=value and persist + distribute on success. */
static void cfg_set_command(const char *key, const char *value) {
    if (!nite_cfg_set_key(&g_cfg, key, value)) {
        printf("ERR unknown key: %s\n", key);
        return;
    }
    cfg_apply_master();
    cfg_save_and_distribute();
    printf("OK %s=%s\n", key, value);
}

// ==========================================================================
// USB Serial / UDP Text Command Handler (GRBL-style config included)
// ==========================================================================

static void handle_text_line(char *line) {
    const char *tokens[MAX_TOKENS];
    int argc = tokenize(line, tokens, MAX_TOKENS);
    if (argc == 0) return;

    // ── Config upload mode: accumulate lines until CONFIG-END ──
    if (cfg_upload_active) {
        if (strcmp(tokens[0], "CONFIG-END") == 0) {
            cfg_upload_active = false;
            cfg_upload_buf[cfg_upload_len] = '\0';
            if (nite_cfg_parse_text(&g_cfg, cfg_upload_buf)) {
                cfg_apply_master();
                cfg_save_and_distribute();
                printf("OK config applied\n");
            } else {
                printf("ERR config parse failed\n");
            }
            cfg_upload_len = 0;
        } else {
            size_t l = strlen(line);
            if (cfg_upload_len + l + 1 < sizeof(cfg_upload_buf)) {
                memcpy(cfg_upload_buf + cfg_upload_len, line, l);
                cfg_upload_len += (uint32_t)l;
                cfg_upload_buf[cfg_upload_len++] = '\n';
            }
        }
        return;
    }

    // ── GRBL-style config commands ──
    if (strcmp(tokens[0], "$$") == 0) {
        char dump[NITE_CFG_TEXT_MAX];
        nite_cfg_format_text(&g_cfg, dump, sizeof(dump));
        printf("%s", dump);
        return;
    }
    if (tokens[0][0] == '$') {
        const char *eq = strchr(tokens[0], '=');
        if (eq) {
            char key[48];
            size_t kl = (size_t)(eq - tokens[0]) - 1;
            if (kl < sizeof(key)) {
                memcpy(key, tokens[0] + 1, kl);
                key[kl] = '\0';
                cfg_set_command(key, eq + 1);
            } else {
                printf("ERR key too long\n");
            }
        } else {
            printf("Use $SECTION.key=value  (e.g. $MASTER.ip=192.168.1.100)\n");
        }
        return;
    }
    if (strcmp(tokens[0], "CONFIG") == 0) {
        cfg_upload_active = true;
        cfg_upload_len = 0;
        printf("CFG-READY\n");
        return;
    }
    if (strcmp(tokens[0], "SAVE") == 0) {
        if (nite_cfg_save(&g_cfg, NITE_CFG_OFFSET_MASTER)) printf("OK saved\n");
        else printf("ERR save failed\n");
        return;
    }
    if (strcmp(tokens[0], "LOAD") == 0) {
        if (nite_cfg_load(&g_cfg, NITE_CFG_OFFSET_MASTER)) {
            cfg_apply_master();
            cfg_save_and_distribute();
            printf("OK loaded\n");
        } else {
            printf("ERR no valid config in flash\n");
        }
        return;
    }

    // ── Motion / status commands ──
    if (strcmp(tokens[0], "MOVEJ") == 0 || strcmp(tokens[0], "MOVEL") == 0) {
        if (argc < 7) {
            printf("ERR MOVE requires: j1 j2 j3 j4 j5 j6\n");
            return;
        }
        for (int i = 0; i < 6 && i < MAX_AXES; i++) {
            current_targets.target[i] = (int32_t)(atof(tokens[i + 1]) * 1000.0f);
        }
        for (int i = 6; i < MAX_AXES; i++) {
            current_targets.target[i] = 0;
        }
        telemetry_data.status_flags &= ~0x01;

        current_targets.magic = ASTRA_MAGIC;
        current_targets.sequence++;
        current_targets.enable_bits = 0xFF;
        current_targets.crc = astra_crc8((uint8_t*)&current_targets, sizeof(astra_udp_cmd_t) - 1);
        link_active = true;
        last_heartbeat_us = time_us_64();
        printf("OK\n");

    } else if (strcmp(tokens[0], "STATUS") == 0) {
        char resp[128];
        snprintf(resp, sizeof(resp), "POS %d %d %d %d %d %d %d %d",
            (int)(telemetry_data.actual[0] / 1000),
            (int)(telemetry_data.actual[1] / 1000),
            (int)(telemetry_data.actual[2] / 1000),
            (int)(telemetry_data.actual[3] / 1000),
            (int)(telemetry_data.actual[4] / 1000),
            (int)(telemetry_data.actual[5] / 1000),
            (int)(telemetry_data.actual[6] / 1000),
            (int)(telemetry_data.actual[7] / 1000));
        printf("%s\n", resp);

    } else if (strcmp(tokens[0], "STOP") == 0) {
        memset((void*)current_targets.target, 0, sizeof(current_targets.target));
        printf("OK\n");

    } else if (strcmp(tokens[0], "MODE") == 0) {
        printf("MODE %s\n", usb_mode ? "USB" : "LAN");

    } else if (strcmp(tokens[0], "ID") == 0) {
        // Stable identity for the host automation tools (pico_dev.py etc.)
        printf("NITE-MASTER\n");

    } else if (strcmp(tokens[0], "BOOTSEL") == 0) {
        // Software reboot straight into the USB bootloader (Method 3 in the
        // BOOTSEL notes): reset_usb_boot(0,0) -> no activity LED, all USB ifaces.
        printf("OK rebooting to BOOTSEL...\n");
        fflush(stdout);
        sleep_ms(50);
        reset_usb_boot(0, 0);

    } else if (strcmp(tokens[0], "HELP") == 0) {
        printf("Commands:\n");
        printf("  MOVEJ|MOVEL j1..j6, STATUS, STOP, MODE, ID, BOOTSEL\n");
        printf("  $$  dump config | $SECTION.key=value | CONFIG<upload> | SAVE | LOAD\n");

    } else {
        printf("ERR Unknown command: %s\n", tokens[0]);
    }
}

static void handle_serial_command(void) {
    handle_text_line(serial_buf);
}

// ==========================================================================
// Main
// ==========================================================================
int main() {
    stdio_init_all();
    sleep_ms(100);

    // Load configuration from flash (or factory defaults on first boot)
    if (nite_cfg_load(&g_cfg, NITE_CFG_OFFSET_MASTER)) {
        printf("Config loaded from flash\n");
    } else {
        nite_cfg_defaults(&g_cfg);
        printf("Config: factory defaults (none in flash yet)\n");
    }
    cfg_upload_port = g_cfg.master.port;
    cfg_apply_master();

    // Initialize MCP23017
    if (!mcp23017_init()) {
        printf("WARNING: MCP23017 not found — LEDs unavailable\n");
    } else {
        printf("MCP23017 ready\n");
    }

    usb_mode = mcp_read_dip_mode();
    printf("Nite 369 Master — Mode: %s\n", usb_mode ? "USB/SERIAL" : "LAN/ETHERNET");

    mcp_set_led(MCP_LED_POWER, true);
    mcp_set_led(MCP_LED_MODE, usb_mode);

    // Initialize W5500
    if (!w5500_init()) {
        printf("WARNING: W5500 init failed — Ethernet may not work\n");
    } else {
        w5500_ready = true;
        // Push the configured IP/MAC (may differ from the static initializer)
        wizchip_setnetinfo(&gWIZNETINFO);
    }

    // Initialize SPI1 for slave communication
    spi_init(SLAVE_SPI, 10 * 1000 * 1000);
    spi_set_format(SLAVE_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(SLAVE_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SLAVE_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SLAVE_MISO, GPIO_FUNC_SPI);

    // CS-per-byte: BOTH slave CS pins are GPIO-driven (no hardware CS).
    // The RP2040 slave only reloads TX on CS re-assert (docs Issue 8), so
    // spi_exchange() toggles CS per byte. See spi_exchange() above.
    gpio_init(SLAVE_CS0);
    gpio_set_dir(SLAVE_CS0, GPIO_OUT);
    gpio_put(SLAVE_CS0, 1);
    gpio_init(SLAVE_CS1);
    gpio_set_dir(SLAVE_CS1, GPIO_OUT);
    gpio_put(SLAVE_CS1, 1);

    gpio_init(SYNC_PULSE_PIN);
    gpio_set_dir(SYNC_PULSE_PIN, GPIO_OUT);
    gpio_put(SYNC_PULSE_PIN, 0);

    // Start safety watchdog (500us interval)
    struct repeating_timer timer;
    add_repeating_timer_us(-500, watchdog_timer_cb, NULL, &timer);

    printf("Nite 369 Master Controller Core Booted\n");
    printf("  FAULT LED = Error/E-Stop | LINK LED = UDP activity\n");

    uint8_t rx_buffer[128];
    uint8_t destip[4];
    uint16_t destport;
    uint32_t blink_counter = 0;

    while (1) {
        // Print watchdog trips from main-loop context (never from the ISR)
        if (watchdog_tripped) {
            watchdog_tripped = false;
            printf("ASTRA-ERROR: Watchdog Trip! Emergency Stop.\n");
        }

        // USB Serial commands
        if (usb_mode) {
            while (serial_read_line()) {
                handle_serial_command();
            }
        }

        // UDP socket management
        switch (getSn_SR(SOCKET_UDP)) {
            case SOCK_UDP: {
                uint16_t size = getSn_RX_RSR(SOCKET_UDP);
                if (size > 0) {
                    int32_t len = recvfrom(SOCKET_UDP, rx_buffer, sizeof(rx_buffer), destip, &destport);
                    if (len > 0) {
                        if (rx_buffer[0] != ASTRA_MAGIC) {
                            // UDP text command (GRBL-style config, $$, etc.)
                            int tlen = len < (int32_t)sizeof(rx_buffer) - 1 ? len : (int32_t)sizeof(rx_buffer) - 1;
                            rx_buffer[tlen] = '\0';
                            handle_text_line((char*)rx_buffer);
                        } else if (len >= (int32_t)sizeof(astra_udp_cmd_t)) {
                            astra_udp_cmd_t *cmd = (astra_udp_cmd_t*)rx_buffer;
                            if (cmd->magic == ASTRA_MAGIC &&
                                astra_crc8(rx_buffer, sizeof(astra_udp_cmd_t) - 1) == cmd->crc) {
                                telemetry_data.status_flags &= ~0x01;
                                memcpy((void*)&current_targets, cmd, sizeof(astra_udp_cmd_t));
                                last_heartbeat_us = time_us_64();
                                link_active = true;
                                udp_pkt_count++;
                                mcp_set_led(MCP_LED_LINK, true);

                                telemetry_data.magic = ASTRA_MAGIC;
                                telemetry_data.sequence_ack = cmd->sequence;
                                telemetry_data.crc = astra_crc8((uint8_t*)&telemetry_data,
                                                               sizeof(astra_udp_telemetry_t) - 1);
                                sendto(SOCKET_UDP, (uint8_t*)&telemetry_data,
                                       sizeof(astra_udp_telemetry_t), destip, destport);
                            }
                        }
                    }
                }
                blink_counter++;
                if (blink_counter >= 100) {
                    blink_counter = 0;
                    mcp_set_led(MCP_LED_LINK, false);
                }
                break;
            }
            case SOCK_CLOSED:
                socket(SOCKET_UDP, Sn_MR_UDP, g_cfg.master.port, 0x00);
                break;
            default:
                break;
        }

        // Config distribution to slaves (takes priority over motion sync)
        if (cfg_send_active) {
            astra_distribute_config();
        }
        // SPI sync to slaves
        else if (link_active) {
            astra_sync_slaves();
            if (telemetry_data.status_flags & 0x01) {
                mcp_set_led(MCP_LED_FAULT, true);
            } else {
                mcp_set_led(MCP_LED_FAULT, false);
            }
        }

        sleep_ms(1);
    }

    return 0;
}
