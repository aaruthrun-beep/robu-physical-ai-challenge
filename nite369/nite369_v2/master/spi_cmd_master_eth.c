/**
 * @file spi_cmd_master_eth.c
 * @brief Master Pico with W5500 Ethernet TCP server
 *
 * Accepts Nite369 commands over TCP (port 23) and forwards to slaves via SPI1.
 * W5500 connected on SPI0 (GP15-19).
 * Slaves on SPI1 (GP9-13).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "w5500_tcp.h"
#include "wizchip_conf.h"
#include "nite_spi_proto.h"
#include "gcode_parser.h"
#include "config_store.h"
#include "firmware_version.h"
#include "workspace_store.h"

// --- Slave SPI (SPI1) ---
#define SPI_PORT    spi1
#define PIN_SCK     10
#define PIN_MOSI    11
#define PIN_MISO    12
#define PIN_CS_S1   9
#define PIN_CS_S2   14   // was GP13 — GP13 is held high on this master (can't drive low)

// Frame length includes the CRC8 byte (see nite_spi_proto.h)
#define BUF_LEN NITE_SPI_BUF_LEN
// Slave 1/2 run at 50kHz (spi_init(SPI_PORT, 50000)); match it to keep both
// ends on the same clock. 100kHz was marginal -> bit flips (4D->4E/46) on
// longer wire runs.
#define SPI_SPEED   50000

static uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];
static int active_slave = 1;

// Forward declarations (used by process_gcode before their definitions)
static float cmd_pos[6];
static void read_encoder_positions(float enc[6]);

// Power-indicator heartbeat: toggles the onboard LED at 1Hz from a timer IRQ
// so a powered Pico always blinks, even while the main loop is busy/blocked.
static struct repeating_timer led_timer;
static bool led_state = false;
static bool led_timer_cb(struct repeating_timer *t) {
    (void)t;
    led_state = !led_state;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    return true;
}
static void led_heartbeat_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    add_repeating_timer_ms(500, led_timer_cb, NULL, &led_timer);  // 1Hz full blink
}

// Per-command 8-bit sequence for #MV (v2 seq byte). The slave uses it as a
// dedupe key so a retried (corrupted-response) frame is NOT re-executed
// after the move finished. Starts at 1; 0 is reserved (legacy/no-seq).
static uint8_t mv_seq = 0;

static void spi_txrx(int cs_pin) {
    // CS-PER-BYTE framing — validated on hardware (docs/spi_debug_issues.md
    // Issue 8, reference test/spi_raw_test.c). The RP2040 SPI0 slave only
    // reloads its TX shift register from the TX FIFO when CS re-asserts
    // (HIGH->LOW). A single multi-byte CS hold makes the slave echo byte 0
    // repeatedly (e.g. 4D 00 4D 00 4D 00 47 00 4D) -> constant BAD FRAME.
    // Toggle CS HIGH between every byte of the frame.
    //
    // The 100us pre-CS delay is critical: after the slave sees CS go LOW it
    // must poll spi_is_readable() and preload its TX FIFO before the master
    // clocks byte 0. Too short a delay -> the slave's byte-0 reply is sampled
    // mid-transition -> 4D arrives as 4E/46/4C/66 (low bits phase-shifted).
    // 10us was too short; 100us matches the settled timing seen on the wire.
    // Combined with slow-slew/2mA CS drive (set in main), this damps the
    // CS-edge ringing that corrupts byte 0 / byte 8 of each frame.
    for (int i = 0; i < BUF_LEN; i++) {
        gpio_put(cs_pin, 0);
        busy_wait_us_32(200);   // let CS settle low + slave preload TX before first clock
        spi_write_read_blocking(SPI_PORT, &out_buf[i], &in_buf[i], 1);
        busy_wait_us_32(100);   // hold after clock so the byte completes
        gpio_put(cs_pin, 1);
        busy_wait_us_32(500);   // let CS settle high long enough for the bit-bang
                                // slave to catch the deassert before the next byte
    }
}

static int get_cs(void) {
    return active_slave == 2 ? PIN_CS_S2 : PIN_CS_S1;
}

// ── v2 SPI layer ────────────────────────────────────────────────────
// Two-frame SPI protocol: frame 1 sends the v2 command, frame 2 reads the
// v2 response. Commands are packed with frame_v2_pack (sync 0xA5 + opcode
// + payload + seq + CRC8); responses are validated with frame_v2_unpack and
// mapped back to the LEGACY status convention (NITE_RSP_OK/BUSY/LIMIT/ERR)
// so the retry logic and tcp_report_slave_result keep working unchanged.
//
// Non-blocking: this runs inside the W5500 TCP poll loop, so blocking
// sleep_ms()/printf() here starves the Ethernet stack. Pacing uses
// busy_wait so the W5500 keeps being serviced.

// Last valid v2 reply frame + its mapped legacy status.
static spi_frame_t g_reply;
static uint8_t g_reply_status = NITE_RSP_ERR;

// Map a validated v2 reply frame to the legacy status byte.
static uint8_t v2_status_from_reply(const spi_frame_t *r) {
    switch (r->opcode) {
        case OP_ACK:
        case OP_PONG:
        case OP_MOTION_REPLY:
        case OP_ENCODER_REPLY:
        case OP_LIMIT_REPLY:
        case OP_HOMING_REPLY:
        case OP_CFG_REPLY:
        case OP_TMC_REPLY:
            return NITE_RSP_OK;
        case OP_BUSY:
            return NITE_RSP_BUSY;
        case OP_FAULT:
            // Soft-limit clamp (payload = FAULT_SOFT_LIMIT) is a definitive
            // "no move" — report it as NITE_RSP_LIMIT so the studio sees why.
            if (r->payload == FAULT_SOFT_LIMIT) return NITE_RSP_LIMIT;
            return NITE_RSP_ERR;
        default:
            return NITE_RSP_ERR;
    }
}

// Send one v2 command frame and read the reply. Returns the mapped legacy
// status; on CRC/sync failure returns NITE_RSP_BAD_FRAME.
static uint8_t v2_send(uint8_t opcode, uint8_t joint_id, int32_t payload, uint8_t seq) {
    spi_frame_t cmd = {
        .sync = FRAME_SYNC_BYTE,
        .joint_id = joint_id,
        .opcode = opcode,
        .payload = payload,
        .seq = seq,
    };
    frame_v2_pack(&cmd, out_buf);
    spi_txrx(get_cs());
    busy_wait_us_32(50000);  // 50ms gap: slave 2's bit-bang shares the bus and
                             // loads the lines; 15ms was too tight -> slave 1
                             // clocked out its reply one frame late (zeros).
    memset(out_buf, 0, BUF_LEN);
    spi_txrx(get_cs());
    if (!frame_v2_unpack(in_buf, &g_reply)) {
        g_reply_status = NITE_RSP_BAD_FRAME;
        return g_reply_status;
    }
    g_reply_status = v2_status_from_reply(&g_reply);
    return g_reply_status;
}

// Retry version — retries on ANY non-OK response (busy, stale, bad frame, error)
// because SPI corruption can cause transient garbage replies too.
// The corruption observed on this link is BURSTY (runs of bad frames followed
// by clean ones), so use a sustained retry window rather than a fixed small
// count: 30 attempts x 15ms = ~450ms of retrying. This catches the clean
// frames between bursts and turns transient noise into a transparent retry
// instead of a reported >ER:SLAVE(F0).
static uint8_t v2_send_retry(uint8_t opcode, uint8_t joint_id, int32_t payload, uint8_t seq) {
    uint8_t last_result = NITE_RSP_ERR;
    uint32_t waited_us = 0;
    for (int attempt = 0; attempt < 30; attempt++) {
        uint8_t result = v2_send(opcode, joint_id, payload, seq);
        last_result = result;
        if (result == NITE_RSP_OK) return NITE_RSP_OK;
        // Soft-limit clamp is a definitive "no move" — don't retry.
        if (result == NITE_RSP_LIMIT) return result;
        if (waited_us >= 350000) return last_result;
        // Busy: the slave is mid-move. The slave now REPLACES a new move
        // (latest-wins jog), so BUSY is rare — but if it happens, wait
        // briefly for the move to finish.
        if (result == NITE_RSP_BUSY) {
            busy_wait_us_32(20000);
            waited_us += 20000;
            continue;
        }
        busy_wait_us_32(15000);
        waited_us += 15000;
    }
    return last_result;
}

// Bounded variant for STATUS/ENCODER queries: 2 attempts, short gap.
// A single retry catches transient MISO corruption without the 30x
// retry storm that made #P/#MS take seconds (starving moves).
static uint8_t v2_send_try2(uint8_t opcode, uint8_t joint_id, int32_t payload, uint8_t seq) {
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t result = v2_send(opcode, joint_id, payload, seq);
        if (result == NITE_RSP_OK) return result;
        busy_wait_us_32(5000);
    }
    return v2_send(opcode, joint_id, payload, seq);
}

// Ping one slave with retries. The heartbeat runs every 1s and the SPI bus
// corrupts ~1 in 10 frames at the boundary bytes, so a single failed ping is
// NOT proof the slave is dead — a corrupted ping frame gets a BAD_FRAME reply
// and would falsely mark a healthy slave DEAD. Retry a few times; only give
// up if every attempt fails.
static bool ping_slave(int slave) {
    active_slave = slave;
    bool alive = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t r = v2_send(OP_PING, JOINT_NONE, 0, (uint8_t)attempt);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_PONG) { alive = true; break; }
        busy_wait_us_32(20000);
    }
    active_slave = 1;
    printf("[PING s%d] rx: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
           slave, in_buf[0], in_buf[1], in_buf[2], in_buf[3], in_buf[4],
           in_buf[5], in_buf[6], in_buf[7], in_buf[8]);
    return alive;
}

// Slow version for TMC UART commands (needs processing time)
static void v2_send_slow(uint8_t opcode, uint8_t joint_id, int32_t payload, uint8_t seq) {
    v2_send(opcode, joint_id, payload, seq);
}

// Flash commands (config save / reset): the slave erases+programs flash
// inside the frame handler, which blocks its SPI loop for ~100-200ms. If the
// master reads the response too early, the slave misses the CS edges, the
// frame-resync resets alignment, and the reply is corrupt (slave appears
// DEAD). 400ms comfortably covers erase+program+verify.
static void v2_send_flash(uint8_t opcode, uint8_t joint_id, int32_t payload, uint8_t seq) {
    spi_frame_t cmd = {
        .sync = FRAME_SYNC_BYTE,
        .joint_id = joint_id,
        .opcode = opcode,
        .payload = payload,
        .seq = seq,
    };
    frame_v2_pack(&cmd, out_buf);
    spi_txrx(get_cs());
    busy_wait_us_32(400000);
    memset(out_buf, 0, BUF_LEN);
    spi_txrx(get_cs());
    if (!frame_v2_unpack(in_buf, &g_reply)) {
        g_reply_status = NITE_RSP_BAD_FRAME;
    } else {
        g_reply_status = v2_status_from_reply(&g_reply);
    }
}

// Capture printf output into TCP send buffer
static char tcp_tx_buf[512];
static int tcp_tx_pos = 0;

static int tcp_printf_capture(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tcp_tx_buf + tcp_tx_pos, sizeof(tcp_tx_buf) - tcp_tx_pos, fmt, args);
    va_end(args);
    if (n > 0) tcp_tx_pos += n;
    return n;
}

// Map a slave response byte to a readable error for the TCP client.
// 0xFE = axis still moving (NITE_RSP_BUSY) -> >ER:BUSY (not a fault)
// 0xFD = move clamped at a soft limit -> >ER:LIMIT (no motion)
// 0xF0 = frame CRC failed on the SPI link after all retries -> >ER:SLAVE(F0)
// anything else -> >ER:SLAVE(XX)
static void tcp_report_slave_result(uint8_t r) {
    if (r == NITE_RSP_OK) {
        tcp_printf_capture(">OK\n");
    } else if (r == NITE_RSP_BUSY) {
        tcp_printf_capture(">ER:BUSY\n");
    } else if (r == 0xFD) {
        tcp_printf_capture(">ER:LIMIT\n");
    } else {
        tcp_printf_capture(">ER:SLAVE(%02X)\n", r);
    }
}

// --- Nite369 Command Processor ---
// Must match spi_cmd_master.c's nite_process exactly

// G-code axis letters -> joint numbers (X→1, Y→2, Z→3, A→4, B→5, C→6)
static const char GCODE_AXIS_LETTERS[6] = {'X','Y','Z','A','B','C'};

// Commanded joint positions (degrees) for G90 absolute mode.
static float gcode_cmd_pos[6] = {0,0,0,0,0,0};
static bool gcode_absolute = true;   // G90 default

// Send a relative move for one joint (degrees -> steps, via config).
// Returns true if a valid move was sent.
static bool gcode_move_joint(int joint_no, float delta_deg, float feed_deg_min) {
    if (joint_no < 1 || joint_no > 6) return false;
    int axis = (joint_no - 1) % 3;
    int slave = (joint_no <= 3) ? 1 : 2;
    uint32_t spd = config_steps_per_deg(axis);   // steps per degree (gear-aware)
    if (spd == 0) spd = 1;
    int32_t steps = (int32_t)(delta_deg * (float)spd);
    if (steps == 0) return true;   // nothing to move

    // Feed rate: deg/min -> steps/sec
    float feed_steps_sec = (feed_deg_min > 0) ? feed_deg_min * (float)spd / 60.0f : 200.0f;
    if (feed_steps_sec < 1) feed_steps_sec = 1;
    uint32_t speed = (uint32_t)feed_steps_sec;
    if (speed > 65535) speed = 65535;
    if (steps > 32767) steps = 32767;
    if (steps < -32767) steps = -32767;

    if (++mv_seq == 0) mv_seq = 1;   // 0 reserved
    active_slave = slave;
    uint8_t r = v2_send_retry(OP_STEP_DELTA, (uint8_t)axis,
                              V2_PAYLOAD_MOVE(speed, steps), mv_seq);
    active_slave = 1;
    return (r == NITE_RSP_OK);
}

// Handle a G-code / M-code line (G1 X10 Y0 F500, M92 X250, ...).
// Returns true if the line was a G/M/T command.
static bool process_gcode(const char *cmd_str) {
    char buf[GCODE_MAX_LINE];
    size_t n = strlen(cmd_str);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    memcpy(buf, cmd_str, n);
    buf[n] = '\0';

    if (!gcode_parse(buf)) return false;

    char cmd = gcode_command();
    int32_t code = gcode_codenum();

    // ---------- G codes ----------
    if (cmd == 'G') {
        switch (code) {
            case 0: case 1: {   // G0/G1 linear move
                float feed = gcode_floatval('F', 0);
                // For each axis present, compute delta and move.
                bool all_ok = true;
                for (int i = 0; i < 6; i++) {
                    char L = GCODE_AXIS_LETTERS[i];
                    if (!gcode_seen(L)) continue;
                    float target = gcode_floatval(L, 0);
                    float delta;
                    if (gcode_absolute) {
                        delta = target - gcode_cmd_pos[i];
                        gcode_cmd_pos[i] = target;
                    } else {
                        delta = target;
                        gcode_cmd_pos[i] += target;
                    }
                    if (delta != 0 && !gcode_move_joint(i + 1, delta, feed)) {
                        all_ok = false;
                    }
                }
                if (all_ok) {
                    tcp_printf_capture(">OK\n");
                } else {
                    tcp_printf_capture(">ER:MOVE_FAIL\n");
                }
                return true;
            }
            case 4:  // G4 dwell
                sleep_ms(gcode_intval('P', 0) > 0 ? (uint32_t)gcode_intval('P', 0) : 0);
                tcp_printf_capture(">OK\n");
                return true;
            case 28: { // G28 home all joints
                tcp_printf_capture(">HOMING:\n");
                active_slave = 1; v2_send(OP_HOME, JOINT_NONE, 0, 0);
                active_slave = 2; v2_send(OP_HOME, JOINT_NONE, 0, 0);
                active_slave = 1;
                // Wait for homing to complete (poll for ~30s timeout)
                for (int t = 0; t < 300; t++) {
                    sleep_ms(100);
                    bool s1_moving = false, s2_moving = false;
                    for (int a = 0; a < 3; a++) {
                        active_slave = 1;
                        if (v2_send_try2(OP_HOMING_STATUS, (uint8_t)a, 0, 0) == NITE_RSP_OK
                            && (g_reply.payload & 0xFF) == 1) s1_moving = true;
                        active_slave = 2;
                        if (v2_send_try2(OP_HOMING_STATUS, (uint8_t)a, 0, 0) == NITE_RSP_OK
                            && (g_reply.payload & 0xFF) == 1) s2_moving = true;
                    }
                    active_slave = 1;
                    if (!s1_moving && !s2_moving) break;
                }
                // Reset commanded position to zeros after homing
                for (int i = 0; i < 6; i++) { cmd_pos[i] = 0; gcode_cmd_pos[i] = 0; }
                tcp_printf_capture(">OK\n");
                return true;
            }
            case 92: { // G92 set position (no move)
                for (int i = 0; i < 6; i++) {
                    char L = GCODE_AXIS_LETTERS[i];
                    if (!gcode_seen(L)) continue;
                    float val = gcode_floatval(L, 0);
                    cmd_pos[i] = val;
                    gcode_cmd_pos[i] = val;
                }
                tcp_printf_capture(">OK\n");
                return true;
            }
            case 90: gcode_absolute = true;  tcp_printf_capture(">OK\n"); return true;
            case 91: gcode_absolute = false; tcp_printf_capture(">OK\n"); return true;
            default:
                tcp_printf_capture(">ER:UNSUPPORTED_G%ld\n", (long)code);
                return true;
        }
    }

    // ---------- M codes ----------
    if (cmd == 'M') {
        switch (code) {
            case 114: { // M114 report actual encoder positions
                float enc[6];
                read_encoder_positions(enc);
                tcp_printf_capture(">E:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    enc[0], enc[1], enc[2], enc[3], enc[4], enc[5]);
                return true;
            }
            case 92: {   // M92 set steps/unit per axis
                for (int i = 0; i < 6; i++) {
                    char L = GCODE_AXIS_LETTERS[i];
                    if (!gcode_seen(L)) continue;
                    float spu = gcode_floatval(L, 0);
                    if (spu <= 0) continue;
                    config_t *c = config_get_mut();
                    int axis = i % 3;  // shared per slave (slave1: 0-2, slave2: 0-2)
                    // steps/unit (deg) -> steps_per_rev = spu * 360
                    c->axes[axis].steps_per_rev = (uint32_t)(spu * 360.0f);
                }
                tcp_printf_capture(">OK\n");
                return true;
            }
            case 203: {  // M203 set max feedrate (deg/min) per axis
                for (int i = 0; i < 6; i++) {
                    char L = GCODE_AXIS_LETTERS[i];
                    if (!gcode_seen(L)) continue;
                    float v = gcode_floatval(L, 0);
                    if (v <= 0) continue;
                    config_t *c = config_get_mut();
                    int axis = i % 3;
                    uint32_t spd = config_steps_per_deg(axis);
                    if (spd == 0) spd = 1;
                    // deg/min -> steps/sec
                    c->axes[axis].max_speed = (uint32_t)(v * (float)spd / 60.0f);
                }
                tcp_printf_capture(">OK\n");
                return true;
            }
            case 201: {  // M201 set accel (deg/s^2) per axis
                for (int i = 0; i < 6; i++) {
                    char L = GCODE_AXIS_LETTERS[i];
                    if (!gcode_seen(L)) continue;
                    float v = gcode_floatval(L, 0);
                    if (v <= 0) continue;
                    config_t *c = config_get_mut();
                    int axis = i % 3;
                    uint32_t spd = config_steps_per_deg(axis);
                    if (spd == 0) spd = 1;
                    c->axes[axis].accel = (uint32_t)(v * (float)spd);
                    c->axes[axis].decel = c->axes[axis].accel;
                }
                tcp_printf_capture(">OK\n");
                return true;
            }
            case 500:  // M500 save config
                config_save();
                tcp_printf_capture(">OK\n");
                return true;
            case 501:  // M501 load config
                config_init();
                tcp_printf_capture(">OK\n");
                return true;
            case 503: {  // M503 report settings (gear-aware steps/deg)
                tcp_printf_capture(">M92 X%.2f Y%.2f Z%.2f A%.2f B%.2f C%.2f\n",
                    (float)config_steps_per_deg(0),
                    (float)config_steps_per_deg(1),
                    (float)config_steps_per_deg(2),
                    (float)config_steps_per_deg(3),
                    (float)config_steps_per_deg(4),
                    (float)config_steps_per_deg(5));
                return true;
            }
            case 18: case 84: {  // disable motors
                active_slave = 1; v2_send_retry(OP_ENABLE, JOINT_NONE, 0, 0); sleep_ms(2);
                active_slave = 2; v2_send_retry(OP_ENABLE, JOINT_NONE, 0, 0); sleep_ms(2);
                active_slave = 1;
                tcp_printf_capture(">OK\n");
                return true;
            }
            default:
                tcp_printf_capture(">ER:UNSUPPORTED_M%ld\n", (long)code);
                return true;
        }
    }

    return false;  // not a G/M command
}

// Read both slaves' per-axis motion config into the master's local mirror:
//   OP_CFG_READ fields -> max_speed/accel/decel, steps_per_rev, gear_ratio
// so #M's coordinated-move computation uses the SAME values the slaves run
// (slave 2 forces wrist values at boot that differ from the master's
// defaults). Fast bounded reads; a rare miss keeps the previous value.
static void refresh_slave_configs(void) {
    config_t *mc = config_get_mut();
    for (int slave = 1; slave <= 2; slave++) {
        active_slave = slave;
        for (int axis = 0; axis < 3; axis++) {
            int g = (slave == 1) ? axis : axis + 3;  // master axis index 0-5
            uint8_t r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_MAX_SPEED, axis), 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) {
                mc->axes[g].max_speed = (uint32_t)g_reply.payload;
            }
            r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_ACCEL, axis), 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) {
                mc->axes[g].accel = (uint32_t)g_reply.payload;
            }
            r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_DECEL, axis), 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) {
                mc->axes[g].decel = (uint32_t)g_reply.payload;
            }
            r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_STEPS_REV, axis), 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) {
                mc->axes[g].steps_per_rev = (uint32_t)g_reply.payload;
            }
            r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_GEAR_RATIO, axis), 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) {
                mc->axes[g].gear_ratio = (uint32_t)g_reply.payload;
            }
        }
    }
    active_slave = 1;
}

// ── Software home position ──────────────────────────────────────────
// Commanded joint position (degrees) as tracked by the master, and the
// stored software home. #SH sets home = current; #GH moves back to it.
static float cmd_pos[6] = {0,0,0,0,0,0};
static float home_pos[6] = {0,0,0,0,0,0};
static bool home_set = false;

// ── Real-time position streaming ─────────────────────────────────────
static uint32_t rt_stream_hz = 0;        // 0 = off, N = send position N times/sec
static uint32_t rt_stream_last_ms = 0;

// ── Trajectory buffer (ring buffer for G0/G1 lines) ─────────────────
#define TRAJ_BUF_SIZE 32
static struct {
    float target[6];   // target joint positions (absolute, degrees)
    float feed;        // feed rate (deg/min)
    bool  valid;
} traj_buf[TRAJ_BUF_SIZE];
static int traj_head = 0;   // next write slot
static int traj_tail = 0;   // next read slot (execute)
static int traj_count = 0;  // items in buffer
static bool traj_executing = false;

// ── Macro recording ──────────────────────────────────────────────────
static bool mac_recording = false;

// ── Helper: read encoder positions from both slaves ──────────────────
static void read_encoder_positions(float enc[6]) {
    memset(enc, 0, sizeof(float) * 6);
    active_slave = 1;
    for (int i = 0; i < 4; i++) {
        uint8_t r = v2_send(OP_ENCODER_READ, (uint8_t)i, 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_ENCODER_REPLY) {
            uint16_t angle = (uint16_t)(g_reply.payload & 0xFFFFu);
            float deg = angle * 360.0f / 4096.0f;
            if (i == 0) enc[0] = deg;
            else if (i == 1) enc[1] = deg;
            else if (i == 3) enc[2] = deg;
        }
        sleep_ms(2);
    }
    active_slave = 2;
    for (int i = 0; i < 3; i++) {
        uint8_t r = v2_send(OP_ENCODER_READ, (uint8_t)i, 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_ENCODER_REPLY) {
            uint16_t angle = (uint16_t)(g_reply.payload & 0xFFFFu);
            enc[3 + i] = angle * 360.0f / 4096.0f;
        }
        sleep_ms(2);
    }
    active_slave = 1;
}

// Coordinated multi-axis move by DELTA degrees per joint (negative moves
// reverse). Computes a common duration T (longest axis at max speed), scales
// each axis's cruise speed to finish in T, stages all axes, then GO. Updates
// cmd_pos[] by the deltas. Returns true on success.
static bool coordinated_move(const float delta_deg[6]) {
    // Use the GLOBAL per-axis config (steps_per_rev / gear_ratio / max_speed /
    // accel / decel) instead of hardcoded arrays, so a #CFG or #CF write
    // changes how #M moves immediately — no recompile. The config mirror is
    // kept current by refresh_slave_configs() at boot and by every #CF/#CFG.
    const config_t *cfg = config_get();

    // The SPI frame's step field is int16 (±32767), so a single #M cannot
    // carry a bigger per-axis delta than that. For larger moves we CHUNK:
    // split the requested delta into N sub-moves, each within the frame
    // budget, and run them back-to-back. This makes multi-axis continuous
    // motion work past ~80° on J2 — the old code silently truncated at 32767.
    const int32_t MAX_FRAME_STEPS = 32400;

    int32_t steps[6] = {0};      // FULL delta in steps (int32 — can exceed 32767)
    uint32_t acc[6], dec[6], vmax[6];
    bool active[6] = {false, false, false, false, false, false};
    int nchunks = 1;

    for (int j = 0; j < 6; j++) {
        uint32_t spd = config_steps_per_deg(j);
        if (spd == 0) spd = 1;
        int64_t s64 = (int64_t)(delta_deg[j] * (float)spd);
        if (s64 == 0) continue;
        // Keep the FULL signed delta (no int16 clamp — chunking handles size).
        steps[j] = (int32_t)s64;
        // GLOBAL per-axis motion profile (sanitized like the slaves do).
        vmax[j] = cfg->axes[j].max_speed;
        acc[j]  = cfg->axes[j].accel;
        dec[j]  = cfg->axes[j].decel;
        if (vmax[j] < 1 || vmax[j] > CONFIG_MAX_SPEED_LIMIT) vmax[j] = 8000;
        if (acc[j]  < 1000 || acc[j]  > 200000) acc[j] = 6000;
        if (dec[j]  < 1000 || dec[j]  > 200000) dec[j] = 20000;
        active[j] = true;

        int64_t abs_s = s64 < 0 ? -s64 : s64;
        int need = (int)(abs_s / MAX_FRAME_STEPS) + 1;
        if (need > nchunks) nchunks = need;
    }
    if (nchunks < 1) nchunks = 1;

    // Execute the move in chunks so no single frame exceeds the int16 budget.
    for (int chunk = 0; chunk < nchunks; chunk++) {
        // Split this chunk evenly; the last chunk absorbs rounding so the
        // total exactly matches the requested delta.
        bool active_this[6] = {false, false, false, false, false, false};
        int32_t csteps[6] = {0};
        for (int j = 0; j < 6; j++) {
            if (!active[j]) continue;
            int32_t total = steps[j];
            int32_t c = total / nchunks;
            if (chunk == nchunks - 1) c = total - c * (nchunks - 1);
            csteps[j] = c;
            if (c != 0) active_this[j] = true;
        }

        // Common duration T = longest axis at its max speed (time-based trapezoid).
        float T_sec = 0;
        for (int j = 0; j < 6; j++) {
            if (!active_this[j]) continue;
            int32_t S = csteps[j] < 0 ? -csteps[j] : csteps[j];
            float v = (float)vmax[j];
            float a = (float)acc[j], d = (float)dec[j];
            if (a < 1) a = 1; if (d < 1) d = 1;
            float ramp_t = v / a + v / d;
            float ramp_s = v * v / (2.0f * a) + v * v / (2.0f * d);
            float t;
            if ((float)S <= ramp_s) {
                t = sqrtf(2.0f * (float)S * (1.0f / a + 1.0f / d));
            } else {
                t = ramp_t + ((float)S - ramp_s) / v;
            }
            if (t > T_sec) T_sec = t;
        }
        if (T_sec < 0.001f) T_sec = 0.001f;
        printf("[M] chunk=%d/%d T=%.3fs\n", chunk + 1, nchunks, T_sec);

        // Scale each axis's cruise speed to finish in T_sec (bisection).
        uint16_t cruise[6];
        for (int j = 0; j < 6; j++) {
            if (!active_this[j]) continue;
            int32_t S = csteps[j] < 0 ? -csteps[j] : csteps[j];
            float a = (float)acc[j], d = (float)dec[j];
            if (a < 1) a = 1; if (d < 1) d = 1;
            float vmaxf = (float)vmax[j];
            float lo = 1.0f, hi = vmaxf;
            for (int it = 0; it < 24; it++) {
                float v = (lo + hi) * 0.5f;
                float ramp_t = v / a + v / d;
                float ramp_s = v * v / (2.0f * a) + v * v / (2.0f * d);
                float t;
                if ((float)S <= ramp_s) {
                    t = sqrtf(2.0f * (float)S * (1.0f / a + 1.0f / d));
                } else {
                    t = ramp_t + ((float)S - ramp_s) / v;
                }
                if (t > T_sec) hi = v; else lo = v;
            }
            float best = (lo + hi) * 0.5f;
            if (best > vmaxf) best = vmaxf;
            if (best < 1.0f) best = 1.0f;
            cruise[j] = (uint16_t)best;
        }

        // STAGE all axes (HOLD flag), then GO.
        for (int i = 0; i < 3; i++) {
            if (!active_this[i]) continue;
            active_slave = 1;
            v2_send_retry(OP_STEP_DELTA, (uint8_t)i | V2_JID_HOLD,
                          V2_PAYLOAD_MOVE(cruise[i], csteps[i]), 0);
        }
        for (int i = 3; i < 6; i++) {
            if (!active_this[i]) continue;
            active_slave = 2;
            v2_send_retry(OP_STEP_DELTA, (uint8_t)(i - 3) | V2_JID_HOLD,
                          V2_PAYLOAD_MOVE(cruise[i], csteps[i]), 0);
        }
        // GO: release both slaves' staged moves simultaneously.
        active_slave = 1;
        v2_send_retry(OP_GO, JOINT_NONE, 0, 0);
        active_slave = 2;
        v2_send_retry(OP_GO, JOINT_NONE, 0, 0);
        active_slave = 1;

        // Wait for this chunk to finish before staging the next one. The
        // slaves only accept a HOLD (staged) move while IDLE; staging while a
        // move is in flight would be queued as a plain move and the GO would
        // start it out of sync. Poll the motion status (moving flag in the
        // reply seq byte) on the axes we started. Timeout (~15s) so a stuck
        // axis can't hang the command channel forever.
        if (chunk < nchunks - 1) {
            uint32_t waited_ms = 0;
            while (waited_ms < 15000) {
                bool any_moving = false;
                for (int j = 0; j < 6; j++) {
                    if (!active_this[j]) continue;
                    int axis = (j < 3) ? j : j - 3;
                    active_slave = (j < 3) ? 1 : 2;
                    uint8_t r = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
                    if (r == NITE_RSP_OK && (g_reply.seq & 1)) any_moving = true;
                }
                active_slave = 1;
                if (!any_moving) break;
                busy_wait_us_32(5000);  // 5ms poll
                waited_ms += 5;
            }
        }
    }

    // Update commanded position.
    for (int j = 0; j < 6; j++) cmd_pos[j] += delta_deg[j];
    return true;
}

static int process_nite369(const char *cmd_str) {
    tcp_tx_pos = 0;

    // Macro recording: append every command (except recording commands) to the buffer
    if (mac_recording && cmd_str[0] != '\0') {
        if (!(cmd_str[0] == 'M' && cmd_str[1] == 'A' && cmd_str[2] == 'C')) {
            mac_record_step(cmd_str);
        }
    }

    // Legacy #G<steps> gripper — a bare "G<number>" with NO spaces/params
    // (e.g. G200). Must be checked BEFORE the G-code path, because
    // process_gcode() accepts ANY G<number> and would reply
    // ">ER:UNSUPPORTED_G200" for pure gripper commands like G200.
    // Maps steps (0..5000, GUI convention) to servo angle: 0 steps = closed
    // (0 deg), 5000 = open (180 deg), sent as deci-degrees to slave 2 (0x58).
    if (cmd_str[0] == 'G') {
        const char *q = cmd_str + 1;
        bool digits_only = (*q != '\0');
        while (*q) {
            if (*q < '0' || *q > '9') { digits_only = false; break; }
            q++;
        }
        if (digits_only) {
            int steps = atoi(cmd_str + 1);
            if (steps < 0) steps = 0;
            if (steps > 5000) steps = 5000;
            int angle_10 = steps * 1800 / 5000;   // 0..1800 deci-degrees
            active_slave = 2;
            v2_send_retry(OP_GRIPPER, JOINT_NONE, angle_10, 0);
            active_slave = 1;
            tcp_printf_capture(">OK\n");
            return tcp_tx_pos;
        }
    }

    // G-code / M-code lines (G1 X10 F500, M92 X250, ...) — check FIRST so
    // they don't collide with the legacy single-letter commands (#G, #M).
    // Only treat it as G/M code when the command number is DIGITS: "#MV..."
    // is the Nite raw-move command (parsed as "M0" by the gcode parser and
    // would wrongly reply >ER:UNSUPPORTED_M0). ALSO skip the legacy
    // multi-joint #M10,20,30,... (6 joint angles): that's "M10" followed by
    // a comma, which the gcode parser would read as M10 and reply
    // >ER:UNSUPPORTED_M10.
    if ((cmd_str[0] == 'G' || cmd_str[0] == 'M') &&
        cmd_str[1] >= '0' && cmd_str[1] <= '9') {
        // If it's "M<number>," it's the legacy #M<j1>,<j2>,... form.
        // Accept floats too ("M2.5,..."): the studio's driver sends degrees
        // with decimals, and a dot would otherwise send it down the G-code
        // path as "M2" -> >ER:UNSUPPORTED_M2. A leading '-' or '.' is
        // allowed so negative/float deltas are not misread as G/M lines.
        bool legacy_m = false;
        if (cmd_str[0] == 'M') {
            const char *q = cmd_str + 1;
            if (*q == '-' || *q == '+') q++;
            while ((*q >= '0' && *q <= '9') || *q == '.') q++;
            if (*q == ',') legacy_m = true;
        }
        if (!legacy_m && process_gcode(cmd_str)) {
            return tcp_tx_pos;
        }
        // fall through if not actually a G/M line
    }

    // Multi-char commands (check BEFORE single-char)
    if (cmd_str[0] == 'E' && cmd_str[1] == 'N') {
        // #EN<mask> — enable drivers (OP_ENABLE payload 1 = on).
        uint32_t mask = strtoul(cmd_str + 2, NULL, 16);
        uint8_t r = 0;
        if (mask & 0x07) {
            active_slave = 1;
            r = v2_send_retry(OP_ENABLE, JOINT_NONE, 1, 0); sleep_ms(2);
        }
        if (mask & 0x38) {
            active_slave = 2;
            r = v2_send_retry(OP_ENABLE, JOINT_NONE, 1, 0); sleep_ms(2);
        }
        active_slave = 1;
        tcp_report_slave_result(r);
    }
    else if (cmd_str[0] == 'D' && cmd_str[1] == 'I') {
        // #DI<mask> — disable drivers (OP_ENABLE payload 0 = off).
        uint32_t mask = strtoul(cmd_str + 2, NULL, 16);
        uint8_t r = 0;
        if (mask & 0x07) {
            active_slave = 1;
            r = v2_send_retry(OP_ENABLE, JOINT_NONE, 0, 0); sleep_ms(2);
        }
        if (mask & 0x38) {
            active_slave = 2;
            r = v2_send_retry(OP_ENABLE, JOINT_NONE, 0, 0); sleep_ms(2);
        }
        active_slave = 1;
        tcp_report_slave_result(r);
    }
    else if (cmd_str[0] == 'E' && cmd_str[1] == 'R') {
        tcp_printf_capture(">ER:UNKNOWN_CMD\n");
    }
    else if (cmd_str[0] == 'P' && cmd_str[1] != 'I') {
        // #P — OPEN-LOOP motion positions (fast). Reads the motion counters
        // via OP_MOTION_STATUS — 6 quick reads.
        float pos[6] = {0};
        for (int j = 1; j <= 6; j++) {
            int axis = (j - 1) % 3;
            active_slave = (j <= 3) ? 1 : 2;
            uint8_t r = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_MOTION_REPLY) {
                pos[j - 1] = (float)V2_STATUS_POS(g_reply.payload);
            }
        }
        active_slave = 1;
        tcp_printf_capture(">P:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
               pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    }
    else if (cmd_str[0] == 'E') {
        // #E — encoders only
        float enc[6] = {0};
        active_slave = 1;
        for (int i = 0; i < 4; i++) {
            uint8_t r = v2_send(OP_ENCODER_READ, (uint8_t)i, 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_ENCODER_REPLY) {
                uint16_t angle = (uint16_t)(g_reply.payload & 0xFFFFu);
                float deg = angle * 360.0f / 4096.0f;
                if (i == 0) enc[0] = deg;
                else if (i == 1) enc[1] = deg;
                else if (i == 3) enc[2] = deg;
            }
            sleep_ms(2);
        }
        active_slave = 2;
        for (int i = 0; i < 3; i++) {
            uint8_t r = v2_send(OP_ENCODER_READ, (uint8_t)i, 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_ENCODER_REPLY) {
                uint16_t angle = (uint16_t)(g_reply.payload & 0xFFFFu);
                float deg = angle * 360.0f / 4096.0f;
                enc[3 + i] = deg;
            }
            sleep_ms(2);
        }
        active_slave = 1;
        tcp_printf_capture(">E:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               enc[0], enc[1], enc[2], enc[3], enc[4], enc[5]);
    }
    else if (cmd_str[0] == 'S' && cmd_str[1] != 'H') {
        tcp_printf_capture(">S:IDLE,00\n");
    }
    else if (cmd_str[0] == 'P' && cmd_str[1] == 'I') {
        // #PING — test the SPI link to both slaves (single probe each).
        bool s1 = ping_slave(1);
        bool s2 = ping_slave(2);
        tcp_printf_capture(">PING:S1=%s,S2=%s\n", s1 ? "OK" : "DEAD", s2 ? "OK" : "DEAD");
    }
    else if (cmd_str[0] == 'V') {
        tcp_printf_capture(">V:Nite369-Master-Eth v%s (build %d)\n",
                           FIRMWARE_VERSION_STR, FIRMWARE_BUILD_COUNT);
    }
    else if (cmd_str[0] == 'M' && cmd_str[1] == 'V') {
        int j = strtol(cmd_str + 2, NULL, 10);
        char *p = strchr(cmd_str + 2, ',');
        int speed = p ? strtol(p + 1, NULL, 10) : 1000;
        p = p ? strchr(p + 1, ',') : NULL;
        int steps = p ? strtol(p + 1, NULL, 10) : 0;
        p = p ? strchr(p + 1, ',') : NULL;
        int accel = p ? strtol(p + 1, NULL, 10) : 0;
        p = p ? strchr(p + 1, ',') : NULL;
        int decel = p ? strtol(p + 1, NULL, 10) : 0;
        if (j < 1 || j > 6 || speed < 1) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        // Clamp to the slave's frame field widths. The slave parses steps as
        // int16_t and speed as uint16_t; unclamped values wrap (e.g. steps
        // 70000 becomes a small positive move, steps -40000 becomes positive
        // -> wrong direction). Reject out-of-range rather than silently wrap.
        if (steps > 32767 || steps < -32767) { tcp_printf_capture(">ER:STEPS_RANGE\n"); return tcp_tx_pos; }
        if (speed > 65535) { tcp_printf_capture(">ER:SPEED_RANGE\n"); return tcp_tx_pos; }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        // Stamp a per-command 8-bit sequence (v2 seq byte). The slave dedupes
        // on it; 0 is reserved (legacy frames always execute).
        if (++mv_seq == 0) mv_seq = 1;
        printf("[MV] j=%d axis=%d speed=%d steps=%d accel=%d decel=%d seq=%u\n",
               j, axis, speed, steps, accel, decel, mv_seq);
        uint8_t r = v2_send_retry(OP_STEP_DELTA, (uint8_t)axis,
                                  V2_PAYLOAD_MOVE(speed, steps), mv_seq);
        active_slave = 1;
        tcp_report_slave_result(r);
    }
    else if (cmd_str[0] == 'T' && cmd_str[1] == 'E' && cmd_str[2] == 'S'
             && cmd_str[3] == 'T' && cmd_str[4] == 'M' && cmd_str[5] == 'O'
             && cmd_str[6] == 'V' && cmd_str[7] == 'E') {
        // #TESTMOVE — move each joint a small amount and report which moved.
        // Sends a 200-step #MV to each joint, waits for it to finish, then
        // reads the motion status (#MS) to confirm the position changed.
        // Useful to find a dead/unwired axis without watching each one.
        // Optional argument: step size in steps (default 200).
        int test_steps = 200;
        if (cmd_str[8] == ',') test_steps = atoi(cmd_str + 9);
        if (test_steps < 10) test_steps = 10;
        if (test_steps > 2000) test_steps = 2000;
        tcp_printf_capture(">TEST:");
        for (int j = 1; j <= 6; j++) {
            // Record the pre-move position.
            int axis = (j - 1) % 3;
            active_slave = (j <= 3) ? 1 : 2;
            uint8_t qr = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
            int16_t before = (qr == NITE_RSP_OK) ? V2_STATUS_POS(g_reply.payload) : 0;
            // Send the test move.
            uint16_t spd = 2000;
            if (++mv_seq == 0) mv_seq = 1;
            uint8_t mr = v2_send_retry(OP_STEP_DELTA, (uint8_t)axis,
                                       V2_PAYLOAD_MOVE(spd, test_steps), mv_seq);
            bool moved = (mr == NITE_RSP_OK);
            if (moved) {
                // Poll status until the axis stops moving (or ~1s timeout),
                // then re-read the position.
                for (int w = 0; w < 20; w++) {
                    qr = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
                    if (qr == NITE_RSP_OK && (g_reply.seq & 1) == 0) break;
                    busy_wait_us_32(50000);
                }
                qr = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
                int16_t after = (qr == NITE_RSP_OK) ? V2_STATUS_POS(g_reply.payload) : 0;
                // Position should have changed by ~test_steps.
                int32_t diff = after - before;
                if (diff < 0) diff = -diff;
                moved = (diff >= test_steps / 2);
            }
            tcp_printf_capture("J%d=%s", j, moved ? "OK" : "FAIL");
            if (j < 6) tcp_printf_capture(" ");
        }
        tcp_printf_capture("\n");
        active_slave = 1;
    }
    else if (cmd_str[0] == 'J' && cmd_str[1] == 'C') {
        // #JC<joint>,<dir>,<speed> — CONTINUOUS jog (hold-to-run).
        // dir: +1 = forward, -1 = reverse. Sends the slave's 0x52
        // continuous-motion command. Stop with #H (halt) or #HS.
        int j = strtol(cmd_str + 2, NULL, 10);
        char *p = strchr(cmd_str + 2, ',');
        int dir = p ? strtol(p + 1, NULL, 10) : 1;
        p = p ? strchr(p + 1, ',') : NULL;
        int speed = p ? strtol(p + 1, NULL, 10) : 200;
        if (j < 1 || j > 6 || speed < 1 || speed > 65535) {
            tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos;
        }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        // OP_CONT_JOG: continuous hold-to-run. payload = V2_PAYLOAD_MOVE(speed, dir).
        uint8_t r = v2_send_retry(OP_CONT_JOG, (uint8_t)axis,
                                  V2_PAYLOAD_MOVE(speed, dir > 0 ? 1 : -1), 0);
        active_slave = 1;
        tcp_report_slave_result(r);
    }
    else if (cmd_str[0] == 'M' && cmd_str[1] == 'S') {
        tcp_printf_capture(">MS:");
        for (int j = 1; j <= 6; j++) {
            int axis = (j - 1) % 3;
            active_slave = (j <= 3) ? 1 : 2;
            // Bounded retry (try2) on transient MISO corruption: a single
            // retry catches the clean frame instead of reporting ?,?,?.
            uint8_t r = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
            if (r == NITE_RSP_OK && g_reply.opcode == OP_MOTION_REPLY) {
                int16_t pos = V2_STATUS_POS(g_reply.payload);
                uint16_t spd = V2_STATUS_SPD(g_reply.payload);
                uint8_t mov = (g_reply.seq & 1) ? 1 : 0;
                tcp_printf_capture("%d,%u,%d", pos, spd, mov);
            } else { tcp_printf_capture("?,?,?"); }
            if (j < 6) tcp_printf_capture(";");
        }
        active_slave = 1;
        tcp_printf_capture("\n");
    }
    else if (cmd_str[0] == 'M' && cmd_str[1] == 'A' && cmd_str[2] == 'C') {
        if (cmd_str[3] == 'R') {
            mac_start_record();
            mac_recording = true;
            tcp_printf_capture(">OK:MAC_RECORDING\n");
        } else if (cmd_str[3] == 'S') {
            mac_stop_record();
            mac_recording = false;
            mac_persist();
            tcp_printf_capture(">OK:MAC_STOPPED (%d steps)\n", mac_count());
        } else if (cmd_str[3] == 'P') {
            int steps = 0;
            if (!mac_play(&steps)) {
                tcp_printf_capture(">ER:MAC_EMPTY\n");
                return tcp_tx_pos;
            }
            tcp_printf_capture(">MAC_PLAY:%d\n", steps);
            for (int i = 0; i < steps; i++) {
                const macro_step_t *s = wp_get_macro_step(i);
                if (!s) break;
                if (s->cmd[0] == 'R' && s->cmd[1] == 'T') continue;
                if (s->cmd[0] == 'M' && s->cmd[1] == 'A' && s->cmd[2] == 'C') continue;
                process_nite369(s->cmd);
                tcp_tx_pos = 0;  // reset after inner call to prevent output pollution
                sleep_ms(10);
            }
            tcp_printf_capture(">OK:MAC_DONE\n");
        } else if (cmd_str[3] == 'L') {
            int n = mac_count();
            tcp_printf_capture(">MACL:%d\n", n);
            for (int i = 0; i < n; i++) {
                const macro_step_t *s = wp_get_macro_step(i);
                if (s) tcp_printf_capture("  [%d] %s\n", i, s->cmd);
            }
        } else if (cmd_str[3] == 'D') {
            mac_clear();
            mac_persist();
            tcp_printf_capture(">OK:MAC_CLEARED\n");
        } else {
            tcp_printf_capture(">ER:INVALID_MAC_CMD\n");
        }
    }
    else if (cmd_str[0] == 'M') {
        // #M<j1>,<j2>,<j3>,<j4>,<j5>,<j6> — COORDINATED multi-axis RELATIVE
        // move (each value is a delta in degrees; all axes finish together).
        float joints[6];
        const char *p = cmd_str + 1;
        for (int i = 0; i < 6; i++) {
            joints[i] = strtof(p, (char **)&p);
            if (*p == ',') p++;
        }
        coordinated_move(joints);
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'S' && cmd_str[1] == 'H') {
        // #SH — set software home at the CURRENT commanded position. After
        // this, #GH returns every joint to this exact position.
        for (int i = 0; i < 6; i++) home_pos[i] = cmd_pos[i];
        home_set = true;
        printf("[HOME] set at %.1f %.1f %.1f %.1f %.1f %.1f\n",
               cmd_pos[0], cmd_pos[1], cmd_pos[2], cmd_pos[3], cmd_pos[4], cmd_pos[5]);
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'G' && cmd_str[1] == 'H') {
        // #GH — go home: coordinated move back to the stored software home
        // (undoes the movements since #SH).
        if (!home_set) { tcp_printf_capture(">ER:NO_HOME\n"); return tcp_tx_pos; }
        float delta[6];
        for (int i = 0; i < 6; i++) delta[i] = home_pos[i] - cmd_pos[i];
        coordinated_move(delta);
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'G') {
        // #G<steps> — gripper servo (retry so corrupted frames are re-sent)
        // 0..5000 steps -> 0..1800 deci-degrees -> 0x58 servo frame.
        int steps = atoi(cmd_str + 1);
        if (steps < 0) steps = 0;
        if (steps > 5000) steps = 5000;
        int angle_10 = steps * 1800 / 5000;
        active_slave = 2;
        v2_send_retry(OP_GRIPPER, JOINT_NONE, angle_10, 0);
        active_slave = 1;
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'T' && cmd_str[1] == 'R') {
        // #TR<addr>,<reg>
        int addr = strtol(cmd_str + 2, NULL, 10);
        if (addr < 0) { tcp_printf_capture(">ER:INVALID_ADDR\n"); return tcp_tx_pos; }
        char *comma = strchr(cmd_str + 2, ',');
        int reg = comma ? strtol(comma + 1, NULL, 16) : 0;
        active_slave = (addr < 4) ? 2 : 1;
        int tmc_addr = addr % 4;
        v2_send_slow(OP_TMC_READ, (uint8_t)tmc_addr, reg, 0);
        if (g_reply_status == NITE_RSP_OK && g_reply.opcode == OP_TMC_REPLY) {
            uint32_t val = (uint32_t)g_reply.payload;
            tcp_printf_capture(">TR:%d,%02X,%08X\n", addr, reg, val);
        } else {
            tcp_printf_capture(">ER:TMC_READ_FAIL\n");
        }
        active_slave = 1;
    }
    else if (cmd_str[0] == 'T' && cmd_str[1] == 'W') {
        // #TW<addr>,<reg>,<hex>
        int addr = strtol(cmd_str + 2, NULL, 10);
        if (addr < 0) { tcp_printf_capture(">ER:INVALID_ADDR\n"); return tcp_tx_pos; }
        char *p = strchr(cmd_str + 2, ',');
        int reg = p ? strtol(p + 1, NULL, 16) : 0;
        p = strchr(p ? p + 1 : cmd_str + 2, ',');
        uint32_t val = p ? strtoul(p + 1, NULL, 16) : 0;
        active_slave = (addr < 4) ? 2 : 1;
        int tmc_addr = addr % 4;
        // OP_TMC_WRITE: seq byte carries the register (see frame.h), payload
        // carries the value.
        v2_send_slow(OP_TMC_WRITE, (uint8_t)tmc_addr, (int32_t)val, (uint8_t)reg);
        active_slave = 1;
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'T' && cmd_str[1] >= '0' && cmd_str[1] <= '9') {
        // #T<addr> — DRV_STATUS
        int addr = strtol(cmd_str + 1, NULL, 10);
        active_slave = (addr < 4) ? 2 : 1;
        int tmc_addr = addr % 4;
        v2_send_slow(OP_TMC_READ, (uint8_t)tmc_addr, 0x6F, 0);
        if (g_reply_status == NITE_RSP_OK && g_reply.opcode == OP_TMC_REPLY) {
            uint32_t val = (uint32_t)g_reply.payload;
            tcp_printf_capture(">T:%d,%08X\n", addr, val);
        } else {
            tcp_printf_capture(">ER:TMC_READ_FAIL\n");
        }
        active_slave = 1;
    }
    else if (cmd_str[0] == 'L' && cmd_str[1] == 'E' && cmd_str[2] == 'D') {
        // #LED<r>,<g>,<b>[,<mode>]
        uint8_t r = (uint8_t)strtoul(cmd_str + 3, NULL, 10);
        char *p = strchr(cmd_str + 3, ',');
        uint8_t g = p ? (uint8_t)strtoul(p + 1, NULL, 10) : 0;
        p = p ? strchr(p + 1, ',') : NULL;
        uint8_t b = p ? (uint8_t)strtoul(p + 1, NULL, 10) : 0;
        p = p ? strchr(p + 1, ',') : NULL;
        uint8_t mode = p ? (uint8_t)strtoul(p + 1, NULL, 10) : 0;
        active_slave = 2;
        v2_send(OP_LED, JOINT_NONE, V2_PAYLOAD_LED(r, g, b, mode), 0);
        active_slave = 1;
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'L') {
        // #L — read limit switches
        active_slave = 1;
        uint8_t r1 = v2_send(OP_LIMIT_READ, JOINT_NONE, 0, 0);
        uint8_t lim1 = (r1 == NITE_RSP_OK && g_reply.opcode == OP_LIMIT_REPLY) ? (uint8_t)g_reply.payload : 0;
        active_slave = 2;
        uint8_t r2 = v2_send(OP_LIMIT_READ, JOINT_NONE, 0, 0);
        uint8_t lim2 = (r2 == NITE_RSP_OK && g_reply.opcode == OP_LIMIT_REPLY) ? (uint8_t)g_reply.payload : 0;
        active_slave = 1;
        uint8_t combined = (lim1 & 0x07) | ((lim2 & 0x07) << 3);
        tcp_printf_capture(">L:%d\n", combined);
    }
    else if (cmd_str[0] == 'H' && cmd_str[1] == 'M') {
        int j = strtol(cmd_str + 2, NULL, 10);
        if (j == 0) {
            active_slave = 1; v2_send(OP_HOME, JOINT_NONE, 0, 0);
            active_slave = 2; v2_send(OP_HOME, JOINT_NONE, 0, 0);
            active_slave = 1;
        } else if (j >= 1 && j <= 6) {
            int axis = (j - 1) % 3;
            active_slave = (j <= 3) ? 1 : 2;
            v2_send(OP_HOME, (uint8_t)axis, 0, 0); active_slave = 1;
        } else { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'H' && cmd_str[1] == 'Q') {
        int j = strtol(cmd_str + 2, NULL, 10);
        if (j < 1 || j > 6) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        uint8_t r = v2_send(OP_HOME, (uint8_t)axis, 2, 0);  // payload 2 = query
        uint8_t h1 = 0, h2 = 0;
        if (r == NITE_RSP_OK && g_reply.opcode == OP_HOMING_REPLY) {
            h1 = (uint8_t)(g_reply.payload & 0xFF);          // homed
            h2 = (uint8_t)((g_reply.payload >> 8) & 0xFF);   // state
        }
        tcp_printf_capture(">HQ:%d,%d,%d\n", j, h1, h2);
        active_slave = 1;
    }
    else if (cmd_str[0] == 'H' && cmd_str[1] == 'C') {
        // #HC<j>,<search>,<creep>,<backoff> — homing config write.
        // v2: three OP_CFG_WRITE calls, one per home field (8/9/10).
        int j = strtol(cmd_str + 2, NULL, 10);
        if (j < 1 || j > 6) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        const char *rest = cmd_str + 2;
        while (*rest && *rest != ',') rest++;
        if (*rest != ',') { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        rest++;
        int search = strtol(rest, NULL, 10);
        while (*rest && *rest != ',') rest++;
        if (*rest != ',') { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        rest++;
        int creep = strtol(rest, NULL, 10);
        while (*rest && *rest != ',') rest++;
        if (*rest != ',') { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        rest++;
        int backoff = strtol(rest, NULL, 10);
        if (search < 1 || creep < 1 || backoff < 0) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        bool ok = true;
        if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_HOME_SEARCH, axis),
                          (int32_t)search, 0) != NITE_RSP_OK) ok = false;
        if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_HOME_CREEP, axis),
                          (int32_t)creep, 0) != NITE_RSP_OK) ok = false;
        if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_HOME_BACKOFF, axis),
                          (int32_t)backoff, 0) != NITE_RSP_OK) ok = false;
        active_slave = 1;
        tcp_printf_capture(ok ? ">OK\n" : ">ER:FAIL\n");
    }
    else if (cmd_str[0] == 'H' && cmd_str[1] == 'G') {
        // #HG<j> — homing config read. v2: read max_speed + home fields.
        int j = strtol(cmd_str + 2, NULL, 10);
        if (j < 1 || j > 6) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        // v1 reported max_speed as search + accel/5 + decel; v2 reads the
        // real home fields when available, falling back to the same.
        // The studio parses >HG:<j>,<search>,<creep>,<backoff>,<off>,<inv_lim>,<inv_dir>.
        uint32_t search = 1000, creep = 200, backoff = 0;
        uint8_t r;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_HOME_SEARCH, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) search = (uint32_t)g_reply.payload;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_HOME_CREEP, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) creep = (uint32_t)g_reply.payload;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_HOME_BACKOFF, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) backoff = (uint32_t)g_reply.payload;
        active_slave = 1;
        tcp_printf_capture(">HG:%d,%u,%u,%u,0,0,0\n", j, search, creep, backoff);
    }
    else if (cmd_str[0] == 'H') {
        // #H — halt all on both slaves. CRITICAL safety command.
        // The halt is idempotent (halting a stopped axis is a no-op), so a
        // corrupted ack after retries is treated as success as long as a fresh
        // single attempt also reaches the slave. Only report FAIL if the link
        // is genuinely down. Report each slave separately.
        uint8_t r1 = 0, r2 = 0;
        active_slave = 1;
        r1 = v2_send_retry(OP_HALT, JOINT_NONE, 0, 0);
        if (r1 != NITE_RSP_OK) r1 = v2_send(OP_HALT, JOINT_NONE, 0, 0);  // idempotent — retry fresh once
        sleep_ms(5);
        active_slave = 2;
        r2 = v2_send_retry(OP_HALT, JOINT_NONE, 0, 0);
        if (r2 != NITE_RSP_OK) r2 = v2_send(OP_HALT, JOINT_NONE, 0, 0);
        sleep_ms(5);
        active_slave = 1;
        if (r1 == NITE_RSP_OK && r2 == NITE_RSP_OK) {
            tcp_printf_capture(">OK\n");
        } else {
            tcp_printf_capture(">ER:HALT S1=%s S2=%s\n",
                   (r1 == NITE_RSP_OK) ? "OK" : "FAIL",
                   (r2 == NITE_RSP_OK) ? "OK" : "FAIL");
        }
    }
    else if (cmd_str[0] == 'C' && cmd_str[1] == 'F' && cmd_str[2] != 'G') {
        int j = strtol(cmd_str + 2, NULL, 10);
        char *p = strchr(cmd_str + 2, ',');
        int speed = p ? strtol(p + 1, NULL, 10) : 2000;
        if (speed < 1) speed = 1;
        if (speed > (int)CONFIG_MAX_SPEED_LIMIT) speed = (int)CONFIG_MAX_SPEED_LIMIT;
        p = p ? strchr(p + 1, ',') : NULL;
        int accel = p ? strtol(p + 1, NULL, 10) : 500;
        p = p ? strchr(p + 1, ',') : NULL;
        int decel = p ? strtol(p + 1, NULL, 10) : accel;
        // Optional 5th value: rapid jog-stop decel (#CF<j>,<max>,<a>,<d>,<jd>)
        p = p ? strchr(p + 1, ',') : NULL;
        int jog_decel = p ? strtol(p + 1, NULL, 10) : -1;
        // Optional 6th value: continuous-jog ramp-up accel (#CF<j>,<max>,<a>,<d>,<jd>,<ja>)
        p = p ? strchr(p + 1, ',') : NULL;
        int jog_accel = p ? strtol(p + 1, NULL, 10) : -1;
        if (j < 1 || j > 6) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        // v2: OP_CFG_WRITE per profile field (5=max_speed, 6=accel, 7=decel).
        // Use the RETRY version and check the result: the SPI link to the
        // slaves is bursty (~1 in 10 frames corrupts), and a silent drop here
        // makes the studio think the max_speed was saved when it wasn't
        // (robot keeps the old value after a >OK reply).
        bool ok = true;
        if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_MAX_SPEED, axis),
                          (int32_t)speed, 0) != NITE_RSP_OK) ok = false;
        if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_ACCEL, axis),
                          (int32_t)accel, 0) != NITE_RSP_OK) ok = false;
        if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_DECEL, axis),
                          (int32_t)decel, 0) != NITE_RSP_OK) ok = false;
        // Optional jog_decel -> field 3 so continuous-jog stops use the
        // rapid decel. Persisted on the slave config.
        if (jog_decel > 0) {
            if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_JOG_DECEL, axis),
                              (int32_t)jog_decel, 0) != NITE_RSP_OK) ok = false;
        }
        // Optional jog_accel -> field 4 so continuous-jog ramp-up uses the
        // jog-specific accel. Persisted on the slave config.
        if (jog_accel > 0) {
            if (v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_JOG_ACCEL, axis),
                              (int32_t)jog_accel, 0) != NITE_RSP_OK) ok = false;
        }
        active_slave = 1;
        // Mirror into the master's local config using the MASTER joint index
        // (j-1), not the slave-local axis index (0-2).
        config_t *mc = config_get_mut();
        int g = j - 1;
        if (g >= 0 && g < 8) {
            mc->axes[g].max_speed = (uint32_t)speed;
            mc->axes[g].accel = (uint32_t)accel;
            mc->axes[g].decel = (uint32_t)decel;
            if (jog_decel > 0) mc->axes[g].jog_decel = (uint32_t)jog_decel;
            if (jog_accel > 0) mc->axes[g].jog_accel = (uint32_t)jog_accel;
        }
        tcp_printf_capture(ok ? ">OK\n" : ">ER:CONFIG_FAIL\n");
    }
    else if (cmd_str[0] == 'C' && cmd_str[1] == 'R') {
        int j = strtol(cmd_str + 2, NULL, 10);
        if (j < 1 || j > 6) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        int axis = (j - 1) % 3;
        active_slave = (j <= 3) ? 1 : 2;
        // v2: OP_CFG_READ per field (5=max_speed, 6=accel, 7=decel,
        // 3=jog_decel, 4=jog_accel). Fast bounded variant (try2): #CR is a
        // status/display read, and a rare miss just reports 0 (the studio
        // re-reads). Using the retry version (30x) made #CR take 1-2s.
        uint32_t spd = 0, acc = 0, dec = 0, jdec = 0, jacc = 0;
        uint8_t r;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_MAX_SPEED, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) spd = (uint32_t)g_reply.payload;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_ACCEL, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) acc = (uint32_t)g_reply.payload;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_DECEL, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) dec = (uint32_t)g_reply.payload;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_JOG_DECEL, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) jdec = (uint32_t)g_reply.payload;
        r = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_JOG_ACCEL, axis), 0, 0);
        if (r == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) jacc = (uint32_t)g_reply.payload;
        tcp_printf_capture(">CR:%d,%u,%u,%u,%lu,%lu\n", j, spd, acc, dec,
                           (unsigned long)jdec, (unsigned long)jacc);
        active_slave = 1;
    }
    else if (cmd_str[0] == 'C' && cmd_str[1] == 'S') {
        active_slave = 1; v2_send_flash(OP_CFG_SAVE, JOINT_NONE, 0, 0);
        active_slave = 2; v2_send_flash(OP_CFG_SAVE, JOINT_NONE, 0, 0);
        active_slave = 1;
        tcp_printf_capture(">OK\n");
    }
    else if (cmd_str[0] == 'C' && cmd_str[1] == 'F' && cmd_str[2] == 'G'
             && cmd_str[3] == 'R' && cmd_str[4] == 'E'
             && cmd_str[5] == 'S' && cmd_str[6] == 'E'
             && cmd_str[7] == 'T') {
        // #CFGRESET — reset config to firmware defaults on the master AND
        // both slaves (OP_CFG_RESET on each), then save the master's copy.
        config_reset();
        config_save();
        active_slave = 1; v2_send_flash(OP_CFG_RESET, JOINT_NONE, 0, 0);
        active_slave = 2; v2_send_flash(OP_CFG_RESET, JOINT_NONE, 0, 0);
        active_slave = 1;
        tcp_printf_capture(">OK:RESET\n");
    }
    else if (cmd_str[0] == 'C' && cmd_str[1] == 'F' && cmd_str[2] == 'G') {
        // #CFG<j> — read extended config (steps_per_rev, gear_ratio, dir_inverted)
        // #CFG<j>,<spr>,<gr>,<di> — write extended config
        int j = strtol(cmd_str + 3, NULL, 10);
        if (j < 0 || j > 6) { tcp_printf_capture(">ER:INVALID\n"); return tcp_tx_pos; }
        char *p = strchr(cmd_str + 3, ',');
        if (p) {
            // Write mode. j == 0 applies the values to ALL SIX joints
            // (a single command for the whole arm); otherwise just joint j.
            uint32_t spr = strtoul(p + 1, NULL, 10);
            p = strchr(p + 1, ',');
            uint32_t gr = p ? strtoul(p + 1, NULL, 10) : 100;
            p = p ? strchr(p + 1, ',') : NULL;
            uint32_t di = p ? strtoul(p + 1, NULL, 10) : 0;
            config_t *mc = config_get_mut();
            int list[6]; int n = 0;
            if (j == 0) { for (int k = 0; k < 6; k++) list[n++] = k + 1; }
            else list[n++] = j;
            for (int idx = 0; idx < n; idx++) {
                int jj = list[idx];
                int axis = (jj - 1) % 3;
                active_slave = (jj <= 3) ? 1 : 2;
                // v2: OP_CFG_WRITE with field-encoded joint_id (0=spr, 1=gr, 2=di).
                v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_STEPS_REV, axis),
                              (int32_t)spr, 0); sleep_ms(2);
                v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_GEAR_RATIO, axis),
                              (int32_t)gr, 0); sleep_ms(2);
                v2_send_retry(OP_CFG_WRITE, V2_JID_CFG(CFG_FIELD_DIR_INVERT, axis),
                              (int32_t)(di ? 1 : 0), 0);
                // Mirror into the master's LOCAL config using the MASTER joint
                // index (jj-1 = 0..5), not the slave-local axis index.
                int g = jj - 1;
                if (g >= 0 && g < 8) {
                    mc->axes[g].steps_per_rev = spr;
                    mc->axes[g].gear_ratio = gr;
                    mc->axes[g].dir_inverted = (di != 0);
                }
            }
            active_slave = 1;
            tcp_printf_capture(">OK\n");
        } else {
            // Read mode — read all 3 fields. Use the FAST bounded read so the
            // studio's per-joint reads (and the old cfg_write pre-read) don't
            // take ~1s each; a rare miss reports 0.
            int axis = (j - 1) % 3;
            active_slave = (j <= 3) ? 1 : 2;
            uint32_t spr = 0, gr = 0, di = 0;
            uint8_t r0 = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_STEPS_REV, axis), 0, 0);
            if (r0 == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) spr = (uint32_t)g_reply.payload;
            uint8_t r1 = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_GEAR_RATIO, axis), 0, 0);
            if (r1 == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) gr = (uint32_t)g_reply.payload;
            uint8_t r2 = v2_send_try2(OP_CFG_READ, V2_JID_CFG(CFG_FIELD_DIR_INVERT, axis), 0, 0);
            if (r2 == NITE_RSP_OK && g_reply.opcode == OP_CFG_REPLY) di = (uint32_t)g_reply.payload;
            active_slave = 1;
            tcp_printf_capture(">CFG:%d,%u,%u,%u\n", j, spr, gr, di);
        }
    }
    // ── Real-time position streaming ─────────────────────────────────
    else if (cmd_str[0] == 'R' && cmd_str[1] == 'T') {
        // #RT<Hz> — auto-stream position at N Hz (0 = off)
        int hz = strtol(cmd_str + 2, NULL, 10);
        if (hz < 0) hz = 0;
        if (hz > 200) hz = 200;  // cap at 200 Hz
        rt_stream_hz = (uint32_t)hz;
        rt_stream_last_ms = to_ms_since_boot(get_absolute_time());
        tcp_printf_capture(">RT:%u\n", rt_stream_hz);
    }
    // ── Waypoints ────────────────────────────────────────────────────
    else if (cmd_str[0] == 'W' && cmd_str[1] == 'P' && cmd_str[2] == 'S') {
        // #WPS<name> — save current position as named waypoint
        const char *name = cmd_str + 3;
        while (*name == ' ' || *name == ',') name++;
        if (*name == '\0') { tcp_printf_capture(">ER:NO_NAME\n"); return tcp_tx_pos; }
        wp_save(name, cmd_pos);
        wp_persist();
        tcp_printf_capture(">OK:WP_SAVED\n");
    }
    else if (cmd_str[0] == 'W' && cmd_str[1] == 'P' && cmd_str[2] == 'M') {
        // #WPM<name> — move to named waypoint
        const char *name = cmd_str + 3;
        while (*name == ' ' || *name == ',') name++;
        if (*name == '\0') { tcp_printf_capture(">ER:NO_NAME\n"); return tcp_tx_pos; }
        int idx = wp_find(name);
        if (idx < 0) { tcp_printf_capture(">ER:WP_NOT_FOUND\n"); return tcp_tx_pos; }
        float target[6];
        wp_load(idx, target);
        float delta[6];
        for (int i = 0; i < 6; i++) delta[i] = target[i] - cmd_pos[i];
        coordinated_move(delta);
        tcp_printf_capture(">OK:WP_MOVED\n");
    }
    else if (cmd_str[0] == 'W' && cmd_str[1] == 'P' && cmd_str[2] == 'L') {
        // #WPL — list all waypoints
        int n = wp_count();
        tcp_printf_capture(">WPL:%d\n", n);
        for (int i = 0; i < n; i++) {
            const waypoint_t *w = wp_get(i);
            if (w) {
                tcp_printf_capture("  %s: %.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                    w->name, w->pos[0], w->pos[1], w->pos[2],
                    w->pos[3], w->pos[4], w->pos[5]);
            }
        }
    }
    else if (cmd_str[0] == 'W' && cmd_str[1] == 'P' && cmd_str[2] == 'D') {
        // #WPD<name> — delete waypoint
        const char *name = cmd_str + 3;
        while (*name == ' ' || *name == ',') name++;
        if (*name == '\0') { tcp_printf_capture(">ER:NO_NAME\n"); return tcp_tx_pos; }
        if (wp_delete(name)) {
            wp_persist();
            tcp_printf_capture(">OK:WP_DELETED\n");
        } else {
            tcp_printf_capture(">ER:WP_NOT_FOUND\n");
        }
    }
    // ── Trajectory buffer ─────────────────────────────────────────────
    else if (cmd_str[0] == 'Q' && cmd_str[1] == 'A') {
        // #QA — queue G-code line (abs target, all 6 axes)
        // #QA X10 Y20 Z0 A0 B0 C0 F500
        if (traj_count >= TRAJ_BUF_SIZE) {
            tcp_printf_capture(">ER:BUF_FULL\n");
            return tcp_tx_pos;
        }
        // Parse axis targets from the G-code-style parameters.
        // Must call gcode_parse() first — the tokenizer was not run yet
        // (cmd starts with 'Q', not 'G'/'M').
        char qa_buf[GCODE_MAX_LINE];
        snprintf(qa_buf, sizeof(qa_buf), "G1 %s", cmd_str + 2);
        gcode_parse(qa_buf);
        float target[6];
        memcpy(target, cmd_pos, sizeof(float) * 6);  // default: current pos
        for (int i = 0; i < 6; i++) {
            if (gcode_seen(GCODE_AXIS_LETTERS[i])) {
                target[i] = gcode_floatval(GCODE_AXIS_LETTERS[i], cmd_pos[i]);
            }
        }
        float feed = gcode_floatval('F', 500.0f);
        int slot = (traj_head + traj_count) % TRAJ_BUF_SIZE;
        memcpy(traj_buf[slot].target, target, sizeof(float) * 6);
        traj_buf[slot].feed = feed;
        traj_buf[slot].valid = true;
        traj_count++;
        tcp_printf_capture(">QA:%d/%d\n", traj_count, TRAJ_BUF_SIZE);
    }
    else if (cmd_str[0] == 'Q' && cmd_str[1] == 'E') {
        // #QE — execute queued trajectory (begin execution)
        if (traj_count == 0) {
            tcp_printf_capture(">ER:BUF_EMPTY\n");
            return tcp_tx_pos;
        }
        traj_executing = true;
        tcp_printf_capture(">QE:STARTING %d moves\n", traj_count);
        // Execution happens in the main loop (non-blocking)
    }
    else if (cmd_str[0] == 'Q' && cmd_str[1] == 'S') {
        // #QS — query trajectory buffer status
        tcp_printf_capture(">QS:count=%d exec=%d head=%d tail=%d\n",
            traj_count, traj_executing ? 1 : 0, traj_head, traj_tail);
    }
    else if (cmd_str[0] == 'Q' && cmd_str[1] == 'C') {
        // #QC — clear trajectory buffer
        traj_head = traj_tail = traj_count = 0;
        traj_executing = false;
        memset(traj_buf, 0, sizeof(traj_buf));
        tcp_printf_capture(">OK:BUF_CLEARED\n");
    }
    else if (cmd_str[0] == 'Q' && cmd_str[1] == 'H') {
        // #QH — halt trajectory execution (does NOT clear buffer)
        traj_executing = false;
        tcp_printf_capture(">OK:HALTED\n");
    }
    else {
        tcp_printf_capture(">ER:UNKNOWN_CMD\n");
    }

    return tcp_tx_pos;
}

// --- Main ---
int main() {
    stdio_init_all();
    sleep_ms(2000);

    // Load axis config (steps_per_rev, gear ratios, max speed/accel) so the
    // G-code interpreter's config_steps_per_deg() returns real values.
    config_init();
    workspace_init();

    printf("\n=== Nite369 Master Ethernet ===\n");

    // Power-indicator heartbeat (1Hz blink on the onboard LED)
    led_heartbeat_init();

    // Init slave SPI (SPI1)
    spi_init(SPI_PORT, SPI_SPEED);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS_S1);
    gpio_set_dir(PIN_CS_S1, GPIO_OUT);
    gpio_put(PIN_CS_S1, 1);
    // Soften the CS edges: the boundary-byte corruption (byte 0 / byte 8
    // sampled wrong) is CS-line ringing on each toggle. Slow slew + 2mA
    // drive damps the edge; a real 100-220 ohm series resistor on the CS
    // wire is the physical fix this approximates.
    gpio_set_slew_rate(PIN_CS_S1, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(PIN_CS_S1, GPIO_DRIVE_STRENGTH_2MA);
    gpio_init(PIN_CS_S2);
    gpio_set_dir(PIN_CS_S2, GPIO_OUT);
    gpio_put(PIN_CS_S2, 1);
    gpio_set_slew_rate(PIN_CS_S2, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(PIN_CS_S2, GPIO_DRIVE_STRENGTH_2MA);

    // Init W5500 — NON-FATAL. The robot must stay drivable over USB serial
    // even if the Ethernet chip is absent or the link is down, so a W5500
    // failure no longer halts the master (USB serial is serviced in the main
    // loop regardless).
    bool w5500_ok = w5500_tcp_init();
    if (!w5500_ok) {
        printf("W5500 init failed — USB serial only!\n");
    }

    // One-time config sync: read both slaves' real motion config into the
    // local mirror (slave 2 forces wrist values at boot). After this, #CF/#CFG
    // writes keep the mirror current, so #M's coordinated math needs no
    // per-command SPI reads. Runs regardless of the Ethernet link — the SPI
    // bus works as long as the slaves are powered.
    refresh_slave_configs();
    printf("[CFG] Slave configs synced\n");

    // Main loop
    char rx_buf[512];
    char cmd_buf[256];
    // Per-client persistent line buffers: each TCP connection accumulates
    // its own command stream. W5500 sockets 1-7 map to client indices 0-6.
    // Socket 0 is the listener, USB serial uses a separate buffer.
    typedef struct {
        char     buf[512];
        uint16_t len;
    } client_line_t;
    static client_line_t client_lines[8];  // one per socket (0-7)
    int tcp_reply_sock = -1;    // which socket to reply to (-1 = USB)
    uint32_t last_link_check = 0;
    uint32_t last_heartbeat_ms = 0;
    bool tcp_started = false;   // TCP server brought up once the link appears
    bool slave1_alive = false;
    bool slave2_alive = false;
    uint8_t s1_fail_count = 0;   // consecutive ping failures (heartbeat hysteresis)
    uint8_t s2_fail_count = 0;

    while (true) {
        // ── USB-serial command input FIRST: always serviced, even when the
        //    Ethernet link is down (no cable / switch powered off). The robot
        //    can always be driven over the COM port. A line from the host is
        //    run through the same command processor as TCP.
        static char ser_buf[512];
        static uint16_t ser_len = 0;
        int ch;
        while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (ch == '\n' || ch == '\r') {
                if (ser_len > 0) {
                    ser_buf[ser_len] = '\0';
                    // Skip a leading '#' (the TCP path strips it before
                    // process_nite369, and handlers expect the bare command).
                    char *cmd = ser_buf;
                    while (*cmd == '#' || *cmd == ' ') cmd++;
                    printf("<< %s\n", ser_buf);
                    int tx_len = process_nite369(cmd);
                    if (tx_len > 0) {
                        printf(">> %.*s", tx_len, tcp_tx_buf);
                    }
                    ser_len = 0;
                }
            } else if (ser_len < sizeof(ser_buf) - 1) {
                ser_buf[ser_len++] = (char)ch;
            }
        }

        // ── Ethernet TCP: multi-client — poll ALL sockets ──
        if (w5500_ok && w5500_tcp_is_link_up()) {
            if (!tcp_started) {
                printf("\nLink up!\n");
                w5500_tcp_listen();
                printf("[W5500] Multi-client TCP server on port 23\n");
                tcp_started = true;
            }
            // Poll every socket — w5500_tcp_poll fills buf with data from
            // the first socket that has RX data, and sets tcp_reply_sock.
            for (int poll_i = 0; poll_i < 8; poll_i++) {
                if (!w5500_tcp_poll(rx_buf, sizeof(rx_buf), &tcp_reply_sock)) {
                    break;  // no more data on any socket this iteration
                }
                int ci = tcp_reply_sock;  // socket number IS the index
                if (ci < 0 || ci >= 8) continue;

                printf("[TCP%d] << %s\n", tcp_reply_sock, rx_buf);

                // Disconnected mid-command? clear that client's buffer.
                if (!w5500_tcp_is_client_connected(tcp_reply_sock)) {
                    printf("[TCP%d] disconnected\n", tcp_reply_sock);
                    client_lines[ci].len = 0;
                    continue;
                }

                // Append to THIS client's line buffer.
                client_line_t *cl = &client_lines[ci];
                for (char *ch = rx_buf; *ch && cl->len < sizeof(cl->buf) - 1; ch++) {
                    cl->buf[cl->len++] = *ch;
                }

                // Process complete commands: '#' starts a Nite command,
                // or a bare line ending with '\n' is treated as G-code.
                char *p = cl->buf;
                while (cl->len > 0) {
                    // Look for '#' command or bare newline-terminated line.
                    char *hash = memchr(p, '#', (size_t)(cl->len - (p - cl->buf)));
                    char *nl = memchr(p, '\n', (size_t)(cl->len - (p - cl->buf)));
                    char *cr = memchr(p, '\r', (size_t)(cl->len - (p - cl->buf)));
                    char *eol = nl ? nl : cr;

                    if (!hash && !eol) break;  // no complete command yet

                    char *cmd_start;
                    char *cmd_end;
                    if (hash && (!eol || hash < eol)) {
                        // '#' command: starts at hash+1, ends at next '#'/EOL
                        cmd_start = hash + 1;
                        cmd_end = cmd_start;
                        size_t remaining = (size_t)(cl->len - (cmd_start - cl->buf));
                        while (remaining > 0 && *cmd_end != '#' && *cmd_end != '\n' && *cmd_end != '\r') {
                            cmd_end++;
                            remaining--;
                        }
                        if (remaining == 0) break;  // incomplete — hold
                    } else {
                        // Bare G-code line (no '#'): from p to EOL.
                        cmd_start = p;
                        cmd_end = eol;
                    }

                    int len = (int)(cmd_end - cmd_start);
                    if (len > 0 && len < (int)sizeof(cmd_buf) - 1) {
                        memcpy(cmd_buf, cmd_start, len);
                        cmd_buf[len] = '\0';
                        int tx_len = process_nite369(cmd_buf);
                        if (tx_len > 0) {
                            w5500_tcp_send_to(tcp_reply_sock, tcp_tx_buf, tx_len);
                            printf("[TCP%d] >> %.*s", tcp_reply_sock, tx_len, tcp_tx_buf);
                        }
                    }
                    // Advance past this command + delimiter.
                    p = cmd_end;
                    if (p < cl->buf + cl->len && (*p == '#' || *p == '\n' || *p == '\r')) p++;
                }
                // Shift unconsumed tail to front.
                if (p > cl->buf) {
                    size_t tail = (size_t)(cl->len - (p - cl->buf));
                    if (tail > 0) memmove(cl->buf, p, tail);
                    cl->len = (uint16_t)tail;
                }
            }
        } else {
            // Link down: nothing to serve over TCP. Reset the one-time flag
            // so a later link-up re-opens the socket cleanly. USB serial (the
            // block above) keeps working the whole time.
            tcp_started = false;
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (w5500_ok && now - last_link_check > 5000) {
            last_link_check = now;
            printf("[W5500] Sockets: ");
            for (int si = 0; si < 8; si++) printf("%d=0x%02X ", si, getSn_SR(si));
            printf("PHY=%d\n", w5500_tcp_is_link_up());
        }

        // Slave heartbeat: ping both slaves every ~1s (single probe each) so a
        // dead/disconnected slave is detected even with no commands in flight.
        // Prints a status line only when a slave's state changes.
        //
        // Hysteresis: only flip to DEAD after 3 CONSECUTIVE failed pings, and
        // flip back to ALIVE on the first good ping. A single failed probe is
        // NOT proof of death — the bit-bang slave's reply can shift/corrupt
        // while a jog is stepping (rate-alarm IRQs delay the SPI loop), and a
        // healthy slave must not flap ALIVE/DEAD every second.
        if (now - last_heartbeat_ms > 1000) {
            last_heartbeat_ms = now;
            bool s1_ok = ping_slave(1);
            bool s2_ok = ping_slave(2);

            if (s1_ok) s1_fail_count = 0;
            else if (s1_fail_count < 255) s1_fail_count++;
            if (s2_ok) s2_fail_count = 0;
            else if (s2_fail_count < 255) s2_fail_count++;

            bool s1_alive = (s1_fail_count < 3);
            bool s2_alive = (s2_fail_count < 3);

            if (s1_alive != slave1_alive || s2_alive != slave2_alive) {
                slave1_alive = s1_alive;
                slave2_alive = s2_alive;
                printf("[HB] Slave1=%s Slave2=%s (s1fail=%u s2fail=%u)\n",
                       s1_alive ? "ALIVE" : "DEAD", s2_alive ? "ALIVE" : "DEAD",
                       (unsigned)s1_fail_count, (unsigned)s2_fail_count);
            }
        }

        // ── Real-time position streaming ───────────────────────────────
        if (rt_stream_hz > 0) {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            uint32_t interval_ms = 1000 / rt_stream_hz;
            if (now_ms - rt_stream_last_ms >= interval_ms) {
                rt_stream_last_ms = now_ms;
                // Read motion counters from both slaves
                float pos[6] = {0};
                for (int j = 1; j <= 6; j++) {
                    int axis = (j - 1) % 3;
                    active_slave = (j <= 3) ? 1 : 2;
                    uint8_t r = v2_send_try2(OP_MOTION_STATUS, (uint8_t)axis, 0, 0);
                    if (r == NITE_RSP_OK && g_reply.opcode == OP_MOTION_REPLY) {
                        pos[j - 1] = (float)V2_STATUS_POS(g_reply.payload);
                    }
                }
                active_slave = 1;
                // Send as a status line (studio can parse >RT:...)
                printf(">RT:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                    pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
                // Also send to active TCP clients
                if (tcp_reply_sock >= 0) {
                    char rt_buf[128];
                    int n = snprintf(rt_buf, sizeof(rt_buf),
                        ">RT:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                        pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
                    w5500_tcp_send_to(tcp_reply_sock, rt_buf, n);
                }
            }
        }

        // ── Trajectory buffer execution ────────────────────────────────
        if (traj_executing && traj_count > 0) {
            // Get the next move from the buffer
            float target[6];
            float feed;
            memcpy(target, traj_buf[traj_tail].target, sizeof(float) * 6);
            feed = traj_buf[traj_tail].feed;
            traj_buf[traj_tail].valid = false;
            traj_tail = (traj_tail + 1) % TRAJ_BUF_SIZE;
            traj_count--;

            // Compute delta from current commanded position
            float delta[6];
            for (int i = 0; i < 6; i++) delta[i] = target[i] - cmd_pos[i];

            // Execute the move (this blocks until complete)
            coordinated_move(delta);

            // If buffer is empty, stop executing
            if (traj_count == 0) {
                traj_executing = false;
                printf("[TRAJ] Buffer empty — execution complete\n");
            }
        }

        // ── Macro recording hook ───────────────────────────────────────
        // (This is a no-op in the main loop; recording happens in process_nite369)
        // The macro recording flag is checked by the command processor.

        sleep_ms(1);
    }
}
