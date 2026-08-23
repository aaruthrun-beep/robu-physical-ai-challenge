/**
 * @file nite_servo.c
 * @brief Nite 369 Servo Pico — LADRC Control Firmware (FIXED)
 *
 * FIXES over the original:
 *   1. CRITICAL: 10kHz loop reduced to 1kHz — software I2C bit-bang takes
 *      ~80us per encoder read, so 4 encoders × 80us = 320us >> 100us budget.
 *      Now: 1kHz loop with 1 encoder read per cycle (round-robin, 250Hz each).
 *   2. CRITICAL: LADRC uses discrete-time ESO (from stage4_single_ladrc_fixed.c)
 *      with exact ZOH discretization and pole placement for guaranteed stability.
 *   3. Limit switches now STOP PIO immediately (disable SM) instead of just holding target.
 *   4. Encoder error recovery: don't update position on failed reads, track errors.
 *   5. PIO state machine properly disabled/cleared/restarted each cycle.
 *   6. Output slew limiting and deadband added.
 *   7. Encoder errors reported via health_bits to Master.
 *   8. Re-added <math.h> for expf() (was previously removed as "unused").
 *   9. FIX (Core 0 USB hang): Changed from blocking spi_write_read_blocking()
 *      to polling spi_is_readable() first. CS stays on GPIO_FUNC_SPI.
 *      Non-blocking: USB serial processed every iteration, SPI only when
 *      Master actively clocks data. See JOURNAL.md for details.
 *
 * Hardware (Slave 1 - Arm Base, or Slave 2 - Wrist):
 *   Slave 1: J1=GP16/17, J2A=GP18/19, J2B=GP20/21, J3=GP26/27
 *       Enc I2C: J1=GP12/13, J2A=GP6/7, J2B=GP8/9, J3=GP10/11
 *   Limits: J1=GP14, J2=GP15, J3=GP22
 *   Enable: GP28 (active LOW)
 *
 *   Slave 2: J4=GP16/17, J5=GP18/19, J6=GP20/21, Gripper=GP26/27
 *       Enc I2C: J4=GP12/13, J5=GP8/9, J6=GP10/11
 *   Limits: J4=GP14, J5=GP15, J6=GP22
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "../inc/nite_transmission.h"
#include "../inc/nite_multicore.h"
#include "../inc/nite_config.h"

#ifdef USE_TMC2209_UART
#include "../inc/nite_tmc2209.h"
#endif

#include "freq_generator.pio.h"

// ==========================================================================
// Configuration
// ==========================================================================
#define AS5600_ADDR     0x36
#define SPI_SCK_PIN     2
#define SPI_TX_PIN      3
#define SPI_RX_PIN      4
#define SPI_CS_PIN      5
#define ENABLE_PIN      28

// FIXED: Reduced from 10kHz to 1kHz — software I2C bit-bang at 1us/bit
// takes ~80us per encoder. Reading all 4 at 10kHz was impossible (320us > 100us budget).
#define CTRL_HZ         1000
#define CTRL_PERIOD_US  1000
// NOTE: MAX_SPEED / OUTPUT_SLEW / DEADBAND_C are now config-driven
// (nite_slave_cfg_t, loaded from flash or pushed by the Master over SPI).
#define MAX_CONSECUTIVE_ENCODER_FAILS 100

// Axis pin mappings (indexed by axis number 0-3)
// FIXED (pin conflict): Slave 1 and Slave 2 have DIFFERENT encoder layouts.
// Build with -DNITE_SLAVE_ID=2 (see CMakeLists: servo_firmware_s2) for the
// wrist Pico. Axis 3 (gripper) on Slave 2 has no encoder -> 0xFF sentinel.
#if defined(NITE_SLAVE_ID) && (NITE_SLAVE_ID == 2)
// Slave 2 (Wrist): J4=12/13, J5=8/9, J6=10/11, Gripper=no encoder (0xFF)
static const uint SDA_PINS[AXES_PER_SLAVE] = { 12,  8, 10, 0xFF };
static const uint SCL_PINS[AXES_PER_SLAVE] = { 13,  9, 11, 0xFF };
static const uint STEP_PINS[AXES_PER_SLAVE] = { 16, 18, 20, 26 };
static const uint DIR_PINS[AXES_PER_SLAVE]  = { 17, 19, 21, 27 };
#else
// Slave 1 (Arm Base): J1=12/13, J2A=6/7, J2B=8/9, J3=10/11
static const uint SDA_PINS[AXES_PER_SLAVE] = { 12,  6,  8, 10 };
static const uint SCL_PINS[AXES_PER_SLAVE] = { 13,  7,  9, 11 };
static const uint STEP_PINS[AXES_PER_SLAVE] = { 16, 18, 20, 26 };
static const uint DIR_PINS[AXES_PER_SLAVE]  = { 17, 19, 21, 27 };
#endif

/** True if this axis has an AS5600 encoder attached (SDA pin is valid). */
static inline bool axis_has_encoder(uint axis) {
    return SDA_PINS[axis] != 0xFF;
}

// 3 limit switches for 3 mechanical joints (J2B is dual-motor, shares J2 limit)
#define NUM_LIMITS 3
static const uint LIMIT_PINS[NUM_LIMITS] = { 14, 15, 22 };

// ==========================================================================
// State
// ==========================================================================
volatile int32_t current_positions[AXES_PER_SLAVE] = {0};
volatile int16_t following_errors[AXES_PER_SLAVE] = {0};
static int32_t prev_raw_angles[AXES_PER_SLAVE] = {0};
static int enc_cycle_counter = 0;

// FIXED (position corruption): after a prolonged failure streak we no longer
// jam prev_raw to 0xFFFF (an invalid raw). Instead we flag the axis for a
// re-seed: the next successful read re-anchors prev_raw WITHOUT integrating
// a bogus delta into current_positions.
static bool enc_needs_reseed[AXES_PER_SLAVE] = {false};

// ==========================================================================
// Config (GRBL-style single-file configuration — our slice)
// ==========================================================================
#if defined(NITE_SLAVE_ID) && (NITE_SLAVE_ID == 2)
#define MY_CFG_FLASH_OFFSET NITE_CFG_OFFSET_SLAVE2
#define MY_CFG_SLICE(f) ((f).slave2)
#else
#define MY_CFG_FLASH_OFFSET NITE_CFG_OFFSET_SLAVE1
#define MY_CFG_SLICE(f) ((f).slave1)
#endif

static nite_config_t g_cfg_full;            // full blob from flash / SPI
static volatile nite_slave_cfg_t g_cfg;     // active slice (written by Core 0)
static volatile uint32_t g_cfg_gen = 0;     // bumped when config changes
static uint32_t cfg_ack_remaining = 0;      // cycles to hold the ACK bit

// Encoder error tracking
static bool encoder_error[AXES_PER_SLAVE] = {false};
static uint32_t encoder_error_count[AXES_PER_SLAVE] = {0};

// ==========================================================================
// Fast Software Bit-Bang I2C
// ==========================================================================
static void i2c_bb_init(uint index) {
    if (!axis_has_encoder(index)) return;  // no encoder on this axis
    gpio_init(SDA_PINS[index]);
    gpio_init(SCL_PINS[index]);
    gpio_set_dir(SDA_PINS[index], GPIO_IN);
    gpio_set_dir(SCL_PINS[index], GPIO_OUT);
    gpio_put(SCL_PINS[index], 1);
    gpio_pull_up(SDA_PINS[index]);
    gpio_pull_up(SCL_PINS[index]);
}

static inline void i2c_bb_delay(void) { sleep_us(1); }

static void i2c_bb_start(uint index) {
    gpio_set_dir(SDA_PINS[index], GPIO_OUT);
    gpio_put(SDA_PINS[index], 0);
    i2c_bb_delay();
    gpio_put(SCL_PINS[index], 0);
    i2c_bb_delay();
}

static void i2c_bb_stop(uint index) {
    gpio_set_dir(SDA_PINS[index], GPIO_OUT);
    gpio_put(SDA_PINS[index], 0);
    i2c_bb_delay();
    gpio_put(SCL_PINS[index], 1);
    i2c_bb_delay();
    gpio_set_dir(SDA_PINS[index], GPIO_IN);
    i2c_bb_delay();
}

static bool i2c_bb_write_byte(uint index, uint8_t byte) {
    gpio_set_dir(SDA_PINS[index], GPIO_OUT);
    for (int i = 0; i < 8; i++) {
        gpio_put(SDA_PINS[index], (byte & 0x80) ? 1 : 0);
        i2c_bb_delay();
        gpio_put(SCL_PINS[index], 1);
        i2c_bb_delay();
        gpio_put(SCL_PINS[index], 0);
        byte <<= 1;
    }
    gpio_set_dir(SDA_PINS[index], GPIO_IN);
    i2c_bb_delay();
    gpio_put(SCL_PINS[index], 1);
    i2c_bb_delay();
    bool ack = (gpio_get(SDA_PINS[index]) == 0);
    gpio_put(SCL_PINS[index], 0);
    return ack;
}

static uint8_t i2c_bb_read_byte(uint index, bool send_ack) {
    gpio_set_dir(SDA_PINS[index], GPIO_IN);
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        gpio_put(SCL_PINS[index], 1);
        i2c_bb_delay();
        if (gpio_get(SDA_PINS[index])) byte |= 1;
        gpio_put(SCL_PINS[index], 0);
        i2c_bb_delay();
    }
    gpio_set_dir(SDA_PINS[index], GPIO_OUT);
    gpio_put(SDA_PINS[index], send_ack ? 0 : 1);
    i2c_bb_delay();
    gpio_put(SCL_PINS[index], 1);
    i2c_bb_delay();
    gpio_put(SCL_PINS[index], 0);
    gpio_set_dir(SDA_PINS[index], GPIO_IN);
    return byte;
}

static uint16_t read_as5600_angle(uint index) {
    // FIXED (bus wedge): every NACK now issues a proper STOP so the bus is
    // never left mid-transaction (previously a NACK on the reg/data writes
    // led to a repeated START without STOP -> bus stuck until power cycle).
    i2c_bb_start(index);
    if (!i2c_bb_write_byte(index, AS5600_ADDR << 1)) {
        i2c_bb_stop(index);
        return 0xFFFF;
    }
    if (!i2c_bb_write_byte(index, 0x0C)) { // Raw Angle High register
        i2c_bb_stop(index);
        return 0xFFFF;
    }
    i2c_bb_start(index);            // Repeated start
    if (!i2c_bb_write_byte(index, (AS5600_ADDR << 1) | 1)) {
        i2c_bb_stop(index);
        return 0xFFFF;
    }
    uint8_t msb = i2c_bb_read_byte(index, true);
    uint8_t lsb = i2c_bb_read_byte(index, false);
    i2c_bb_stop(index);
    return ((uint16_t)msb << 8) | lsb;
}

// ==========================================================================
// PIO Stepper Init
// ==========================================================================
static void astra_freq_generator_init(PIO pio, uint sm, uint offset, uint step_pin) {
    pio_gpio_init(pio, step_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, step_pin, 1, true);
    pio_sm_config c = freq_generator_program_get_default_config(offset);
    sm_config_set_set_pins(&c, step_pin, 1);
    sm_config_set_out_shift(&c, false, false, 32);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// ==========================================================================
// Discrete-Time LADRC (stable ESO via exact ZOH discretization)
// ==========================================================================
typedef struct {
    float wc, wo, b0;
    float kp, kd;
    float l1, l2, l3;
    float z1, z2, z3;
    float h;
    float u_prev;
} ladrc_ctrl_t;

static inline void ladrc_init(ladrc_ctrl_t *c, float wc, float wo, float b0, float h) {
    c->wc = wc; c->wo = wo; c->b0 = b0; c->h = h;
    c->kp = wc * wc;
    c->kd = 2.0f * wc;
    float beta = expf(-wo * h);
    float omb = 1.0f - beta;
    c->l1 = 3.0f * omb;
    c->l2 = omb * omb * (5.0f + beta) / (2.0f * h);
    c->l3 = omb * omb * omb / (h * h);
    c->z1 = c->z2 = c->z3 = 0.0f;
    c->u_prev = 0.0f;
}

static inline float ladrc_update(ladrc_ctrl_t *c, float ref, float act, float max_speed) {
    float e = act - c->z1;
    float z1_next = c->z1 + c->h * c->z2 + 0.5f * c->h * c->h * c->z3
                   + 0.5f * c->b0 * c->h * c->h * c->u_prev + c->l1 * e;
    float z2_next = c->z2 + c->h * c->z3 + c->b0 * c->h * c->u_prev + c->l2 * e;
    float z3_next = c->z3 + c->l3 * e;
    c->z1 = z1_next; c->z2 = z2_next; c->z3 = z3_next;
    float u0 = c->kp * (ref - c->z1) - c->kd * c->z2;
    float u = (u0 - c->z3) / c->b0;
    if (u > max_speed)  u = max_speed;
    if (u < -max_speed) u = -max_speed;
    c->u_prev = u;
    return u;
}

// ==========================================================================
// Forward declarations
// ==========================================================================
void astra_calibrate_encoders(void);

// ==========================================================================
// Serial Command Protocol
// ==========================================================================
#define SERIAL_BUF_SIZE    128
#define MAX_TOKENS         16
#define MOVE_TIMEOUT_MS    10000

static char serial_buf[SERIAL_BUF_SIZE];
static int serial_buf_pos = 0;

// Non-blocking move tracking — lets Core 0 continue polling SPI + USB
// while Core 1 executes the move. Next state checked in main loop.
typedef enum {
    MOVE_IDLE,
    MOVE_WAITING
} move_state_t;
static move_state_t move_state = MOVE_IDLE;
static uint32_t move_start_ms = 0;

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

static inline void serial_respond(const char *s) { printf("%s\n", s); }

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
    if (serial_buf_pos < SERIAL_BUF_SIZE - 1)
        serial_buf[serial_buf_pos++] = (char)c;
    return false;
}

static void cmd_move(int argc, const char **tokens) {
    if (argc < 8) {
        serial_respond("ERR MOVE requires: j1 j2 j3 j4 j5 j6 SPD speed");
        return;
    }
    int32_t targets[AXES_PER_SLAVE];
    for (int i = 0; i < AXES_PER_SLAVE; i++)
        targets[i] = (int32_t)(atof(tokens[i + 1]) * 1000.0f);
    astra_push_targets(targets);
    serial_respond("OK");
    // Non-blocking: set state machine, return immediately to main loop
    move_state = MOVE_WAITING;
    move_start_ms = to_ms_since_boot(get_absolute_time());
}

// Called every main loop iteration — non-blocking move completion check
static void poll_move_completion(void) {
    if (move_state != MOVE_WAITING) return;

    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - move_start_ms;
    if (elapsed > MOVE_TIMEOUT_MS) {
        serial_respond("ERR Timeout");
        move_state = MOVE_IDLE;
        return;
    }

    bool settled = true;
    for (int i = 0; i < AXES_PER_SLAVE; i++) {
        if (abs(following_errors[i]) > 10) { settled = false; break; }
    }
    if (settled) {
        serial_respond("DONE");
        move_state = MOVE_IDLE;
    }
}

static void cmd_status(void) {
    char resp[128];
    snprintf(resp, sizeof(resp), "POS %d %d %d %d",
        (int)(current_positions[0] / 1000),
        (int)(current_positions[1] / 1000),
        (int)(current_positions[2] / 1000),
        (int)(current_positions[3] / 1000));
    serial_respond(resp);
}

static void cmd_stop(void) {
    int32_t zeros[AXES_PER_SLAVE] = {0};
    astra_push_targets(zeros);
    serial_respond("OK");
    move_state = MOVE_IDLE;  // Cancel any pending move
}

static void cmd_home(void) {
    astra_calibrate_encoders();
    serial_respond("OK");
    serial_respond("DONE");
}

static void cmd_dout(int argc, const char **tokens) {
    if (argc < 3) { serial_respond("ERR DOUT requires: pin value"); return; }
    int pin = atoi(tokens[1]);
    int val = atoi(tokens[2]);
    if (pin >= 0 && pin < AXES_PER_SLAVE)
        gpio_put(DIR_PINS[pin], val ? 1 : 0);
    serial_respond("OK");
}

static void serial_command_handler(void) {
    const char *tokens[MAX_TOKENS];
    int argc = tokenize(serial_buf, tokens, MAX_TOKENS);
    if (argc == 0) return;
    if (strcmp(tokens[0], "MOVEJ") == 0 || strcmp(tokens[0], "MOVEL") == 0) {
        cmd_move(argc, tokens);
    } else if (strcmp(tokens[0], "STATUS") == 0) {
        cmd_status();
    } else if (strcmp(tokens[0], "STOP") == 0) {
        cmd_stop();
    } else if (strcmp(tokens[0], "HOME") == 0) {
        cmd_home();
    } else if (strcmp(tokens[0], "DOUT") == 0) {
        cmd_dout(argc, tokens);
    } else if (strcmp(tokens[0], "ID") == 0) {
        // Stable identity for host automation tools (pico_dev.py etc.)
#if defined(NITE_SLAVE_ID) && (NITE_SLAVE_ID == 2)
        serial_respond("NITE-SLAVE2");
#else
        serial_respond("NITE-SLAVE1");
#endif
    } else if (strcmp(tokens[0], "BOOTSEL") == 0) {
        // Software reboot straight into the USB bootloader.
        serial_respond("OK rebooting to BOOTSEL...");
        sleep_ms(50);
        reset_usb_boot(0, 0);
    } else {
        serial_respond("ERR Unknown command");
    }
}

// ==========================================================================
// Core 1: LADRC Control Loop (1kHz)
// ==========================================================================
void core1_control_loop(void) {
    int32_t local_targets[AXES_PER_SLAVE] = {0};
    uint32_t sys_clk = clock_get_hz(clk_sys);

    // FIXED: Discrete-time LADRC with guaranteed stability.
    // Gains are config-driven (defaults: wc=20, wo=100, b0=120).
    nite_slave_cfg_t cfg;
    uint32_t last_cfg_gen = 0;
    cfg = g_cfg;
    ladrc_ctrl_t controllers[AXES_PER_SLAVE];
    for (int i = 0; i < AXES_PER_SLAVE; i++) {
        ladrc_init(&controllers[i], cfg.ladrc_wc, cfg.ladrc_wo, cfg.ladrc_b0, 1.0f / CTRL_HZ);
    }

    float prev_output[AXES_PER_SLAVE] = {0.0f};
    // FIXED (step-rate bug): fractional step accumulators + PIO run-state.
    float step_accum[AXES_PER_SLAVE] = {0.0f};
    bool sm_running[AXES_PER_SLAVE] = {false};

    printf("NITE-CORE-1: LADRC Engine at %d Hz (staggered encoder reads)\n", CTRL_HZ);

    while (1) {
        // Flash-write handshake: park here (RAM spin) while Core 0 programs flash
        nite_cfg_flash_guard();

        uint64_t start_time = time_us_64();

        // Re-tune LADRC if the config changed (Core 0 applies new blobs over SPI)
        if (g_cfg_gen != last_cfg_gen) {
            last_cfg_gen = g_cfg_gen;
            cfg = g_cfg;
            for (int i = 0; i < AXES_PER_SLAVE; i++)
                ladrc_init(&controllers[i], cfg.ladrc_wc, cfg.ladrc_wo, cfg.ladrc_b0, 1.0f / CTRL_HZ);
        }

        // 1. Pull latest targets
        astra_pull_targets(local_targets);

        // 2. Check limit switches — FIXED: NOW stops PIO immediately on trigger
        //    Simply holding target allowed overshoot. Disabling SM gives instant stop.
        // FIXED (safety, CRITICAL): limit switches are NC -> GND with
        // pull-ups, so a PRESSED switch reads LOW. The old gpio_get()==HIGH
        // check was inverted: e-stop fired when switches were released and
        // the arm ran freely into a limit!
        bool limit_triggered = false;
        for (int i = 0; i < NUM_LIMITS; i++) {
            bool level = gpio_get(LIMIT_PINS[i]);
            // Configurable polarity: 0 = active-low (NC->GND), 1 = active-high
            bool trig = cfg.limit_polarity[i] ? level : !level;
            if (trig) {
                limit_triggered = true;
                break;
            }
        }
        if (limit_triggered) {
            for (int j = 0; j < AXES_PER_SLAVE; j++)
                local_targets[j] = current_positions[j];
        }

        // 3. Read ONE encoder per cycle (round-robin)
        //    FIXED: 10kHz was impossible — 4 encoders via 1us bit-bang = 320us > 100us budget.
        //    Now: 1kHz with 1 encoder/cycle = 250Hz per axis (still plenty for mechanical arm)
        enc_cycle_counter = (enc_cycle_counter + 1) % AXES_PER_SLAVE;
        int enc_idx = enc_cycle_counter;

        if (axis_has_encoder(enc_idx)) {
            uint16_t raw_val = read_as5600_angle(enc_idx);
            if (raw_val != 0xFFFF) {
                int32_t raw = raw_val & 0x0FFF;
                if (enc_needs_reseed[enc_idx]) {
                    // Prolonged failure: re-anchor WITHOUT position jump
                    prev_raw_angles[enc_idx] = raw;
                    enc_needs_reseed[enc_idx] = false;
                } else {
                    int32_t diff = raw - prev_raw_angles[enc_idx];
                    if (diff > 2048) {
                        current_positions[enc_idx] -= (4096 - diff);
                    } else if (diff < -2048) {
                        current_positions[enc_idx] += (4096 + diff);
                    } else {
                        current_positions[enc_idx] += diff;
                    }
                    prev_raw_angles[enc_idx] = raw;
                }
                encoder_error[enc_idx] = false;
                encoder_error_count[enc_idx] = 0;
            } else {
                // FIXED: Don't update position or prev_raw on failed read.
                // Prevents position spike when encoder reconnects.
                encoder_error[enc_idx] = true;
                encoder_error_count[enc_idx]++;
                if (encoder_error_count[enc_idx] > MAX_CONSECUTIVE_ENCODER_FAILS) {
                    // Flag for re-seed on next successful read (no 0xFFFF)
                    enc_needs_reseed[enc_idx] = true;
                }
            }
        }
        // No-encoder axis (e.g. Slave 2 gripper): nothing to read; the LADRC
        // section below holds the axis at its current position.

        // 4. LADRC update for all axes
        for (int i = 0; i < AXES_PER_SLAVE; i++) {
            float ref = (float)local_targets[i];
            float act = (float)current_positions[i];
            following_errors[i] = (int16_t)(ref - act);

            float speed_out;
            if (abs(following_errors[i]) <= (int16_t)cfg.deadband) {
                speed_out = 0.0f;
            } else {
                speed_out = ladrc_update(&controllers[i], ref, act, cfg.max_speed);
            }

            // Slew rate limiting (config-driven)
            float delta = speed_out - prev_output[i];
            if (delta > cfg.output_slew)  speed_out = prev_output[i] + cfg.output_slew;
            if (delta < -cfg.output_slew) speed_out = prev_output[i] - cfg.output_slew;
            prev_output[i] = speed_out;

            if (limit_triggered) {
                speed_out = 0.0f;
                prev_output[i] = 0.0f;
            }

            // No encoder feedback (e.g. gripper axis on Slave 2): LADRC
            // cannot close the loop here — hold position to avoid runaway.
            if (!axis_has_encoder(i)) {
                speed_out = 0.0f;
                prev_output[i] = 0.0f;
                following_errors[i] = 0;
            }

            // Map to PIO
            int32_t speed = (int32_t)speed_out;
            if (speed < 0) {
                // Configurable direction inversion per axis
                gpio_put(DIR_PINS[i], cfg.dir_invert[i] ? 0 : 1);
                speed = -speed;
            } else {
                gpio_put(DIR_PINS[i], cfg.dir_invert[i] ? 1 : 0);
            }

            // FIXED (CRITICAL step-rate bug): the old code restarted the SM
            // and pushed ONE payload per 1ms cycle, so speed/STEPS_PER_CYCLE
            // (max 1500/1000) collapsed to exactly 1 step per cycle — every
            // nonzero speed ran at a constant ~1000 steps/s. Now the SM runs
            // continuously and a fractional accumulator integrates speed/CTRL_HZ
            // each cycle, emitting steps at the TRUE commanded rate with the
            // correct inter-step spacing.
            if (speed == 0) {
                if (sm_running[i]) {
                    // Hard stop: disable SM + clear FIFO + restart (no glitches)
                    pio_sm_set_enabled(pio0, i, false);
                    pio_sm_clear_fifos(pio0, i);
                    pio_sm_restart(pio0, i);
                    sm_running[i] = false;
                }
                step_accum[i] = 0.0f;
            } else {
                if (!sm_running[i]) {
                    // Bring the SM up cleanly on first motion
                    pio_sm_set_enabled(pio0, i, false);
                    pio_sm_clear_fifos(pio0, i);
                    pio_sm_restart(pio0, i);
                    pio_sm_set_enabled(pio0, i, true);
                    sm_running[i] = true;
                }
                // Integrate fractional steps for this cycle
                step_accum[i] += (float)speed / (float)CTRL_HZ;
                int32_t steps_now = (int32_t)step_accum[i];
                if (steps_now > 0) {
                    // Per-step spacing: sys_clk/speed minus the fixed 240-cycle
                    // high pulse inside freq_generator.pio (loop1 = 24*10).
                    uint32_t delay_cycles = sys_clk / (uint32_t)speed;
                    if (delay_cycles > 240) delay_cycles -= 240;
                    if (delay_cycles > 0x3FFFFF) delay_cycles = 0x3FFFFF;
                    if (delay_cycles < 30) delay_cycles = 30;
                    uint32_t payload = ((uint32_t)steps_now & 0x3FF) | (delay_cycles << 10);
                    // Non-blocking put: never stall the 1kHz control loop.
                    // FIXED (lossless): only subtract the steps that were
                    // actually queued — if the FIFO is momentarily full the
                    // accumulator keeps them and they emit next cycle.
                    if (!pio_sm_is_tx_fifo_full(pio0, i)) {
                        step_accum[i] -= (float)steps_now;
                        pio_sm_put(pio0, i, payload);
                    }
                }
            }
        }

        // 5. Deterministic timing
        uint64_t elapsed = time_us_64() - start_time;
        if (elapsed < CTRL_PERIOD_US)
            busy_wait_us(CTRL_PERIOD_US - (uint32_t)elapsed);
    }
}

// ==========================================================================
// Calibration
// ==========================================================================
void astra_calibrate_encoders(void) {
    printf("ASTRA-BOOT: Running absolute calibration...\n");
    int ok_count = 0;
    for (int i = 0; i < AXES_PER_SLAVE; i++) {
        if (!axis_has_encoder(i)) {
            printf("  Axis %d: no encoder (open-loop axis) — skipped\n", i);
            continue;
        }
        uint16_t initial_val = read_as5600_angle(i);
        if (initial_val != 0xFFFF) {
            prev_raw_angles[i] = initial_val & 0x0FFF;
            current_positions[i] = 0;
            encoder_error[i] = false;
            encoder_error_count[i] = 0;
            enc_needs_reseed[i] = false;
            printf("  Axis %d Calibrated to raw: %d\n", i, prev_raw_angles[i]);
            ok_count++;
        } else {
            printf("  WARNING: Axis %d Encoder FAIL! Check wiring.\n", i);
            // FIXED: if the encoder is dead at boot and reconnects later,
            // re-anchor on the first successful read instead of integrating
            // a bogus delta against prev_raw=0 (position jump).
            enc_needs_reseed[i] = true;
        }
    }
    if (ok_count == 0)
        printf("  CRITICAL: No encoders detected! System may not function.\n");
    printf("ASTRA-BOOT: Calibration done (%d/%d axes OK).\n", ok_count, AXES_PER_SLAVE);
}

// ==========================================================================
// Main
// ==========================================================================
int main() {
    stdio_init_all();

    // Load our config slice from flash (or factory defaults on first boot)
    if (nite_cfg_load(&g_cfg_full, MY_CFG_FLASH_OFFSET)) {
        g_cfg = MY_CFG_SLICE(g_cfg_full);
        printf("Config loaded from flash\n");
    } else {
        nite_cfg_defaults(&g_cfg_full);
        g_cfg = MY_CFG_SLICE(g_cfg_full);
        printf("Config: factory defaults (none in flash yet)\n");
    }
    g_cfg_gen = 1;

    // I2C buses
    for (int i = 0; i < AXES_PER_SLAVE; i++)
        i2c_bb_init(i);

    // Direction pins
    for (int i = 0; i < AXES_PER_SLAVE; i++) {
        gpio_init(DIR_PINS[i]);
        gpio_set_dir(DIR_PINS[i], GPIO_OUT);
        gpio_put(DIR_PINS[i], 0);
    }

    // Limit switches (pull-up, NC -> GND)
    for (int i = 0; i < NUM_LIMITS; i++) {
        gpio_init(LIMIT_PINS[i]);
        gpio_set_dir(LIMIT_PINS[i], GPIO_IN);
        gpio_pull_up(LIMIT_PINS[i]);
    }

    // Enable pin — start disabled
#ifndef USE_TMC2209_UART
    // When TMC2209 UART is active, tmc2209_uart_init() handles ENABLE pin init
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_OUT);
    gpio_put(ENABLE_PIN, 1);
#endif

    // PIO step generators
    uint pio_offset = pio_add_program(pio0, &freq_generator_program);
    for (int i = 0; i < AXES_PER_SLAVE; i++)
        astra_freq_generator_init(pio0, i, pio_offset, STEP_PINS[i]);

    // SPI slave — CS stays on GPIO_FUNC_SPI for proper SS edge detection
    // FIXED: Core 0 was hanging on spi_write_read_blocking() when Master wasn't
    // actively clocking. Now polls spi_is_readable() (non-blocking) before
    // committing to the full 26-byte exchange.
    spi_init(spi0, 10 * 1000 * 1000);
    spi_set_slave(spi0, true);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CS_PIN, GPIO_FUNC_SPI);

    // Calibrate
    astra_calibrate_encoders();

#ifdef USE_TMC2209_UART
    // Initialize TMC2209 UART and configure all wrist drivers
    printf("\n═══ TMC2209 UART Configuration ═══\n");
    tmc2209_uart_init();

    // Configure all drivers from the config file (current, microsteps, mode)
    nite_cfg_apply_tmc(&g_cfg);

    // Enable drivers now that configuration is complete
    tmc2209_set_enabled(true);
    printf("\n═══ Boot sequence complete ═══\n\n");
#else
    // Enable drivers directly (no TMC2209 UART — TMC2160 or raw STEP/DIR mode)
    gpio_put(ENABLE_PIN, 0);
#endif

    // Launch Core 1
    astra_sync_init();
    nite_cfg_set_core1_running(true);   // enables the flash-write handshake
    multicore_launch_core1(core1_control_loop);

    // Core 0: SPI + USB
    astra_spi_cmd_t cmd;
    astra_spi_feedback_t feedback;
    memset(&feedback, 0, sizeof(feedback));

    printf("NITE-CORE-0: Serial parser active (MOVEJ, STATUS, STOP, HOME, DOUT)\n");
    printf("  SPI: polling spi_is_readable() — non-blocking, CS on hardware\n");
    printf("  USB serial: always responsive\n");

    while (1) {
        // USB serial polling — NON-BLOCKING, always runs because SPI is
        // only done when Master has started clocking data into RX FIFO
        while (serial_read_line())
            serial_command_handler();

        // Non-blocking move completion check (instead of blocking sleep_ms loop)
        poll_move_completion();

        // Build feedback with encoder error reporting via health_bits
        for (int i = 0; i < AXES_PER_SLAVE; i++) {
            feedback.actual[i] = current_positions[i];
            feedback.error[i] = following_errors[i];
        }
        feedback.health_bits = 0x00;
        for (int i = 0; i < AXES_PER_SLAVE; i++) {
            if (encoder_error[i])
                feedback.health_bits |= (1 << i);
        }
        // Config applied -> hold the ACK bit so the Master sees it (bit 7)
        if (cfg_ack_remaining > 0) {
            feedback.health_bits |= ASTRA_CFG_ACK_BIT;
            cfg_ack_remaining--;
        }
        feedback.crc = astra_crc8((uint8_t*)&feedback, sizeof(feedback) - 1);

        // FIXED: Poll RX FIFO non-blocking before committing to the frame.
        // Previously, spi_write_read_blocking() blocked forever in slave mode
        // because it waits for the Master to clock 26 bytes.
        // The Master only initiates transfers every ~1ms, so Core 0
        // was stuck ~99.9% of the time — USB serial never processed.
        //
        // spi_is_readable() checks the RX FIFO level register — it returns
        // immediately (non-blocking) even in slave mode. When the Master
        // starts clocking, the first byte arrives in ~0.8µs (at 10MHz).
        // spi_is_readable() does NOT consume the byte (it's a peek).
        //
        // CS-PER-BYTE (docs/spi_debug_issues.md Issue 8): the RP2040 SPI
        // slave only reloads its TX shift register from the TX FIFO when CS
        // re-asserts (HIGH→LOW edge), so the Master toggles CS between every
        // byte. Mirror that here with 1-byte transfers so each byte aligns
        // with the Master's per-byte CS pulses. A multi-byte burst under one
        // CS assertion only transfers byte 0 correctly in BOTH directions.
        if (spi_is_readable(spi0)) {
            uint8_t *fb_bytes = (uint8_t*)&feedback;
            uint8_t *cmd_bytes = (uint8_t*)&cmd;
            for (int i = 0; i < SPI_XFER_SIZE; i++) {
                spi_write_read_blocking(spi0, &fb_bytes[i], &cmd_bytes[i], 1);
            }

            // Config frame from the Master (GRBL-style config broadcast)
            if (cmd.control_word == ASTRA_CFG_TAG) {
                if (astra_crc8((uint8_t*)&cmd, sizeof(cmd) - 1) == cmd.crc) {
                    if (nite_cfg_rx_frame(&cmd)) {
                        nite_config_t full;
                        if (nite_cfg_rx_result(&full)) {
                            g_cfg_full = full;
                            g_cfg = MY_CFG_SLICE(g_cfg_full);
                            g_cfg_gen++;   // Core 1 re-tunes on next cycle
                            if (nite_cfg_save(&full, MY_CFG_FLASH_OFFSET)) {
                                printf("CFG-APPLIED (saved to flash)\n");
                            } else {
                                printf("CFG-APPLIED (flash save FAILED)\n");
                            }
                            nite_cfg_apply_tmc(&g_cfg);
                            cfg_ack_remaining = 200;  // hold ACK bit ~200ms
                        }
                        nite_cfg_rx_reset();
                    }
                }
            } else if (astra_crc8((uint8_t*)&cmd, sizeof(cmd) - 1) == cmd.crc) {
                if (cmd.control_word == 0) {
                    // FIXED (safety): Master cleared enable bits -> e-stop.
                    // Slave previously ignored control_word entirely, so the
                    // Master's enable/disable had no effect at the drivers.
                    int32_t zeros[AXES_PER_SLAVE] = {0};
                    astra_push_targets(zeros);
                } else {
                    astra_push_targets(cmd.target);
                }
            }
        }
    }
    return 0;
}
