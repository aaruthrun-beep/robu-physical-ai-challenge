/**
 * @file spi_cmd_slave_s1.c
 * @brief SPI Command Slave 1 — Arm Base
 *
 * Simple GPIO step generation (no PIO). Shoulder (J2A+J2B) always paired.
 * CS-per-byte SPI. 50kHz. 2-frame protocol.
 * AS5600 magnetic encoders via bit-bang I2C.
 *
 * Axis map: 0=J1, 1=Shoulder(J2A+J2B), 2=J3
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "motion_profile.h"
#include "homing.h"
#include "config_store.h"
#include "nite_spi_proto.h"
#include "firmware_version.h"

#define SPI_PORT    spi0
// BIT-BANG slave SPI on GP2/3/4/5 — slave 1's ORIGINAL soldered pins.
// Bit-bang (not hardware SPI) so we get the same FRAME RESYNC that fixed
// slave 2: hardware-SPI slave mode cannot detect the 50ms inter-frame CS
// gap (CS is muxed to the peripheral), so it stayed frame-misaligned.
// Bit-bang works on any pins. SCK=2 (in), MISO=3 (out, tri-state when CS
// high), MOSI=4 (in), CS=5 (in, active low).
#define PIN_SCK     2
#define PIN_MISO    3
#define PIN_MOSI    4
#define PIN_CS      5

// Build a v2 reply frame (sync + joint_id echo + opcode + payload + seq +
// CRC) into tx. The master validates replies with frame_v2_unpack, so every
// reply MUST be a well-formed v2 frame.
static void v2_reply(uint8_t *tx, uint8_t opcode, uint8_t joint_id,
                     int32_t payload, uint8_t seq) {
    spi_frame_t r = { .sync = FRAME_SYNC_BYTE, .joint_id = joint_id,
                      .opcode = opcode, .payload = payload, .seq = seq };
    frame_v2_pack(&r, tx);
}
// Frame length includes the CRC8 byte (see nite_spi_proto.h)
#define BUF_LEN     NITE_SPI_BUF_LEN

#define EN_PIN      28
#define LIMIT_J1    14
#define LIMIT_J2    15
// LIMIT_J3 back on GP1 (encoder J2A is back on GP6/7, so GP1 is free again)
#define LIMIT_J3    1

#define NJ 4
#define AXIS_J1   0
#define AXIS_J2A  1
#define AXIS_J2B  2
#define AXIS_J3   3

// AS5600 encoder pins — original layout (SPI is back on GP2/3/4/5, so
// encoders return to GP6/7/8/9/10/11 as before)
#define NUM_ENCODERS 4
static const uint ENC_SDA[NUM_ENCODERS] = { 12,  6,  8, 10 };
static const uint ENC_SCL[NUM_ENCODERS] = { 13,  7,  9, 11 };
#define AS5600_ADDR  0x36

typedef struct {
    uint step_pin;
    uint dir_pin;
    const char *name;
} Joint;

static const Joint joints[NJ] = {
    {16, 17, "J1"},
    {18, 19, "J2A"},
    {20, 21, "J2B"},
    {26, 27, "J3"},
};

// ── AS5600 Bit-Bang I2C ──────────────────────────────────────────────
static void bb_delay(void) { sleep_us(1); }

// Per-transaction deadline so a stuck encoder bus (clock stretching, SDA
// held low by a slave, etc.) can never hang the SPI loop forever. A normal
// read takes ~300us; 2ms is generous.
#define BB_TIMEOUT_US 2000
static uint64_t bb_deadline_us = 0;
static bool bb_error = false;

static bool bb_timed_out(void) {
    return bb_deadline_us != 0 && time_us_64() > bb_deadline_us;
}

static void bb_start(int idx) {
    gpio_set_dir(ENC_SDA[idx], GPIO_OUT);
    gpio_put(ENC_SDA[idx], 0); bb_delay();
    gpio_put(ENC_SCL[idx], 0); bb_delay();
}

static void bb_stop(int idx) {
    gpio_set_dir(ENC_SDA[idx], GPIO_OUT);
    gpio_put(ENC_SDA[idx], 0); bb_delay();
    gpio_put(ENC_SCL[idx], 1); bb_delay();
    gpio_set_dir(ENC_SDA[idx], GPIO_IN); bb_delay();
}

// I2C bus recovery: release SDA and pulse SCL up to 9 times, then STOP.
// This clears a stuck-low SDA / hung slave so the next read starts clean.
static void bb_recover_bus(int idx) {
    gpio_set_dir(ENC_SDA[idx], GPIO_IN);   // release SDA (pull-up drives high)
    gpio_pull_up(ENC_SDA[idx]);
    for (int i = 0; i < 9 && bb_timed_out() == false; i++) {
        gpio_put(ENC_SCL[idx], 0); bb_delay();
        gpio_put(ENC_SCL[idx], 1); bb_delay();
    }
    bb_stop(idx);
}

static bool bb_write_byte(int idx, uint8_t byte) {
    if (bb_error) return false;
    gpio_set_dir(ENC_SDA[idx], GPIO_OUT);
    for (int i = 0; i < 8; i++) {
        if (bb_timed_out()) { bb_error = true; return false; }
        gpio_put(ENC_SDA[idx], (byte & 0x80) ? 1 : 0);
        bb_delay();
        gpio_put(ENC_SCL[idx], 1); bb_delay();
        gpio_put(ENC_SCL[idx], 0); byte <<= 1;
    }
    if (bb_timed_out()) { bb_error = true; return false; }
    gpio_set_dir(ENC_SDA[idx], GPIO_IN); bb_delay();
    gpio_put(ENC_SCL[idx], 1); bb_delay();
    bool ack = (gpio_get(ENC_SDA[idx]) == 0);
    gpio_put(ENC_SCL[idx], 0);
    return ack;
}

static uint8_t bb_read_byte(int idx, bool send_ack) {
    if (bb_error) return 0xFF;
    gpio_set_dir(ENC_SDA[idx], GPIO_IN);
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (bb_timed_out()) { bb_error = true; return 0xFF; }
        byte <<= 1;
        gpio_put(ENC_SCL[idx], 1); bb_delay();
        if (gpio_get(ENC_SDA[idx])) byte |= 1;
        gpio_put(ENC_SCL[idx], 0); bb_delay();
    }
    if (bb_timed_out()) { bb_error = true; return 0xFF; }
    gpio_set_dir(ENC_SDA[idx], GPIO_OUT);
    gpio_put(ENC_SDA[idx], send_ack ? 0 : 1); bb_delay();
    gpio_put(ENC_SCL[idx], 1); bb_delay();
    gpio_put(ENC_SCL[idx], 0);
    gpio_set_dir(ENC_SDA[idx], GPIO_IN);
    return byte;
}

// Start a bounded transaction. Returns the raw 12-bit angle (0-4095) or
// 0xFFFF on error; never hangs.
static uint16_t read_as5600(int idx) {
    if (idx < 0 || idx >= NUM_ENCODERS) return 0xFFFF;
    bb_deadline_us = time_us_64() + BB_TIMEOUT_US;
    bb_error = false;
    bb_start(idx);
    if (bb_error || !bb_write_byte(idx, AS5600_ADDR << 1)) { bb_recover_bus(idx); return 0xFFFF; }
    if (bb_error || !bb_write_byte(idx, 0x0C))             { bb_recover_bus(idx); return 0xFFFF; }
    bb_start(idx);
    if (bb_error || !bb_write_byte(idx, (AS5600_ADDR << 1) | 1)) { bb_recover_bus(idx); return 0xFFFF; }
    uint8_t msb = bb_read_byte(idx, true);
    uint8_t lsb = bb_read_byte(idx, false);
    bb_stop(idx);
    if (bb_error) { bb_recover_bus(idx); return 0xFFFF; }
    return ((uint16_t)(msb & 0x0F) << 8) | lsb;
}

// Bounded read of the AS5600 status register (0x0B): MD/ML/MH diagnostic bits.
// Returns 0xFF on error (mirrors read_as5600's bounded, non-hanging behavior).
static uint8_t read_as5600_status(int idx) {
    if (idx < 0 || idx >= NUM_ENCODERS) return 0xFF;
    bb_deadline_us = time_us_64() + BB_TIMEOUT_US;
    bb_error = false;
    bb_start(idx);
    if (bb_error || !bb_write_byte(idx, AS5600_ADDR << 1)) { bb_recover_bus(idx); return 0xFF; }
    if (bb_error || !bb_write_byte(idx, 0x0B))             { bb_recover_bus(idx); return 0xFF; }
    bb_start(idx);
    if (bb_error || !bb_write_byte(idx, (AS5600_ADDR << 1) | 1)) { bb_recover_bus(idx); return 0xFF; }
    uint8_t status = bb_read_byte(idx, false);
    bb_stop(idx);
    if (bb_error) { bb_recover_bus(idx); return 0xFF; }
    return status;
}

static void enc_init_all(void) {
    for (int i = 0; i < NUM_ENCODERS; i++) {
        gpio_init(ENC_SDA[i]); gpio_set_dir(ENC_SDA[i], GPIO_IN); gpio_pull_up(ENC_SDA[i]);
        gpio_init(ENC_SCL[i]); gpio_set_dir(ENC_SCL[i], GPIO_OUT); gpio_put(ENC_SCL[i], 1); gpio_pull_up(ENC_SCL[i]);
    }
}

static bool en_active = false;
static bool run_active[NJ] = {false};
static struct repeating_timer run_timers[NJ];

// Step engine: per-axis state for timed step sequences
static struct {
    bool active;
    bool continuous;
    int joint_idx;      // which joint(s) to pulse
    int dir;
    uint32_t hz;
    int32_t steps_left;
} step_engine[NJ];

static void set_enable(bool on) {
    en_active = on;
    gpio_put(EN_PIN, on ? 0 : 1);
}

static void set_dir(int idx, int dir) {
    gpio_put(joints[idx].dir_pin, dir > 0);
}

static void pulse_step(int idx) {
    gpio_put(joints[idx].step_pin, 1);
}

static void clear_step(int idx) {
    gpio_put(joints[idx].step_pin, 0);
}

// Timer callback for continuous run
static bool run_timer_cb(struct repeating_timer *t) {
    int idx = (int)(intptr_t)t->user_data;
    gpio_put(joints[idx].step_pin, !gpio_get(joints[idx].step_pin));
    return true;
}

// Timer callback for timed stepping (step N at hz)
// Timer period = half-period. Toggle pin each callback. Count step on falling edge.
static bool any_axis_active(void) {
    for (int i = 0; i < NJ; i++) {
        if (step_engine[i].active) return true;
    }
    return false;
}

static bool step_timer_cb(struct repeating_timer *t) {
    int idx = (int)(intptr_t)t->user_data;
    gpio_put(joints[idx].step_pin, !gpio_get(joints[idx].step_pin));

    if (!gpio_get(joints[idx].step_pin)) {
        step_engine[idx].steps_left--;
    }

    if (step_engine[idx].steps_left <= 0) {
        gpio_put(joints[idx].step_pin, 0);
        step_engine[idx].active = false;
        if (!any_axis_active()) set_enable(false);
        return false;
    }
    return true;
}

// Shoulder: toggle both J2A+J2B step pins simultaneously
static void shoulder_toggle(void) {
    bool val = !gpio_get(joints[AXIS_J2A].step_pin);
    gpio_put(joints[AXIS_J2A].step_pin, val);
    gpio_put(joints[AXIS_J2B].step_pin, val);
}

static void shoulder_clear(void) {
    clear_step(AXIS_J2A);
    clear_step(AXIS_J2B);
}

static bool shoulder_step_cb(struct repeating_timer *t) {
    (void)t;
    shoulder_toggle();

    if (!gpio_get(joints[AXIS_J2A].step_pin)) {
        step_engine[AXIS_J2A].steps_left--;
    }

    if (step_engine[AXIS_J2A].steps_left <= 0) {
        shoulder_clear();
        step_engine[AXIS_J2A].active = false;
        step_engine[AXIS_J2B].active = false;
        if (!any_axis_active()) set_enable(false);
        return false;
    }
    return true;
}

static bool shoulder_run_cb(struct repeating_timer *t) {
    (void)t;
    shoulder_toggle();
    return true;
}

static void motion_start_rate_timer(int axis);  // fwd decl

static void start_continuous(int axis, int dir, uint32_t hz) {
    // CRITICAL SECTION: the rate-timer ISR reads step_engine[] and
    // the step pins. Disable IRQs while mutating so a command can't interleave
    // with the ISR (which could drop/double a step or leave a pin stuck HIGH).
    // NOTE: the config's direction-inversion bit (#CFG di=1) is applied by
    // apply_dir() on every rate-alarm tick — NOT here. Applying it here too
    // double-inverts and makes the jog ignore the Invert toggle.
    uint32_t irq_save = save_and_disable_interrupts();
    if (axis == 1) {
        // Shoulder (J2A+J2B paired): accelerate with motion_jog_start.
        step_engine[AXIS_J2A].active = true;
        step_engine[AXIS_J2A].continuous = true;
        step_engine[AXIS_J2A].dir = dir;
        step_engine[AXIS_J2A].hz = hz;
        step_engine[AXIS_J2B].active = true;
        set_dir(AXIS_J2A, dir);
        set_dir(AXIS_J2B, dir);
        motion_jog_start(1, dir, hz);
        motion_start_rate_timer(1);
    } else {
        int j = (axis == 0) ? AXIS_J1 : AXIS_J3;
        step_engine[j].active = true;
        step_engine[j].continuous = true;
        step_engine[j].dir = dir;
        step_engine[j].hz = hz;
        set_dir(j, dir);
        motion_jog_start(j, dir, hz);
        motion_start_rate_timer(j);
    }
    restore_interrupts(irq_save);
}

static void stop_axis(int axis) {
    // CRITICAL SECTION (see start_continuous).
    uint32_t irq_save = save_and_disable_interrupts();
    if (axis == 1) {
        if (step_engine[AXIS_J2A].active) {
            step_engine[AXIS_J2A].active = false;
            step_engine[AXIS_J2B].active = false;
            motion_jog_stop(1);
            cancel_repeating_timer(&run_timers[AXIS_J2A]);
        }
        shoulder_clear();
    } else {
        int j = (axis == 0) ? AXIS_J1 : AXIS_J3;
        if (step_engine[j].active) {
            step_engine[j].active = false;
            motion_jog_stop(j);
            cancel_repeating_timer(&run_timers[j]);
        }
        clear_step(j);
    }
    restore_interrupts(irq_save);
}

static void start_timed(int axis, int dir, uint32_t hz, int32_t steps) {
    stop_axis(axis);
    // Apply the config's direction-inversion bit (#CFG di=1).
    if (config_get()->axes[axis].dir_inverted) dir = -dir;

    // CRITICAL SECTION (see start_continuous).
    uint32_t irq_save = save_and_disable_interrupts();
    if (axis == 1) {
        set_dir(AXIS_J2A, dir);
        set_dir(AXIS_J2B, dir);
        step_engine[AXIS_J2A].active = true;
        step_engine[AXIS_J2A].continuous = false;
        step_engine[AXIS_J2A].dir = dir;
        step_engine[AXIS_J2A].hz = hz;
        step_engine[AXIS_J2A].steps_left = steps;
        step_engine[AXIS_J2B].active = true;
        add_repeating_timer_us((int64_t)(500000 / hz), shoulder_step_cb, NULL, &run_timers[AXIS_J2A]);
    } else {
        int j = (axis == 0) ? AXIS_J1 : AXIS_J3;
        set_dir(j, dir);
        step_engine[j].active = true;
        step_engine[j].continuous = false;
        step_engine[j].dir = dir;
        step_engine[j].hz = hz;
        step_engine[j].steps_left = steps;
        add_repeating_timer_us((int64_t)(500000 / hz), step_timer_cb, (void*)(intptr_t)j, &run_timers[j]);
    }
    restore_interrupts(irq_save);
}

static void halt_all(void) {
    // stop_axis()/clear_step() already use critical sections; motion_stop_all
    // and the homing stops mutate shared planner state so hold IRQs off across
    // the whole halt so the ISR never sees a partial stop.
    uint32_t irq_save = save_and_disable_interrupts();
    for (int i = 0; i < NJ; i++) {
        stop_axis(i);
        clear_step(i);
    }
    motion_stop_all();
    homing_stop(0); homing_stop(1); homing_stop(2);
    set_enable(false);
    restore_interrupts(irq_save);
}

// ── Motion planner ───────────────────────────────────────────────────
// (per-axis rate timers replace the old fixed 1kHz tick; see rate_timer_cb)

// Power-indicator heartbeat: toggles the onboard LED at 1Hz from a timer IRQ
// so a powered Pico always blinks, even while the SPI loop is blocked.
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

// Map logical axis (0=J1, 1=Shoulder, 2=J3) to physical joint indices
// For single axes: same index. For shoulder: both AXIS_J2A and AXIS_J2B.
static void apply_step_pulse(int logical_axis, bool step_active) {
    if (logical_axis == 1) {
        // Shoulder: pulse both J2A and J2B
        gpio_put(joints[AXIS_J2A].step_pin, step_active ? 1 : 0);
        gpio_put(joints[AXIS_J2B].step_pin, step_active ? 1 : 0);
    } else {
        int j = (logical_axis == 0) ? AXIS_J1 : AXIS_J3;
        gpio_put(joints[j].step_pin, step_active ? 1 : 0);
    }
}

static void apply_dir(int logical_axis, int dir) {
    // Apply the config's direction-inversion bit (set via #CFG di=1).
    bool invert = config_get()->axes[logical_axis].dir_inverted;
    if (invert) dir = -dir;
    if (logical_axis == 1) {
        set_dir(AXIS_J2A, dir);
        set_dir(AXIS_J2B, dir);
    } else {
        int j = (logical_axis == 0) ? AXIS_J1 : AXIS_J3;
        set_dir(j, dir);
    }
}

// ── Per-axis rate timers (hardware-timer stepping) ──────────────────
// Each logical axis has its own ONE-SHOT alarm that fires at
// marlin_edge_interval_us(rate) — the edge interval for the current rate.
// On each fire we toggle the step pin (rising edge = one step), recompute
// the rate from the block's elapsed time, and return the new interval so
// the alarm reschedules itself. One-shot alarms (not repeating timers) are
// the safe way to have a variable period: re-arming a repeating_timer from
// its own callback corrupts the timer pool (moves end early / jerk).

// Track which logical-axis step pins are currently HIGH (for step counting).
static bool step_high[3] = {false, false, false};

// Motion command queue — 1 slot per axis. A new jog arriving while the axis
// is moving is queued and executed when the current move finishes, so rapid
// jogs chain instead of restarting from zero.
typedef struct {
    int32_t delta;
    uint16_t speed;
    uint32_t accel;
    uint32_t decel;
    bool pending;
} motion_cmd_t;
static motion_cmd_t motion_queue[3];

// Staged (hold) moves for SYNC-start: a move frame with the hold bit set is
// set up but NOT started; 0x59 GO starts all staged axes together so a
// multi-axis #M moves all joints simultaneously instead of one after another.
typedef struct {
    int32_t delta;
    uint16_t speed;
    uint32_t accel;
    uint32_t decel;
    bool staged;
} staged_move_t;
static staged_move_t staged_moves[3];

// Start every staged axis at once (called by the 0x59 GO command).
static void start_all_staged(void) {
    for (int a = 0; a < 3; a++) {
        if (!staged_moves[a].staged) continue;
        staged_moves[a].staged = false;
        uint32_t irq_save = save_and_disable_interrupts();
        set_enable(true);
        motion_set_limits(a, staged_moves[a].speed, staged_moves[a].accel);
        motion_set_decel(a, staged_moves[a].decel);
        step_high[a] = false;
        motion_move(a, staged_moves[a].delta);
        motion_start_rate_timer(a);
        restore_interrupts(irq_save);
    }
}

static void exec_queued_motion(int axis) {
    if (!motion_queue[axis].pending) return;
    motion_cmd_t *cmd = &motion_queue[axis];
    cmd->pending = false;
    set_enable(true);
    motion_set_limits(axis, cmd->speed, cmd->accel);
    motion_set_decel(axis, cmd->decel);
    step_high[axis] = false;
    motion_move(axis, cmd->delta);
    motion_start_rate_timer(axis);
}

static alarm_id_t rate_alarms[3] = {-1, -1, -1};

static int64_t rate_alarm_cb(alarm_id_t id, void *user_data) {
    (void)id;
    int axis = (int)(intptr_t)user_data;

    // Continuous-jog mode: accelerate / cruise / rapid-decel to stop.
    if (motion_axis_jogging(axis)) {
        // Pass +1/-1 (not 0/1) so the direction-inversion bit works:
        // apply_dir inverts the SIGN; inverting 0 is a no-op (set_dir
        // maps dir>0, so 0 and -1 both drive the pin LOW). With 0/1 the
        // Invert toggle did nothing and the jog always spun one way.
        apply_dir(axis, motion_jog_dir(axis));

        uint32_t interval = motion_get_interval_us(axis);
        uint32_t rate = motion_jog_tick(axis, interval);
        interval = marlin_edge_interval_us(rate > 0 ? rate : MARLIN_MIN_STEP_RATE);

        // Toggle the step pin (shoulder pulses both J2A+J2B). Rising = step.
        bool rising = !step_high[axis];
        apply_step_pulse(axis, rising);
        step_high[axis] = rising;
        if (rising) motion_jog_edge(axis);

        if (!motion_axis_jogging(axis)) {
            // Jog finished (rapid decel complete): end the pulse, stop.
            apply_step_pulse(axis, false);
            step_high[axis] = false;
            rate_alarms[axis] = -1;
            step_engine[axis].active = false;
            if (axis == 1) step_engine[AXIS_J2B].active = false;
            if (!motion_any_moving() && !homing_any_active()) set_enable(false);
            return 0;
        }
        return (int64_t)interval;
    }

    if (!motion_is_moving(axis)) { rate_alarms[axis] = -1; return 0; }

    // Ensure direction is set (target may have been updated by a replace).
    int32_t pos = motion_get_position(axis);
    int32_t target = motion_get_target(axis);
    // +1/-1 (not 0/1) so dir_inverted inverts the sign correctly.
    apply_dir(axis, (target > pos) ? 1 : -1);

    // This interval elapsed; advance the block clock and get the new rate.
    uint32_t interval = motion_get_interval_us(axis);
    motion_advance(axis, interval);
    interval = motion_get_interval_us(axis);

    // Toggle the step pin. Rising edge = one step (dir pin is set above).
    bool rising = !step_high[axis];
    apply_step_pulse(axis, rising);
    step_high[axis] = rising;
    if (rising) {
        if (!motion_edge_fired(axis)) {
            // Move complete: end the pulse.
            apply_step_pulse(axis, false);
            step_high[axis] = false;
            rate_alarms[axis] = -1;
            // If another jog was queued while busy, start it now (the next
            // move accelerates from rest after this one finishes).
            if (motion_queue[axis].pending) {
                exec_queued_motion(axis);
                motion_start_rate_timer(axis);
            } else if (!motion_any_moving() && !homing_any_active()) {
                set_enable(false);
            }
            return 0;  // this alarm is done (a queued move re-arms itself)
        }
    }

    // Return the new edge interval — the alarm reschedules itself.
    return (int64_t)interval;
}

static void motion_start_rate_timer(int axis) {
    if (rate_alarms[axis] >= 0) cancel_alarm(rate_alarms[axis]);
    rate_alarms[axis] = -1;
    step_high[axis] = false;
    // Start with the initial-rate edge interval.
    uint32_t iv = motion_get_interval_us(axis);
    rate_alarms[axis] = add_alarm_in_us(iv, rate_alarm_cb,
                                        (void*)(intptr_t)axis, true);
}

static void motion_stop_rate_timer(int axis) {
    if (rate_alarms[axis] >= 0) cancel_alarm(rate_alarms[axis]);
    rate_alarms[axis] = -1;
    apply_step_pulse(axis, false);
    step_high[axis] = false;
}

// ── Homing main-loop updater (call from main while loop) ─────────────
static void homing_update_all(void) {
    bool lim[3] = {
        !gpio_get(LIMIT_J1),
        !gpio_get(LIMIT_J2),
        !gpio_get(LIMIT_J3),
    };
    for (int i = 0; i < 3; i++) {
        if (homing_get_state(i) != HOME_IDLE) {
            if (homing_update(i, lim[i])) {
                // Homing complete for axis i
            }
        }
    }
}

int main() {
    stdio_init_all();
    // Wait up to 2s for USB to attach so the banner isn't lost, but NEVER
    // block forever: the LED heartbeat must start even with no USB host, so
    // a powered Pico is always identifiable by its blink.
    for (int i = 0; i < 20 && !stdio_usb_connected(); i++) sleep_ms(100);
    sleep_ms(200);

    printf("SPI CMD Slave 1 - Arm Base (GPIO step) v%s (build %d)\n",
           FIRMWARE_VERSION_STR, FIRMWARE_BUILD_COUNT);

    for (int i = 0; i < NJ; i++) {
        gpio_init(joints[i].step_pin); gpio_set_dir(joints[i].step_pin, GPIO_OUT); gpio_put(joints[i].step_pin, 0);
        gpio_init(joints[i].dir_pin);  gpio_set_dir(joints[i].dir_pin, GPIO_OUT); gpio_put(joints[i].dir_pin, 0);
    }

    gpio_init(EN_PIN); gpio_set_dir(EN_PIN, GPIO_OUT); gpio_put(EN_PIN, 1);

    gpio_init(LIMIT_J1); gpio_set_dir(LIMIT_J1, GPIO_IN); gpio_pull_up(LIMIT_J1);
    gpio_init(LIMIT_J2); gpio_set_dir(LIMIT_J2, GPIO_IN); gpio_pull_up(LIMIT_J2);
    gpio_init(LIMIT_J3); gpio_set_dir(LIMIT_J3, GPIO_IN); gpio_pull_up(LIMIT_J3);

    led_heartbeat_init();

    enc_init_all();

    // Init new modules
    config_init();
    motion_init();
    // ── SANITIZE (not force) AXIS CONFIG ─────────────────────────────
    // The config is stored in flash and can hold garbage (seen: J3 max_speed
    // 1721, accel 4812 from a corrupt studio write), which made J3 crawl.
    // Only correct values that are clearly out of range — a VALID saved #CFG
    // write (steps_per_rev / gear_ratio / dir_inverted) must survive reboot,
    // so we don't clobber it unconditionally.
    // decel is clamped UP to 20000: a lower decel makes the move tail crawl
    // at ~20 steps/s (J2 visibly "keeps running slowly" after the move looks
    // done). A snappy decel ends motion crisply.
    {
        config_t *scfg = config_get_mut();
        for (int i = 0; i < 3; i++) {
            if (scfg->axes[i].max_speed < 1000 || scfg->axes[i].max_speed > 20000)
                scfg->axes[i].max_speed = 8000;
            if (scfg->axes[i].accel < 1000 || scfg->axes[i].accel > 200000)
                scfg->axes[i].accel = 6000;
            if (scfg->axes[i].decel < 20000 || scfg->axes[i].decel > 200000)
                scfg->axes[i].decel = 20000;
            if (scfg->axes[i].jog_accel < 1000 || scfg->axes[i].jog_accel > 200000)
                scfg->axes[i].jog_accel = 50000;
            if (scfg->axes[i].jog_decel < 1000 || scfg->axes[i].jog_decel > 400000)
                scfg->axes[i].jog_decel = 100000;
            if (scfg->axes[i].steps_per_rev < 200 || scfg->axes[i].steps_per_rev > 6400)
                scfg->axes[i].steps_per_rev = 3200;
            if (scfg->axes[i].gear_ratio < 100 || scfg->axes[i].gear_ratio > 10000)
                scfg->axes[i].gear_ratio = 100;
        }
    }
    // Apply config defaults to motion module
    const config_t *cfg = config_get();
    for (int i = 0; i < 3; i++) {
        motion_set_limits(i, cfg->axes[i].max_speed, cfg->axes[i].accel);
        motion_set_decel(i, cfg->axes[i].decel);
        motion_set_jog_accel(i, cfg->axes[i].jog_accel);
        motion_set_jog_decel(i, cfg->axes[i].jog_decel);
    }
    // Responsive jog: the config default jog_accel (1000) makes a 4000-step/s
    // jog take ~4s to ramp — it looks like the jog "doesn't work." Force a
    // snappy ramp so hold-to-jog responds immediately (same as slave 2).
    for (int i = 0; i < 3; i++) {
        if (cfg->axes[i].jog_accel < 20000) motion_set_jog_accel(i, 50000);
        if (cfg->axes[i].jog_decel < 20000) motion_set_jog_decel(i, 100000);
    }
    homing_init(3);
    homing_set_config(0, cfg->axes[0].max_speed / 2, 100, 200,
                      cfg->axes[0].home_invert_lim, cfg->axes[0].home_invert_dir);
    homing_set_config(1, cfg->axes[1].max_speed / 2, 100, 200,
                      cfg->axes[1].home_invert_lim, cfg->axes[1].home_invert_dir);
    homing_set_config(2, cfg->axes[2].max_speed / 2, 100, 200,
                      cfg->axes[2].home_invert_lim, cfg->axes[2].home_invert_dir);

    // BIT-BANG SPI slave on GP2/3/4/5 (slave 1's original pins) — same code
    // as slave 2, with FRAME RESYNC: measure the CS-high wait; >2ms = the
    // 50ms inter-frame gap -> reset to byte 0 so the next frame is aligned.
    // MISO tri-states when CS high (shared MISO wire with slave 2).
    gpio_init(PIN_SCK);  gpio_set_dir(PIN_SCK,  GPIO_IN);
    gpio_init(PIN_MOSI); gpio_set_dir(PIN_MOSI, GPIO_IN);
    gpio_init(PIN_CS);   gpio_set_dir(PIN_CS,   GPIO_IN);
    gpio_pull_up(PIN_CS);
    gpio_init(PIN_MISO); gpio_set_dir(PIN_MISO, GPIO_IN);
    gpio_put(PIN_MISO, 0);
    printf("[S1] BIT-BANG SPI slave ready (SCK=GP2 MOSI=GP4 MISO=GP3 CS=GP5)\n");

    static uint8_t tx_buf[BUF_LEN], rx_buf[BUF_LEN];
    memset(tx_buf, 0, BUF_LEN);
    tx_buf[0] = NITE_RSP_STALE;

    while (true) {
        // Stamp the response frame with CRC8 before transmitting
        tx_buf[NITE_SPI_CRC_IDX] = nite_spi_crc(tx_buf);

        // CS-PER-BYTE framing — the master toggles CS HIGH between every byte,
        // so the slave exchanges ONE byte per transfer.
        //
        // BIT-BANG: emulate SPI0 mode 0 (same as slave 2). Drive MISO while
        // SCK is LOW so the master samples a settled bit on the rising edge.
        // MISO tri-states when CS high (shared MISO wire with slave 2).
        // FRAME RESYNC: >2ms CS-high = inter-frame gap -> byte 0 of next frame.
        for (int i = 0; i < BUF_LEN; i++) {
            uint8_t out = tx_buf[i];
            uint8_t in  = 0;
            // Wait for CS active (low). RESYNC: >2ms high = frame gap.
            {
                uint64_t cs_high_start = time_us_64();
                while (gpio_get(PIN_CS)) {
                    gpio_set_dir(PIN_MISO, GPIO_IN);
                    gpio_put(PIN_MISO, 0);
                    tight_loop_contents();
                }
                if (time_us_64() - cs_high_start > 2000) {
                    i = -1;  // loop ++ makes this byte 0 of the next frame
                    continue;
                }
            }
            gpio_set_dir(PIN_MISO, GPIO_OUT);
            for (int bit = 7; bit >= 0; bit--) {
                while (gpio_get(PIN_SCK)) { tight_loop_contents(); }
                gpio_put(PIN_MISO, (out >> bit) & 1);
                while (!gpio_get(PIN_SCK)) { tight_loop_contents(); }
                if (gpio_get(PIN_MOSI)) in |= (1 << bit);
                while (gpio_get(PIN_SCK)) { tight_loop_contents(); }
            }
            rx_buf[i] = in;
            // Wait for CS deassert (high), then release MISO.
            {
                uint64_t cs_high_start = time_us_64();
                while (!gpio_get(PIN_CS)) { tight_loop_contents(); }
                if (time_us_64() - cs_high_start > 2000) {
                    i = -1;  // loop ++ makes this byte 0 of the next frame
                }
            }
            gpio_set_dir(PIN_MISO, GPIO_IN);
            gpio_put(PIN_MISO, 0);
        }

        // LED is driven by the 1Hz heartbeat timer (led_heartbeat_init), not here.

        // Update homing (non-blocking, runs every SPI cycle ~2ms)
        homing_update_all();

        // ── v2 frame dispatch ──────────────────────────────────────
        // Unpack the structured frame (sync 0xA5 + joint_id + opcode +
        // int32 payload + seq + CRC8). frame_v2_unpack verifies sync AND
        // CRC, so a corrupted command is NEVER executed — it is answered
        // with OP_FAULT(FAULT_BAD_FRAME) and the master retries.
        //
        // The master's read-response frame (all zeros) has sync 0x00 and
        // fails unpack; it is answered with STALE exactly like v1.
        spi_frame_t f;
        if (rx_buf[0] == 0) {
            // Response frame from master (zeros) — no CRC check needed.
            memset(tx_buf, 0, BUF_LEN);
            tx_buf[0] = NITE_RSP_STALE;
            continue;
        }
        if (!frame_v2_unpack(rx_buf, &f)) {
            // NO printf here — a USB-CDC print in the SPI loop blocks for ms,
            // making the slave miss the next CS edge and desync permanently.
            memset(tx_buf, 0, BUF_LEN);
            spi_frame_t bad = { .sync = FRAME_SYNC_BYTE, .joint_id = 0xFF,
                                .opcode = OP_FAULT, .payload = FAULT_BAD_FRAME, .seq = 0 };
            frame_v2_pack(&bad, tx_buf);
            continue;
        }

        // Echo the sequence byte so the master can correlate replies. The
        // seq also carries the moving flag on OP_MOTION_REPLY (bit 0).
        uint8_t seq = f.seq;

        memset(tx_buf, 0, BUF_LEN);

        switch (f.opcode) {
            case OP_PING: {
                // Liveness probe: reply OP_PONG (the master's heartbeat
                // checks g_reply.opcode == OP_PONG).
                v2_reply(tx_buf, OP_PONG, f.joint_id, 0, seq);
                break;
            }

            case OP_ENABLE: {
                // payload: 1 = enable, 0 = disable. joint_id: axis or 0xFF.
                uint8_t axis = f.joint_id;
                bool on = f.payload != 0;
                if (axis == 0xFF) {
                    if (!on) halt_all();
                    else set_enable(true);
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else if (axis <= 2) {
                    if (!on) stop_axis(axis);
                    set_enable(on);
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_TMC, seq);
                }
                break;
            }

            case OP_CONT_JOG: {
                // Continuous jog (hold-to-run): payload = V2_PAYLOAD_MOVE(speed, dir).
                uint8_t axis = f.joint_id & 0x7F;
                uint16_t speed = V2_MOVE_SPEED(f.payload);
                int8_t dir = (int8_t)V2_MOVE_STEPS(f.payload);
                if (axis > 2 || speed < 1) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                set_enable(true);
                start_continuous(axis, dir, speed);
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_HALT: {
                uint8_t axis = f.joint_id;
                if (axis == 0xFF) { halt_all(); v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq); }
                else if (axis <= 2) { stop_axis(axis); v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq); }
                else { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); }
                break;
            }

            case OP_LIMIT_READ: {
                uint8_t lim = 0;
                if (!gpio_get(LIMIT_J1)) lim |= (1 << 0);
                if (!gpio_get(LIMIT_J2)) lim |= (1 << 1);
                if (!gpio_get(LIMIT_J3)) lim |= (1 << 2);
                v2_reply(tx_buf, OP_LIMIT_REPLY, f.joint_id, lim, seq);
                break;
            }

            case OP_ENCODER_READ: {
                // joint_id = encoder index (0-3). Reply payload: low 16 bits =
                // raw 12-bit angle, bits 16-18 = MD/ML/MH status bits.
                uint8_t idx = f.joint_id;
                if (idx < NUM_ENCODERS) {
                    uint16_t raw = read_as5600(idx);
                    if (raw == 0xFFFF) {
                        v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_ENCODER, seq);
                    } else {
                        uint8_t status = read_as5600_status(idx);
                        int32_t payload = raw | (((status >> 3) & 1) << 16) |
                                          (((status >> 4) & 1) << 17) |
                                          (((status >> 5) & 1) << 18);
                        v2_reply(tx_buf, OP_ENCODER_REPLY, f.joint_id, payload, seq);
                    }
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_ENCODER, seq);
                }
                break;
            }

            // ── Motion profile commands ──────────────────────────────

            case OP_STEP_DELTA: {
                // Motion move with profile: payload = V2_PAYLOAD_MOVE(speed,
                // steps), steps signed (sign = direction). Accel/decel come
                // from config defaults. The HOLD flag (V2_JID_HOLD in joint_id)
                // stages the move for OP_GO instead of starting it now.
                uint8_t axis = f.joint_id & 0x7F;
                bool hold = (f.joint_id & V2_JID_HOLD) != 0;
                uint16_t speed = V2_MOVE_SPEED(f.payload);
                int16_t raw_steps = V2_MOVE_STEPS(f.payload);
#ifdef SPI_CMD_DEBUG
                printf("[OP_STEP_DELTA] axis=%d speed=%u steps=%d hold=%d seq=%u\n",
                       axis, speed, raw_steps, hold, seq);
#endif
                if (axis > 2 || speed < 1) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                {
                    uint32_t a = cfg->axes[axis].accel;
                    uint32_t d = cfg->axes[axis].decel;
                    // The robot executes raw moves: the studio computes the step
                    // count (gear ratio) and clamps speed to its motion-panel
                    // max. The robot only uses the commanded speed as the
                    // trapezoid cruise and accel/decel from config.
                    // Soft-limit clamp: keep the move within [lim_min, lim_max]
                    int32_t clamped = config_clamp_move(axis, raw_steps);
                    // If the move is fully clamped (at a soft limit), report it
                    // so the studio/console can see why nothing moved.
                    if (clamped == 0 && raw_steps != 0) {
                        v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_SOFT_LIMIT, seq);
                        break;
                    }
                    // NOTE: no printf here — a USB-CDC print in the SPI loop
                    // blocks for ms, making the slave miss the next CS edge and
                    // the master see a late/corrupt reply (responds too late).
                    if (motion_is_moving(axis)) {
                        // Axis busy — QUEUE the move (1 slot). It runs when the
                        // current move completes, so rapid jogs chain end-to-end
                        // (finish the first move, then accelerate into the next)
                        // instead of restarting from zero mid-move.
                        motion_queue[axis].delta = clamped;
                        motion_queue[axis].speed = speed;
                        motion_queue[axis].accel = a;
                        motion_queue[axis].decel = d;
                        motion_queue[axis].pending = true;
                    } else {
                        if (hold) {
                            // SYNC-start: stage the move (set up but don't
                            // start) — OP_GO starts it together with the other
                            // axes. In v2 the HOLD flag is a dedicated bit in
                            // joint_id, so it can never collide with the move
                            // sequence (the v1 a6/a7 collision is gone).
                            staged_moves[axis].delta = clamped;
                            staged_moves[axis].speed = speed;
                            staged_moves[axis].accel = a;
                            staged_moves[axis].decel = d;
                            staged_moves[axis].staged = true;
                        } else {
                            // CRITICAL SECTION: motion_move + step_high mutation
                            // must not interleave with the rate-timer ISR.
                            uint32_t irq_save = save_and_disable_interrupts();
                            set_enable(true);
                            motion_set_limits(axis, speed, a);
                            motion_set_decel(axis, d);
                            step_high[axis] = false;
                            motion_move(axis, clamped);
                            // Start (or re-arm) the per-axis rate timer.
                            motion_start_rate_timer(axis);
                            restore_interrupts(irq_save);
                        }
                    }
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                }
                break;
            }

            case OP_GO: {
                // SYNC GO: start all staged (hold) moves together. Sent by the
                // master after staging every axis of a multi-axis #M command.
                start_all_staged();
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_MOTION_STATUS: {
                // Query motion state: joint_id = axis (0-2). Reply
                // OP_MOTION_REPLY with payload = V2_PAYLOAD_STATUS(pos, spd)
                // and the moving flag in seq bit 0.
                uint8_t axis = f.joint_id;
                if (axis > 2) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                int32_t pos = motion_get_position(axis);
                uint32_t spd = motion_get_speed(axis);
                bool moving = motion_is_moving(axis) || homing_get_state(axis) != HOME_IDLE;
                uint8_t rseq = (uint8_t)(moving ? 1 : 0);
                v2_reply(tx_buf, OP_MOTION_REPLY, f.joint_id,
                         V2_PAYLOAD_STATUS(pos, spd), rseq);
                break;
            }

            case OP_HOME: {
                // Homing: joint_id = axis (0-2 or 0xFF), payload: 0=start,
                // 1=stop, 2=query.
                uint8_t axis = f.joint_id;
                uint8_t sub = (uint8_t)f.payload;
                if (axis > 2) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                if (sub == 0) {
                    set_enable(true);
                    homing_start(axis);
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else if (sub == 1) {
                    homing_stop(axis);
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else if (sub == 2) {
                    uint8_t h1 = homing_is_homed(axis) ? 1 : 0;
                    uint8_t h2 = (uint8_t)homing_get_state(axis);
                    v2_reply(tx_buf, OP_HOMING_REPLY, f.joint_id,
                             ((int32_t)h1) | ((int32_t)h2 << 8), seq);
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                }
                break;
            }

            case OP_CFG_READ: {
                // Config read: joint_id = V2_JID_CFG(field, axis).
                // Fields 0-4 = ext config, 5-7 = max_speed/accel/decel,
                // 8-10 = homing search/creep/backoff.
                uint8_t field = V2_CFG_FIELD(f.joint_id);
                uint8_t axis  = V2_CFG_AXIS(f.joint_id);
                if (axis > 2) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                const config_t *c = config_get();
                uint32_t val = 0;
                bool ok = true;
                switch (field) {
                    case CFG_FIELD_STEPS_REV:   val = c->axes[axis].steps_per_rev; break;
                    case CFG_FIELD_GEAR_RATIO:  val = c->axes[axis].gear_ratio; break;
                    case CFG_FIELD_DIR_INVERT:  val = c->axes[axis].dir_inverted ? 1 : 0; break;
                    case CFG_FIELD_JOG_DECEL:   val = c->axes[axis].jog_decel; break;
                    case CFG_FIELD_JOG_ACCEL:   val = c->axes[axis].jog_accel; break;
                    case CFG_FIELD_MAX_SPEED:   val = c->axes[axis].max_speed; break;
                    case CFG_FIELD_ACCEL:       val = c->axes[axis].accel; break;
                    case CFG_FIELD_DECEL:       val = c->axes[axis].decel; break;
                    case CFG_FIELD_HOME_SEARCH:
                    case CFG_FIELD_HOME_CREEP:
                    case CFG_FIELD_HOME_BACKOFF: {
                        uint32_t s = 0, cr = 0; int32_t bk = 0;
                        if (!homing_get_config(axis, &s, &cr, &bk)) ok = false;
                        else if (field == CFG_FIELD_HOME_SEARCH) val = s;
                        else if (field == CFG_FIELD_HOME_CREEP) val = cr;
                        else val = (uint32_t)bk;
                        break;
                    }
                    default: ok = false; break;
                }
                if (ok) {
                    v2_reply(tx_buf, OP_CFG_REPLY, f.joint_id, (int32_t)val, seq);
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                }
                break;
            }

            case OP_CFG_WRITE: {
                // Config write: joint_id = V2_JID_CFG(field, axis), payload = value.
                uint8_t field = V2_CFG_FIELD(f.joint_id);
                uint8_t axis  = V2_CFG_AXIS(f.joint_id);
                if (axis > 2) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                uint32_t val = (uint32_t)f.payload;
                config_t *c = config_get_mut();
                bool ok = true;
                switch (field) {
                    case CFG_FIELD_STEPS_REV:   c->axes[axis].steps_per_rev = val; break;
                    case CFG_FIELD_GEAR_RATIO:  c->axes[axis].gear_ratio = val; break;
                    case CFG_FIELD_DIR_INVERT:  c->axes[axis].dir_inverted = (val != 0); break;
                    case CFG_FIELD_JOG_DECEL:   c->axes[axis].jog_decel = val; motion_set_jog_decel(axis, val); break;
                    case CFG_FIELD_JOG_ACCEL:   c->axes[axis].jog_accel = val; motion_set_jog_accel(axis, val); break;
                    case CFG_FIELD_MAX_SPEED:   c->axes[axis].max_speed = val; break;
                    case CFG_FIELD_ACCEL:       c->axes[axis].accel = val; break;
                    case CFG_FIELD_DECEL:       c->axes[axis].decel = val; break;
                    case CFG_FIELD_HOME_SEARCH:
                    case CFG_FIELD_HOME_CREEP:
                    case CFG_FIELD_HOME_BACKOFF: {
                        uint32_t s = 0, cr = 0; int32_t bk = 0;
                        if (!homing_get_config(axis, &s, &cr, &bk)) { ok = false; break; }
                        if (field == CFG_FIELD_HOME_SEARCH) s = val;
                        else if (field == CFG_FIELD_HOME_CREEP) cr = val;
                        else bk = (int32_t)val;
                        homing_set_config(axis, s, cr, bk,
                                          c->axes[axis].home_invert_lim,
                                          c->axes[axis].home_invert_dir);
                        break;
                    }
                    default: ok = false; break;
                }
                if (!ok) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                // Apply motion profile limits live.
                motion_set_limits(axis, c->axes[axis].max_speed, c->axes[axis].accel);
                motion_set_decel(axis, c->axes[axis].decel);
                motion_set_jog_accel(axis, c->axes[axis].jog_accel);
                motion_set_jog_decel(axis, c->axes[axis].jog_decel);
                // Persist so #CFG survives a reboot (was RAM-only in v1).
                config_save();
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_CFG_SAVE: {
                // Config save to flash. The flash erase+program (~100ms,
                // interrupts disabled) runs HERE, inside the frame handler.
                // The master's v2_send_flash waits 400ms after the command
                // frame before reading the response, so the save completes
                // before the response frame — no frame is missed.
                if (config_save()) {
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                }
                break;
            }

            case OP_CFG_RESET: {
                // Config reset to defaults
                config_reset();
                motion_init();
                motion_set_limits(0, 2000, 500);
                motion_set_limits(1, 2000, 500);
                motion_set_limits(2, 2000, 500);
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            default:
                v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                break;
        }
    }
}
