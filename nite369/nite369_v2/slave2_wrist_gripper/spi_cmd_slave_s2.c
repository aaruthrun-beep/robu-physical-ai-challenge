/**
 * @file spi_cmd_slave_s2.c
 * @brief SPI Command Slave 2 — Wrist (J4, J5, J6, Gripper)
 *
 * Simple GPIO step generation. 4 independent axes.
 * CS-per-byte SPI. 50kHz. 2-frame protocol.
 * TMC2209 UART, WS2812B LED, AS5600 encoders via bit-bang I2C.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "tmc_uart.h"
#include "ws2812.h"
#include "motion_profile.h"
#include "homing.h"
#include "config_store.h"
#include "nite_spi_proto.h"
#include "firmware_version.h"

#define SPI_PORT    spi0
// BIT-BANG slave SPI on GP10/11/12/14 — the pin set PROVEN working by the
// fresh bb_spi test (master CS=GP14; GP13 is held high on the master and
// cannot drive low). SCK=10 (in), MISO=11 (out, tri-state when CS high),
// MOSI=12 (in), CS=14 (in, active low). Bit-bang so we get FRAME RESYNC.
#define PIN_SCK     10
#define PIN_MISO    11
#define PIN_MOSI    12
#define PIN_CS      14
// Frame length includes the CRC8 byte (see nite_spi_proto.h)
#define BUF_LEN     NITE_SPI_BUF_LEN

// Build a v2 reply frame (sync + joint_id echo + opcode + payload + seq +
// CRC) into tx. The master validates replies with frame_v2_unpack, so every
// reply MUST be a well-formed v2 frame.
static void v2_reply(uint8_t *tx, uint8_t opcode, uint8_t joint_id,
                     int32_t payload, uint8_t seq) {
    spi_frame_t r = { .sync = FRAME_SYNC_BYTE, .joint_id = joint_id,
                      .opcode = opcode, .payload = payload, .seq = seq };
    frame_v2_pack(&r, tx);
}

#define EN_PIN      28
#define LIMIT_J4    14
#define LIMIT_J5    15
#define LIMIT_J6    22

// Gripper servo (PWM on GP6, slice 3A — free pin; GP26/27 stay stepper axis 3).
// Pulse range 500-2500us at 50Hz, angle 0-180 deg sent as deci-degrees.
#define SERVO_GRIP_PIN      6
#define SERVO_GRIP_FREQ_HZ  50
#define SERVO_GRIP_MIN_US   500
#define SERVO_GRIP_MAX_US   2500
#define SERVO_GRIP_MIN_ANG  0
#define SERVO_GRIP_MAX_ANG  180

#define NJ 4

// AS5600 encoder pins — 3 encoders (J4, J5, J6; no gripper encoder).
// SPI now uses GP10/11/12/14, so encoders moved off those pins onto the
// freed GP2/3/4/5 (the old SPI pins): J4=GP2/3, J5=GP0/1, J6=GP4/5.
#define NUM_ENCODERS 3
static const uint ENC_SDA[NUM_ENCODERS] = {  2,  0,  4 };
static const uint ENC_SCL[NUM_ENCODERS] = {  3,  1,  5 };
#define AS5600_ADDR  0x36

typedef struct {
    uint step_pin;
    uint dir_pin;
} Joint;

static const Joint joints[NJ] = {
    // NOTE: J4 and J5 motors are physically SWAPPED on the pins.
    // GP16/17 is the J5 motor, GP18/19 is the J4 motor. The axis mapping
    // is corrected here so axis 0 (J4) drives GP18/19 and axis 1 (J5)
    // drives GP16/17.
    {18, 19},  // J4 (Forearm)      — motor on GP18/19
    {16, 17},  // J5 (Wrist Pitch)  — motor on GP16/17
    {20, 21},  // J6 (Wrist Roll)
    {26, 27},  // Gripper
};

// ── Gripper servo PWM ────────────────────────────────────────────────
// 50Hz PWM on GP6 (slice 3, free pin). PWM clock divided to 1MHz so the
// wrap = 20000 is a 20ms period and the level is the pulse width in us.
// angle_10 = deci-degrees (0-1800 -> 0.0-180.0 deg), clamped, then mapped
// linearly to SERVO_GRIP_MIN_US..MAX_US.
static void gripper_servo_set(uint16_t angle_10) {
    if (angle_10 > SERVO_GRIP_MAX_ANG * 10) angle_10 = SERVO_GRIP_MAX_ANG * 10;
    uint32_t us = SERVO_GRIP_MIN_US +
        (uint32_t)angle_10 * (SERVO_GRIP_MAX_US - SERVO_GRIP_MIN_US) /
        (SERVO_GRIP_MAX_ANG * 10 - SERVO_GRIP_MIN_ANG);
    pwm_set_gpio_level(SERVO_GRIP_PIN, us);
}

static void gripper_servo_init(void) {
    gpio_set_function(SERVO_GRIP_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_GRIP_PIN);
    pwm_set_clkdiv(slice, 125.0f);   // 125MHz / 125 = 1MHz PWM clock
    pwm_set_wrap(slice, 19999);      // 20000 counts = 20ms = 50Hz
    pwm_set_enabled(slice, true);
    gripper_servo_set(900);          // center (90 deg) at boot
}

// ── AS5600 Bit-Bang I2C (DISABLED) ──────────────────────────────────
// No encoders wired; the J5 encoder pins GP0/1 collide with the TMC
// PDN_UART (tmc_uart.h: TX=GP0, RX=GP1). Calling enc_init_all() re-claims
// GP0/1 as bit-bang I2C after tmc_uart_init(), which makes every TMC read
// fail. Compile the encoder code out so GP0/1 stay on the UART peripheral.
#if 0
// ── AS5600 Bit-Bang I2C ──────────────────────────────────────────────
static void bb_delay(void) { sleep_us(1); }

// Per-transaction deadline so a stuck encoder bus (clock stretching, SDA
// held low by a slave, etc.) can never hang the SPI loop forever.
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
// Clears a stuck-low SDA / hung slave so the next read starts clean.
static void bb_recover_bus(int idx) {
    gpio_set_dir(ENC_SDA[idx], GPIO_IN);
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

// Bounded read; never hangs. Returns raw 12-bit angle or 0xFFFF on error.
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
#endif // encoders disabled (see note above)

static bool en_active = false;

static struct {
    bool active;
    bool continuous;
    int dir;
    uint32_t hz;
    int32_t steps_left;
} step_engine[NJ];

static struct repeating_timer run_timers[NJ];

// Motion command buffer — queue 1 command per axis when busy
typedef struct {
    int32_t delta;
    uint16_t speed;
    uint32_t accel;
    uint32_t decel;
    bool pending;
} motion_cmd_t;
static motion_cmd_t motion_queue[NJ];

// Staged (hold) moves for SYNC-start: a move frame with the hold bit set is
// set up but NOT started; 0x59 GO starts all staged axes together so a
// multi-axis #M moves all joints simultaneously instead of one after another.
// Differential axes stage BOTH physical motors (1 and 2).
typedef struct {
    int32_t delta;
    uint16_t speed;
    uint32_t accel;
    uint32_t decel;
    bool staged;
} staged_move_t;
static staged_move_t staged_moves[NJ];

// Per-axis rate alarms + step-edge tracking (declared early: stop_axis and
// halt_all cancel alarms and need these).
static alarm_id_t rate_alarms[NJ] = {-1, -1, -1, -1};
static bool step_high[NJ] = {false, false, false, false};

static void set_enable(bool on);                    // fwd decl
static void motion_start_rate_timer(int axis);      // fwd decl

// Start every staged axis at once (called by the 0x59 GO command).
static void start_all_staged(void) {
    for (int a = 0; a < NJ; a++) {
        if (!staged_moves[a].staged) continue;
        staged_moves[a].staged = false;
        set_enable(true);
        motion_set_limits(a, staged_moves[a].speed, staged_moves[a].accel);
        motion_set_decel(a, staged_moves[a].decel);
        step_high[a] = false;
        motion_move(a, staged_moves[a].delta);
        motion_start_rate_timer(a);
    }
}

static void set_enable(bool on) {
    en_active = on;
    gpio_put(EN_PIN, on ? 0 : 1);
}

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

static bool run_timer_cb(struct repeating_timer *t) {
    int idx = (int)(intptr_t)t->user_data;
    gpio_put(joints[idx].step_pin, !gpio_get(joints[idx].step_pin));
    return true;
}

static void stop_axis(int axis) {
    if (axis < 0 || axis >= NJ) return;
    // Differential wrist: stopping virtual J5/J6 must stop BOTH physical
    // motors together (they share one virtual joint). The jog state lives on
    // the VIRTUAL axis (single alarm pulses both motors — see diff_jog_start),
    // so trigger the rapid-decel on the virtual axis.
    if (axis == 1 || axis == 2) {
        if (step_engine[axis].active || motion_axis_jogging(axis)) {
            step_engine[axis].active = false;
            motion_jog_stop(axis);   // rapid-decel ramp, then stop
            cancel_repeating_timer(&run_timers[axis]);
        }
        return;
    }
    // Continuous jog: enter rapid deceleration instead of an instant stop.
    if (step_engine[axis].active || motion_axis_jogging(axis)) {
        step_engine[axis].active = false;
        motion_jog_stop(axis);
        cancel_repeating_timer(&run_timers[axis]);
    }
    gpio_put(joints[axis].step_pin, 0);
}

static void motion_start_rate_timer(int axis);  // fwd decl (defined below)

static void start_continuous(int axis, int dir, uint32_t hz) {
    stop_axis(axis);
    // NOTE: the config's direction-inversion bit (#CFG di=1) is applied by
    // the rate alarm on every tick — NOT here. Applying it here too
    // double-inverts and forces the jog to only ever move one way.
    gpio_put(joints[axis].dir_pin, dir > 0);
    // Accelerate from rest to `hz` using the configured accel, run until
    // stopped (rapid decel at jog_decel). Uses the per-axis rate alarm.
    set_enable(true);
    motion_jog_start(axis, dir, hz);
    step_engine[axis].active = true;
    step_engine[axis].continuous = true;
    step_engine[axis].dir = dir;
    step_engine[axis].hz = hz;
    motion_start_rate_timer(axis);
}

static void start_timed(int axis, int dir, uint32_t hz, int32_t steps) {
    stop_axis(axis);
    if (config_get()->axes[axis].dir_inverted) dir = -dir;
    gpio_put(joints[axis].dir_pin, dir > 0);
    step_engine[axis].active = true;
    step_engine[axis].continuous = false;
    step_engine[axis].dir = dir;
    step_engine[axis].hz = hz;
    step_engine[axis].steps_left = steps;
    add_repeating_timer_us((int64_t)(500000 / hz), step_timer_cb, (void*)(intptr_t)axis, &run_timers[axis]);
}

static void halt_all(void) {
    for (int i = 0; i < NJ; i++) stop_axis(i);
    motion_stop_all();
    for (int i = 0; i < NJ; i++) motion_queue[i].pending = false;
    set_enable(false);
}

// ── Differential wrist (J5 pitch / J6 roll) ───────────────────────────
// J5 and J6 are NOT two independent motors: they are a single 2-motor
// differential (symmetric, 1:1 gears — no internal ratio). The two physical
// motors (axis 1 and axis 2) combine into two VIRTUAL joints:
//   pitch (J5) = (motorA + motorB) / 2   → both motors SAME direction
//   roll  (J6) = (motorA - motorB) / 2   → motors OPPOSITE directions
// The master/studio command the virtual joints (steps at the virtual axis).
// Because the differential is 1:1 the split is exact — no ratio constant:
//   motorA = +steps, motorB = +steps   (pitch)
//   motorA = +steps, motorB = -steps   (roll)
// The two motors MUST run simultaneously so the differential stays in sync.
//
// CRITICAL (why slave 1 works and slave 2 broke): the differential jog uses
// ONE rate alarm on the VIRTUAL axis that pulses BOTH motors in the same
// callback — exactly like slave 1's shoulder (one alarm pulses J2A+J2B).
// Two independent alarms (one per motor) doubled the IRQ rate, starving the
// bit-bang SPI loop and corrupting frames (the "S2=FAIL during J5/J6 jog").
#define DIFF_MOTOR_A 1   // physical axis 1
#define DIFF_MOTOR_B 2   // physical axis 2

// Returns +1 if the virtual axis should drive motor B in the same direction
// as motor A (pitch), -1 if opposite (roll). Virtual axis 1 = pitch, 2 = roll.
static inline int diff_b_sign(int virtual_axis) {
    return (virtual_axis == 1) ? 1 : -1;
}

// Finite move on a virtual differential axis: run BOTH motors at the same
// speed with the same accel/decel so the split stays synchronized.
static void diff_move(int virtual_axis, int32_t steps, uint32_t speed,
                      uint32_t accel, uint32_t decel) {
    if (virtual_axis < 1 || virtual_axis > 2) return;
    int bsign = diff_b_sign(virtual_axis);
    int32_t a_steps = steps;
    int32_t b_steps = (bsign > 0) ? steps : -steps;

    set_enable(true);
    motion_set_limits(DIFF_MOTOR_A, speed, accel);
    motion_set_decel(DIFF_MOTOR_A, decel);
    motion_set_limits(DIFF_MOTOR_B, speed, accel);
    motion_set_decel(DIFF_MOTOR_B, decel);
    step_high[DIFF_MOTOR_A] = false;
    step_high[DIFF_MOTOR_B] = false;
    motion_move(DIFF_MOTOR_A, a_steps);
    motion_move(DIFF_MOTOR_B, b_steps);
    motion_start_rate_timer(DIFF_MOTOR_A);
    motion_start_rate_timer(DIFF_MOTOR_B);
}

// Continuous jog on a virtual differential axis: both motors ramp together,
// same direction (pitch) or opposite (roll). Stopping both is handled by
// motion_jog_stop on each (via stop_axis).
//
// ONE rate alarm on the VIRTUAL axis pulses BOTH motors (see the CRITICAL
// note above) so the IRQ load is the same as a single-axis jog — the bit-bang
// SPI loop survives. The alarm's jog path detects the virtual axis and pulses
// the physical pair together.
static void diff_jog_start(int virtual_axis, int dir, uint32_t hz) {
    if (virtual_axis < 1 || virtual_axis > 2) return;
    int bsign = diff_b_sign(virtual_axis);
    int a_dir = (dir > 0) ? 1 : -1;
    int b_dir = (bsign > 0) ? a_dir : -a_dir;

    // CRITICAL: both motors MUST accelerate/decelerate identically or the
    // differential runs lopsided (one motor visibly slower). motion_jog_start
    // normally uses each axis's OWN jog_accel/jog_decel from config, which can
    // differ between axis 1 and axis 2. Force the VIRTUAL joint's values onto
    // BOTH physical motors so they ramp in lockstep.
    uint32_t jog_a = config_get()->axes[virtual_axis].jog_accel;
    uint32_t jog_d = config_get()->axes[virtual_axis].jog_decel;
    if (jog_a == 0) jog_a = config_get()->axes[virtual_axis].accel;
    if (jog_d == 0) jog_d = jog_a;
    motion_set_jog_accel(DIFF_MOTOR_A, jog_a);
    motion_set_jog_accel(DIFF_MOTOR_B, jog_a);
    motion_set_jog_decel(DIFF_MOTOR_A, jog_d);
    motion_set_jog_decel(DIFF_MOTOR_B, jog_d);

    set_enable(true);
    // Start the jog on the VIRTUAL axis (1 or 2). The rate alarm for the
    // virtual axis pulses BOTH physical motors in one callback.
    motion_jog_start(virtual_axis, dir, hz);
    step_engine[virtual_axis].active = true;
    step_engine[virtual_axis].continuous = true;
    step_engine[virtual_axis].dir = dir;
    step_engine[virtual_axis].hz = hz;
    // Also mark the physical motors jogging so stop_axis/any_axis_active see
    // them, but do NOT start separate alarms for them.
    step_engine[DIFF_MOTOR_A].active = true;
    step_engine[DIFF_MOTOR_A].continuous = true;
    step_engine[DIFF_MOTOR_A].dir = a_dir;
    step_engine[DIFF_MOTOR_B].active = true;
    step_engine[DIFF_MOTOR_B].continuous = true;
    step_engine[DIFF_MOTOR_B].dir = b_dir;
    // ONE alarm on the virtual axis — halves the IRQ load vs two alarms.
    motion_start_rate_timer(virtual_axis);
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

// ── Per-axis rate timers (hardware-timer stepping) ──────────────────
// Same design as slave 1: each axis's ONE-SHOT alarm fires at
// marlin_edge_interval_us(rate), toggles the step pin (rising edge = one
// step), recomputes the rate from the block's elapsed time, and returns the
// new interval so the alarm reschedules itself. One-shot alarms are the safe
// way to get a variable period (re-arming a repeating_timer from its own
// callback corrupts the timer pool).

static void exec_queued_motion(int axis);  // fwd decl (defined below)

static int64_t rate_alarm_cb(alarm_id_t id, void *user_data) {
    (void)id;
    int axis = (int)(intptr_t)user_data;

    // Continuous-jog mode: accelerate / cruise / rapid-decel to stop.
    if (motion_axis_jogging(axis)) {
        // Differential virtual axis (J5/J6): ONE alarm pulses BOTH motors in
        // the same callback (like slave 1's shoulder), halving the IRQ load.
        // The virtual axis carries the jog state; the physical motors just
        // get their step pins toggled together (opposite for roll).
        bool is_diff = (axis == 1 || axis == 2);
        // Set direction from the jog state. Use +1/-1 (NOT 0/1): the
        // dir_inverted bit flips the SIGN, and flipping 0 is a no-op (both
        // 0 and -1 drive the DIR pin LOW via `> 0`), so with 0/1 the Invert
        // toggle did nothing on the reverse jog.
        int jdir = motion_jog_dir(axis);
        if (config_get()->axes[axis].dir_inverted) jdir = -jdir;
        // Motor A direction: the virtual axis IS motor A for J5/J6 only when
        // the virtual axis number matches; for the pair we always drive the
        // physical motors 1 and 2 explicitly (see DIFF_MOTOR_A/B).
        int a_dir_pin = jdir > 0 ? 1 : 0;
        gpio_put(joints[DIFF_MOTOR_A].dir_pin, a_dir_pin);
        if (is_diff) {
            // Motor B opposite for roll (J6), same for pitch (J5).
            int b_pin = (jdir > 0) == (diff_b_sign(axis) > 0) ? 1 : 0;
            gpio_put(joints[DIFF_MOTOR_B].dir_pin, b_pin);
        } else {
            gpio_put(joints[axis].dir_pin, a_dir_pin);
        }

        uint32_t interval = motion_get_interval_us(axis);
        motion_jog_tick(axis, interval);
        // motion_jog_tick already computed timer_interval_us (marlin_edge_interval_us);
        // reuse it instead of re-dividing — saves a 32-bit software divide per edge.
        interval = motion_get_interval_us(axis);

        // Toggle the step pin(s). Rising edge = one step (position tracking).
        // For the differential ALWAYS pulse BOTH physical motors 1 and 2 —
        // never joints[axis] (the virtual axis may BE motor B, which would
        // double-pulse one motor and never pulse the other: the "J6 only moves
        // the last motor" bug).
        // The rising reference must come from the axis being driven: for a
        // non-diff axis (J4 = 0, gripper = 3) use step_high[axis], NOT
        // step_high[DIFF_MOTOR_A] — reading the wrong axis's state made the
        // step pin stick HIGH (motor energized but never stepping: "axis 4
        // holds but doesn't move").
        int ref = is_diff ? DIFF_MOTOR_A : axis;
        bool rising = !step_high[ref];
        if (is_diff) {
            gpio_put(joints[DIFF_MOTOR_A].step_pin, rising ? 1 : 0);
            step_high[DIFF_MOTOR_A] = rising;
            gpio_put(joints[DIFF_MOTOR_B].step_pin, rising ? 1 : 0);
            step_high[DIFF_MOTOR_B] = rising;
        } else {
            gpio_put(joints[axis].step_pin, rising ? 1 : 0);
            step_high[axis] = rising;
        }
        if (rising) {
            if (is_diff) {
                motion_jog_edge(DIFF_MOTOR_A);
                motion_jog_edge(DIFF_MOTOR_B);
            } else {
                motion_jog_edge(axis);  // count the step in the jog position
            }
        }

        if (!motion_axis_jogging(axis)) {
            // Jog finished (rapid decel complete): end the pulse, stop.
            if (is_diff) {
                gpio_put(joints[DIFF_MOTOR_A].step_pin, 0);
                gpio_put(joints[DIFF_MOTOR_B].step_pin, 0);
                step_high[DIFF_MOTOR_A] = false;
                step_high[DIFF_MOTOR_B] = false;
                step_engine[DIFF_MOTOR_A].active = false;
                step_engine[DIFF_MOTOR_B].active = false;
            } else {
                gpio_put(joints[axis].step_pin, 0);
                step_high[axis] = false;
            }
            rate_alarms[axis] = -1;
            step_engine[axis].active = false;
            if (!motion_any_moving() && !homing_any_active()) set_enable(false);
            return 0;
        }
        return (int64_t)interval;
    }

    if (!motion_is_moving(axis)) { rate_alarms[axis] = -1; return 0; }

    // Ensure direction is set (target may have been updated by a replace),
    // applying the config's direction-inversion bit (#CFG di=1).
    int32_t pos = motion_get_position(axis);
    int32_t target = motion_get_target(axis);
    // +1/-1 (not 0/1) so dir_inverted flips the sign; flipping 0 is a no-op.
    int dir = (target > pos) ? 1 : -1;
    if (config_get()->axes[axis].dir_inverted) dir = -dir;
    gpio_put(joints[axis].dir_pin, dir > 0);

    // This interval elapsed; advance the block clock and get the new rate.
    uint32_t interval = motion_get_interval_us(axis);
    motion_advance(axis, interval);
    interval = motion_get_interval_us(axis);

    // Toggle the step pin. Rising edge = one step.
    bool rising = !step_high[axis];
    gpio_put(joints[axis].step_pin, rising ? 1 : 0);
    step_high[axis] = rising;
    if (rising) {
        if (!motion_edge_fired(axis)) {
            // Move complete: end the pulse.
            gpio_put(joints[axis].step_pin, 0);
            step_high[axis] = false;
            rate_alarms[axis] = -1;
            // If another move was queued while busy, start it now (the next
            // move accelerates from rest after this one finishes).
            if (motion_queue[axis].pending) {
                exec_queued_motion(axis);
                motion_start_rate_timer(axis);
                // Differential wrist: a queued move lands on BOTH motors 1/2.
                // If this is one of them and the OTHER also has a queued move,
                // start it NOW so the pair stays in sync (otherwise the second
                // motor idles and the differential runs lopsided / appears
                // broken on rapid successive J5/J6 commands).
                if ((axis == DIFF_MOTOR_A || axis == DIFF_MOTOR_B) &&
                    motion_queue[DIFF_MOTOR_A].pending &&
                    motion_queue[DIFF_MOTOR_B].pending) {
                    exec_queued_motion(axis == DIFF_MOTOR_A ? DIFF_MOTOR_B : DIFF_MOTOR_A);
                    motion_start_rate_timer(axis == DIFF_MOTOR_A ? DIFF_MOTOR_B : DIFF_MOTOR_A);
                }
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
    uint32_t iv = motion_get_interval_us(axis);
    rate_alarms[axis] = add_alarm_in_us(iv, rate_alarm_cb,
                                        (void*)(intptr_t)axis, true);
}

// Execute a queued motion command
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

static void homing_update_all(void) {
    bool lim[3] = {
        !gpio_get(LIMIT_J4),
        !gpio_get(LIMIT_J5),
        !gpio_get(LIMIT_J6),
    };
    for (int i = 0; i < 3; i++) {
        if (homing_get_state(i) != HOME_IDLE) {
            homing_update(i, lim[i]);
        }
    }
}

int main() {
    stdio_init_all();
    // 2s boot delay: lets the serial monitor attach before boot output and
    // lets both master + slave power up in sync (master also delays 2s).
    sleep_ms(2000);

    printf("SPI CMD Slave 2 - Wrist (GPIO step) v%s (build %d)\n",
           FIRMWARE_VERSION_STR, FIRMWARE_BUILD_COUNT);

    for (int i = 0; i < NJ; i++) {
        gpio_init(joints[i].step_pin); gpio_set_dir(joints[i].step_pin, GPIO_OUT); gpio_put(joints[i].step_pin, 0);
        gpio_init(joints[i].dir_pin);  gpio_set_dir(joints[i].dir_pin, GPIO_OUT); gpio_put(joints[i].dir_pin, 0);
    }
    printf("[S2] step/dir pins ready\n");

    gripper_servo_init();
    printf("[S2] gripper servo PWM ready (GP%d, 50Hz, 500-2500us)\n", SERVO_GRIP_PIN);

    gpio_init(EN_PIN); gpio_set_dir(EN_PIN, GPIO_OUT); gpio_put(EN_PIN, 1);

    gpio_init(LIMIT_J4); gpio_set_dir(LIMIT_J4, GPIO_IN); gpio_pull_up(LIMIT_J4);
    gpio_init(LIMIT_J5); gpio_set_dir(LIMIT_J5, GPIO_IN); gpio_pull_up(LIMIT_J5);
    gpio_init(LIMIT_J6); gpio_set_dir(LIMIT_J6, GPIO_IN); gpio_pull_up(LIMIT_J6);

    led_heartbeat_init();
    printf("[S2] LED heartbeat started (blinking = powered/running)\n");

    // BIT-BANG SPI slave on GP10/11/12/14 (proven working by bb_spi test).
    // SCK=GP10 (in), MOSI=GP12 (in), CS=GP14 (in, active low), MISO=GP11.
    // MISO starts TRI-STATED (input): it is shared with slave 1's MISO, so it
    // must be high-Z whenever CS is high. The loop drives it only while CS is
    // low (selected). Plain GPIO drive — identical to slave 1 (which pings
    // clean); 12mA/fast-slew was removed because the fast edges rang on the
    // shared MISO wire and caused the master to sample the reply shifted
    // (the persistent "F0 00 E6 4F" rotation).
    gpio_init(PIN_SCK);  gpio_set_dir(PIN_SCK,  GPIO_IN);
    gpio_init(PIN_MOSI); gpio_set_dir(PIN_MOSI, GPIO_IN);
    gpio_init(PIN_CS);   gpio_set_dir(PIN_CS,   GPIO_IN);
    // Pull-up on CS: the master's CS idles high; with the shared SCK/MOSI
    // bus toggling during the OTHER slave's pings, a floating CS input can
    // glitch low from crosstalk and trigger the bit-bang mid-frame (observed
    // as "BAD FRAME: 50 40 78 50..."). The pull-up holds CS deasserted.
    gpio_pull_up(PIN_CS);
    gpio_init(PIN_MISO); gpio_set_dir(PIN_MISO, GPIO_IN);
    gpio_put(PIN_MISO, 0);
    printf("[S2] BIT-BANG SPI slave ready (SCK=GP10 MOSI=GP12 MISO=GP11 CS=GP14)\n");

    tmc_uart_init();
    printf("[S2] TMC UART init done\n");

    // ── Differential motor matching (J5 motor A = addr 1, J6 motor B = addr 2) ──
    // The two differential motors MUST run at the SAME microstep or they spin
    // at different speeds for the same step count (the wrist goes lopsided).
    // Force all 4 drivers to 1/8 (MRES=3) at boot so a failed studio #TW write
    // or a jumper mismatch can't leave them different — and so the motors are
    // NOT stuck at full-step (which makes J4/J6 crawl at 1/8 the commanded
    // speed). Bounded by the per-byte 20ms echo timeout — worst case ~1s
    // delay, never a hang.
    //   GCONF    = 0x00000084  SpreadCycle + MSTEP_REG_SELECT (UART controls microstep)
    //   CHOPCONF = 0x13010253  1/8 microstep (MRES=3) + 256x interpolation
    //   IHOLD    = 0x00040810  IRUN=16 (~1.0A), IHOLD=8 (50%)
    for (int d = 0; d < 4; d++) {
        tmc_write_register(d, TMC_REG_GCONF,      0x00000084u);
        tmc_write_register(d, TMC_REG_CHOPCONF,   0x13010253u);
        tmc_write_register(d, TMC_REG_IHOLD_IRUN, 0x00040810u);
    }
    printf("[S2] Differential drivers matched: 1/8 microstep\n");

    // WS2812 init DELAYED until a command requests it: ws2812_init() uses
    // pio_claim_unused_sm(pio0, true) which PANICS (hangs) if PIO0 has no free
    // state machine — observed as a boot hang right after "TMC UART init done"
    // with MISO idle-high (master saw F0 FF FF on every ping). The slave must
    // reach the SPI loop unconditionally, so do not touch PIO/WS2812 at boot.
    // The heartbeat LED already indicates power/running.
    printf("[S2] WS2812 deferred (skip PIO at boot to avoid hang)\n");

    // Encoders DISABLED — no encoders wired; GP0/1 must stay on the TMC
    // UART (see the #if 0 block above). enc_init_all() was re-claiming
    // GP0/1 as bit-bang I2C and killing every TMC read.

    // Init new modules
    config_init();
    motion_init();
    // ── WRIST FIXED CONFIG ──────────────────────────────────────────
    // The config table is SHARED with slave 1 (axes 0/1/2 = J1/J2/J3 there),
    // so the stored values for slave 2's wrist (axes 0/1/2 = J4/J5/J6 here)
    // can be garbage or belong to slave 1. Force the wrist to known-good
    // mechanical values EVERY boot — the studio cannot break it:
    //   axis 0 = J4 (forearm)  1:16 gearbox, 1/8 step -> 1600*16/360 = 71.1 steps/deg
    //   axis 1 = J5 motor A    1:25 gearbox, 1/8 step -> 1600*25/360 = 111.1 steps/deg
    //   axis 2 = J6 motor B    1:25 gearbox, 1/8 step -> 111.1 steps/deg (identical to J5)
    //   axis 3 = gripper       1:1, 1/8 step
    config_t *wcfg = config_get_mut();
    for (int i = 0; i < 4; i++) {
        // 1/8 microstepping (200 x 8 = 1600 steps/rev) — full step stalls the
        // motors at high speed (they crawled). 1/8 gives smooth fast motion.
        wcfg->axes[i].steps_per_rev = 1600;
        wcfg->axes[i].max_speed     = 8000;
        wcfg->axes[i].accel         = 50000;
        wcfg->axes[i].decel         = 50000;
        wcfg->axes[i].jog_accel     = 50000;
        wcfg->axes[i].jog_decel     = 100000;
    }
    wcfg->axes[0].gear_ratio = 1600;   // J4: 1:16
    wcfg->axes[1].gear_ratio = 2500;   // J5: 1:25
    wcfg->axes[2].gear_ratio = 2500;   // J6: 1:25 (identical)
    wcfg->axes[3].gear_ratio = 100;    // gripper: 1:1
    // Apply config defaults to motion module
    const config_t *cfg = config_get();
    for (int i = 0; i < NJ; i++) {
        motion_set_limits(i, cfg->axes[i].max_speed, cfg->axes[i].accel);
        motion_set_decel(i, cfg->axes[i].decel);
        motion_set_jog_accel(i, cfg->axes[i].jog_accel);
        motion_set_jog_decel(i, cfg->axes[i].jog_decel);
    }
    homing_init(3);
    homing_set_config(0, cfg->axes[0].max_speed / 2, 100, 200,
                      cfg->axes[0].home_invert_lim, cfg->axes[0].home_invert_dir);
    homing_set_config(1, cfg->axes[1].max_speed / 2, 100, 200,
                      cfg->axes[1].home_invert_lim, cfg->axes[1].home_invert_dir);
    homing_set_config(2, cfg->axes[2].max_speed / 2, 100, 200,
                      cfg->axes[2].home_invert_lim, cfg->axes[2].home_invert_dir);
    printf("[S2] Config + motion + homing init done\n");

    static uint8_t tx_buf[BUF_LEN], rx_buf[BUF_LEN];
    memset(tx_buf, 0, BUF_LEN);
    tx_buf[0] = NITE_RSP_STALE;

    printf("[S2] Entering SPI command loop — waiting for master...\n");
    while (true) {
        // Stamp the response frame with CRC8 before transmitting
        tx_buf[NITE_SPI_CRC_IDX] = nite_spi_crc(tx_buf);

        // CS-PER-BYTE framing — validated on hardware (docs/spi_debug_issues.md
        // Issue 8, reference test/spi_raw_test.c). The RP2040 SPI0 slave only
        // reloads its TX shift register from the TX FIFO when CS re-asserts
        // (HIGH->LOW). The master toggles CS HIGH between every byte, so the
        // slave must exchange ONE byte per transfer.
        //
        // FRAME RESYNC: the master holds CS high for 50ms between frames but
        // only ~500us between bytes. If the slave ever loses byte alignment
        // (e.g. from a missed edge at boot), it would stay offset forever and
        // never see the ping as byte 0 -> replies STALE forever (observed:
        // master read "00 00 E6 4F..." = stale frame rotated 3 bytes). So we
        // measure the CS-high wait: if it exceeds ~2ms it's the inter-frame
        // gap -> reset the byte counter to 0 and realign to the next frame.
        // This loop is IDENTICAL to slave 1's (which pings clean): unbounded
        // SCK waits, CS-deassert resync at 2ms, NO printf inside the SPI loop
        // (a USB-CDC print blocks for ms and desyncs the bit-bang permanently).
        for (int i = 0; i < BUF_LEN; i++) {
            uint8_t out = tx_buf[i];
            uint8_t in  = 0;
            // Wait for CS active (low) — the master toggles CS per byte.
            // RESYNC: if CS stays high >2ms it's the inter-frame gap -> this
            // byte is byte 0 of the next frame.
            {
                uint64_t cs_high_start = time_us_64();
                while (gpio_get(PIN_CS)) {
                    // Not selected: MISO must be high-Z so we don't fight the
                    // other slave on the shared MISO line.
                    gpio_set_dir(PIN_MISO, GPIO_IN);
                    gpio_put(PIN_MISO, 0);
                    tight_loop_contents();
                }
                if (time_us_64() - cs_high_start > 2000) {
                    i = -1;  // loop ++ makes this byte 0 of the next frame
                    continue;  // re-run the byte-0 entry wait fresh
                }
            }
            // Selected: drive MISO (output).
            gpio_set_dir(PIN_MISO, GPIO_OUT);
            for (int bit = 7; bit >= 0; bit--) {
                // Wait for SCK low (start of bit cell). Drive MISO HERE, while
                // SCK is low, so the master samples a settled value on the
                // rising edge (SPI mode 0).
                while (gpio_get(PIN_SCK)) { tight_loop_contents(); }
                gpio_put(PIN_MISO, (out >> bit) & 1);
                // Wait for SCK rising edge (master drives MOSI just before)
                while (!gpio_get(PIN_SCK)) { tight_loop_contents(); }
                // Sample MOSI now that SCK is high
                if (gpio_get(PIN_MOSI)) in |= (1 << bit);
                // Wait for SCK low again
                while (gpio_get(PIN_SCK)) { tight_loop_contents(); }
            }
            rx_buf[i] = in;
            // Wait for CS deassert (high), then release MISO again.
            {
                uint64_t cs_high_start = time_us_64();
                while (!gpio_get(PIN_CS)) { tight_loop_contents(); }
                // If CS stays high >2ms it's the inter-frame gap: realign to
                // byte 0 of the next frame.
                if (time_us_64() - cs_high_start > 2000) {
                    i = -1;  // loop ++ makes this byte 0 of the next frame
                }
            }
            gpio_set_dir(PIN_MISO, GPIO_IN);
            gpio_put(PIN_MISO, 0);
        }

        // LED is driven by the 1Hz heartbeat timer (led_heartbeat_init), not here.

        // Update homing
#ifndef SPI_ONLY_TEST
        homing_update_all();
#endif

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
            // making the slave miss the next CS edge and desync permanently
            // (same as slave 1).
            memset(tx_buf, 0, BUF_LEN);
            spi_frame_t bad = { .sync = FRAME_SYNC_BYTE, .joint_id = 0xFF,
                                .opcode = OP_FAULT, .payload = FAULT_BAD_FRAME, .seq = 0 };
            frame_v2_pack(&bad, tx_buf);
            continue;
        }

        // Echo the sequence byte so the master can correlate replies.
        uint8_t seq = f.seq;

        memset(tx_buf, 0, BUF_LEN);

        // NOTE: NO per-command printf here. The master waits only 15ms between
        // frame 1 (command) and frame 2 (read response); a USB-CDC printf
        // blocks the SPI loop for ms and makes the slave miss frame 2's CS
        // edge, so the reply goes out one frame late and the master reads
        // stale zeros. Slave 1 (which works) does not log every command.
        // Debug prints live inside the individual handlers only.

#ifdef SPI_ONLY_TEST
        // SPI-ONLY TEST: no command handlers — every frame just gets a clean
        // OK reply. Proves the SPI link (receive + transmit) works end to end
        // without TMC/encoder/config/motion/homing involved.
        v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
#else
        switch (f.opcode) {
            case OP_PING: {
                // Liveness probe: reply OP_PONG.
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
                    printf("[S2] EN all -> %s\n", on ? "ON" : "OFF (halt)");
                } else if (axis < NJ) {
                    if (!on) stop_axis(axis);
                    set_enable(on);
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                    printf("[S2] EN axis %u -> %s\n", axis, on ? "ON" : "OFF");
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                    printf("[S2] EN invalid axis %u\n", axis);
                }
                break;
            }

            case OP_CONT_JOG: {
                // Continuous jog (hold-to-run): payload = V2_PAYLOAD_MOVE(speed, dir).
                uint8_t axis = f.joint_id & 0x7F;
                uint16_t speed = V2_MOVE_SPEED(f.payload);
                int8_t dir = (int8_t)V2_MOVE_STEPS(f.payload);
                if (axis >= NJ || speed < 1) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                set_enable(true);
                if (axis == 1 || axis == 2) {
                    // Differential wrist jog: drive both motors together.
                    diff_jog_start(axis, dir, speed);
                } else {
                    start_continuous(axis, dir, speed);
                }
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_HALT: {
                uint8_t axis = f.joint_id;
                if (axis == 0xFF) { halt_all(); v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq); printf("[S2] HALT all\n"); }
                else if (axis < NJ) { stop_axis(axis); v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq); printf("[S2] HALT axis %u\n", axis); }
                else { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); }
                break;
            }

            case OP_LIMIT_READ: {
                uint8_t lim = 0;
                if (!gpio_get(LIMIT_J4)) lim |= (1 << 0);
                if (!gpio_get(LIMIT_J5)) lim |= (1 << 1);
                if (!gpio_get(LIMIT_J6)) lim |= (1 << 2);
                v2_reply(tx_buf, OP_LIMIT_REPLY, f.joint_id, lim, seq);
                break;
            }

            case OP_TMC_READ: {
                // joint_id = TMC address, payload = register.
                uint8_t addr = f.joint_id;
                uint8_t reg = (uint8_t)(f.payload & 0xFF);
                uint32_t val;
                if (tmc_read_register(addr, reg, &val)) {
                    v2_reply(tx_buf, OP_TMC_REPLY, f.joint_id, (int32_t)val, seq);
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_TMC, seq);
                }
                break;
            }

            case OP_TMC_WRITE: {
                // joint_id = TMC address, payload = value, seq = register.
                uint8_t addr = f.joint_id;
                uint8_t reg = seq;
                uint32_t val = (uint32_t)f.payload;
                tmc_write_register(addr, reg, val);
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_LED: {
                // payload = V2_PAYLOAD_LED(r, g, b, mode).
                uint8_t r = (uint8_t)(((uint32_t)f.payload >> 24) & 0xFF);
                uint8_t g = (uint8_t)(((uint32_t)f.payload >> 16) & 0xFF);
                uint8_t b = (uint8_t)(((uint32_t)f.payload >> 8) & 0xFF);
                uint8_t mode = (uint8_t)((uint32_t)f.payload & 0xFF);
                if (mode == 0xFF) {
                    ws2812_clear();
                } else if (mode == 0x00) {
                    ws2812_set_all(r, g, b);
                    ws2812_show();
                } else {
                    // mode = pixel index (0-7)
                    if (mode < WS2812_NUM_LEDS) {
                        ws2812_set_pixel(mode, r, g, b);
                        ws2812_show();
                    }
                }
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_GRIPPER: {
                // payload = angle in deci-degrees (0-1800 -> 0-180deg),
                // mapped to 500-2500us on GP6. 0xFFFF = disable PWM (release).
                uint16_t angle_10 = (uint16_t)((uint32_t)f.payload & 0xFFFF);
                if (angle_10 == 0xFFFF) {
                    pwm_set_enabled(pwm_gpio_to_slice_num(SERVO_GRIP_PIN), false);
                } else {
                    gripper_servo_set(angle_10);
                }
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            case OP_ENCODER_READ: {
                // DISABLED (no encoders wired; bit-bang code compiled out so
                // GP0/1 stay on the TMC UART). Reply OP_FAULT.
                v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_ENCODER, seq);
                break;
            }

            // ── Motion profile commands (Slave 2: 4 axes) ───────────

            case OP_STEP_DELTA: {
                // Motion move: payload = V2_PAYLOAD_MOVE(speed, steps), steps
                // signed (sign = direction). Axes 1/2 are the differential
                // wrist (virtual J5/J6): a single command drives BOTH physical
                // motors (see diff_move). The HOLD flag (V2_JID_HOLD in
                // joint_id) stages the move for OP_GO instead of starting now.
                uint8_t axis = f.joint_id & 0x7F;
                bool hold = (f.joint_id & V2_JID_HOLD) != 0;
                uint16_t speed = V2_MOVE_SPEED(f.payload);
                int16_t raw_steps = V2_MOVE_STEPS(f.payload);
                if (axis >= NJ || speed < 1) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); printf("[S2] MV reject axis=%u speed=%u\n", axis, speed); break; }
                uint32_t a = config_get()->axes[axis].accel;
                uint32_t d = config_get()->axes[axis].decel;

                if (axis == 1 || axis == 2) {
                    // Differential wrist — run both motors simultaneously.
                    if (motion_is_moving(DIFF_MOTOR_A) || motion_is_moving(DIFF_MOTOR_B)) {
                        // Queue on both motors (same params) so a busy wrist
                        // still accepts the next virtual move.
                        for (int m = 1; m <= 2; m++) {
                            motion_queue[m].delta = (m == DIFF_MOTOR_A) ? raw_steps
                                                     : (diff_b_sign(axis) > 0 ? raw_steps : -raw_steps);
                            motion_queue[m].speed = speed;
                            motion_queue[m].accel = a;
                            motion_queue[m].decel = d;
                            motion_queue[m].pending = true;
                        }
                        printf("[S2] MV diff axis %u busy — queued %d steps\n", axis, raw_steps);
                    } else if (hold) {
                        // SYNC-start: stage BOTH physical motors. In v2 the
                        // HOLD flag is a dedicated joint_id bit, so it can
                        // never collide with the move sequence.
                        for (int m = 1; m <= 2; m++) {
                            staged_moves[m].delta = (m == DIFF_MOTOR_A) ? raw_steps
                                                     : (diff_b_sign(axis) > 0 ? raw_steps : -raw_steps);
                            staged_moves[m].speed = speed;
                            staged_moves[m].accel = a;
                            staged_moves[m].decel = d;
                            staged_moves[m].staged = true;
                        }
                    } else {
                        diff_move(axis, raw_steps, speed, a, d);
                        printf("[S2] MV diff axis %u -> %d steps @ %u/s\n", axis, raw_steps, speed);
                    }
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                    break;
                }

                // Robot executes raw moves: studio computes steps (gear ratio)
                // and clamps speed. Robot uses commanded speed as cruise and
                // accel/decel from config.
                if (motion_is_moving(axis)) {
                    // Axis busy — queue command (max 1 deep)
                    motion_queue[axis].delta = raw_steps;
                    motion_queue[axis].speed = speed;
                    motion_queue[axis].accel = a;
                    motion_queue[axis].decel = d;
                    motion_queue[axis].pending = true;
                    printf("[S2] MV axis %u busy — queued %d steps\n", axis, raw_steps);
                } else {
                    // Soft-limit clamp: keep the move within [lim_min, lim_max]
                    int32_t clamped = config_clamp_move(axis, raw_steps);
                    if (clamped == 0 && raw_steps != 0) {
                        // Fully clamped (at a soft limit) — report it so the
                        // studio sees >ER:LIMIT instead of a silent no-move.
                        v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_SOFT_LIMIT, seq);
                        break;
                    }
                    if (clamped != raw_steps) {
                        printf("[S2] MV axis %u soft-limit: %d -> %d steps\n", axis, raw_steps, clamped);
                    }
                    if (hold) {
                        // SYNC-start: stage, don't start yet. In v2 the HOLD
                        // flag is a dedicated joint_id bit (no a6/a7 collision).
                        staged_moves[axis].delta = clamped;
                        staged_moves[axis].speed = speed;
                        staged_moves[axis].accel = a;
                        staged_moves[axis].decel = d;
                        staged_moves[axis].staged = true;
                    } else {
                        set_enable(true);
                        motion_set_limits(axis, speed, a);
                        motion_set_decel(axis, d);
                        step_high[axis] = false;
                        motion_move(axis, clamped);
                        // Start (or re-arm) the per-axis rate timer.
                        motion_start_rate_timer(axis);
                        printf("[S2] MV axis %u -> %d steps @ %u/s\n", axis, clamped, speed);
                    }
                }
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
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
                // Query motion state: joint_id = axis (0-3). Reply
                // OP_MOTION_REPLY with payload = V2_PAYLOAD_STATUS(pos, spd)
                // and the moving flag in seq bit 0.
                uint8_t axis = f.joint_id;
                if (axis >= NJ) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                int32_t pos = motion_get_position(axis);
                uint32_t spd = motion_get_speed(axis);
                bool moving = motion_is_moving(axis);
                uint8_t rseq = (uint8_t)(moving ? 1 : 0);
                v2_reply(tx_buf, OP_MOTION_REPLY, f.joint_id,
                         V2_PAYLOAD_STATUS(pos, spd), rseq);
                break;
            }

            case OP_HOME: {
                // Homing: joint_id = axis (0-3), payload: 0=start, 1=stop, 2=query.
                uint8_t axis = f.joint_id;
                uint8_t sub = (uint8_t)f.payload;
                if (axis > 2) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                if (sub == 0) {
                    set_enable(true);
                    if (axis == 1 || axis == 2) {
                        // Differential wrist: both physical motors home together
                        // (they share the virtual joint and one limit switch).
                        homing_start(DIFF_MOTOR_A);
                        homing_start(DIFF_MOTOR_B);
                    } else {
                        homing_start(axis);
                    }
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else if (sub == 1) {
                    if (axis == 1 || axis == 2) {
                        homing_stop(DIFF_MOTOR_A);
                        homing_stop(DIFF_MOTOR_B);
                    } else {
                        homing_stop(axis);
                    }
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                } else if (sub == 2) {
                    uint8_t h1 = 0, h2 = 0;
                    if (axis == 1 || axis == 2) {
                        h1 = (homing_is_homed(DIFF_MOTOR_A) && homing_is_homed(DIFF_MOTOR_B)) ? 1 : 0;
                        h2 = (uint8_t)homing_get_state(DIFF_MOTOR_A);
                    } else {
                        h1 = homing_is_homed(axis) ? 1 : 0;
                        h2 = (uint8_t)homing_get_state(axis);
                    }
                    v2_reply(tx_buf, OP_HOMING_REPLY, f.joint_id,
                             ((int32_t)h1) | ((int32_t)h2 << 8), seq);
                } else {
                    v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                }
                break;
            }

            case OP_CFG_READ: {
                // Config read: joint_id = V2_JID_CFG(field, axis).
                uint8_t field = V2_CFG_FIELD(f.joint_id);
                uint8_t axis  = V2_CFG_AXIS(f.joint_id);
                if (axis >= NJ) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
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
                if (axis >= NJ) { v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq); break; }
                // WRIST LOCKED: ignore studio writes to J4/J5/J6 (axes 0/1/2
                // on slave 2) — they keep the fixed mechanical defaults.
                // EXCEPTION: dir_inverted (field 2) is a legitimate user
                // control (the motion-config Invert toggle) — let it through.
                if ((axis == 0 || axis == 1 || axis == 2) && field != CFG_FIELD_DIR_INVERT) {
                    v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                    break;
                }
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
                // Persist so #CFG survives a reboot.
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
                // Config reset to defaults (master #CFGRESET, both slaves).
                config_reset();
                motion_init();
                motion_set_limits(0, 2000, 500);
                motion_set_limits(1, 2000, 500);
                motion_set_limits(2, 2000, 500);
                motion_set_limits(3, 2000, 500);
                v2_reply(tx_buf, OP_ACK, f.joint_id, 0, seq);
                break;
            }

            default:
                v2_reply(tx_buf, OP_FAULT, f.joint_id, FAULT_NONE, seq);
                printf("[S2] UNKNOWN opcode 0x%02X\n", f.opcode);
                break;
        }
#endif // SPI_ONLY_TEST
    }
}
