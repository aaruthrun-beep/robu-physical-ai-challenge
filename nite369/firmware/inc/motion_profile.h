#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "motion_marlin.h"

#define MOTION_MAX_AXES 8

typedef struct {
    // Configuration
    uint32_t max_speed;      // steps/sec
    uint32_t accel;          // steps/sec^2
    uint32_t decel;          // steps/sec^2

    // State
    bool active;
    int32_t target_pos;
    int32_t current_pos;
    int32_t total_steps;     // total steps in current move
    int32_t remaining;       // steps remaining
    uint32_t current_speed;  // steps/sec (current)
    int32_t speed_fraction;  // (unused, kept for compat)
    uint32_t timer_interval_us;
    int dir;                 // +1 or -1

    // Marlin trapezoid block for the current move (time-based).
    marlin_block_t block;

    // Continuous-jog state (motion_jog_start/stop).
    bool     jog_active;        // jog running (accel phase or cruising)
    bool     jog_stopping;      // decelerating to stop
    uint32_t jog_target_rate;   // requested jog rate (steps/sec)
    uint32_t jog_accel;         // jog-specific ramp-up (steps/sec^2)
    uint32_t jog_decel;         // rapid decel for jog stop (steps/sec^2)
    int      jog_dir;           // +1 / -1

    // Elapsed time in the current block (us), advanced by the caller's
    // rate-timer callback at the interval it just fired.
    uint32_t elapsed_us;

    // Per-axis rate timer (owned by the caller's timer callback loop).
    struct repeating_timer rate_timer;
    bool edge_high;          // which edge we just fired (for step counting)
} motion_axis_t;

void motion_init(void);
void motion_set_limits(int axis, uint32_t max_speed, uint32_t accel);
void motion_set_decel(int axis, uint32_t decel);

/** Set the rapid deceleration (steps/sec^2) used for continuous-jog stops. */
void motion_set_jog_decel(int axis, uint32_t decel);

/** Set the ramp-up acceleration (steps/sec^2) used for continuous-jog starts. */
void motion_set_jog_accel(int axis, uint32_t accel);

/**
 * Start a CONTINUOUS jog on `axis` in `dir` (+1/-1), accelerating from rest
 * to `rate` (steps/sec) using the axis's configured `accel`. Runs until
 * motion_jog_stop() is called, then decelerates at `jog_decel` (rapid).
 */
void motion_jog_start(int axis, int dir, uint32_t rate);

/**
 * Stop a continuous jog: decelerate from the current rate to zero at the
 * axis's configured rapid `jog_decel`, then stop the step pin.
 */
void motion_jog_stop(int axis);

/**
 * Advance a continuous jog by `delta_us` and return the current step rate.
 * The caller's rate alarm toggles the step pin at marlin_edge_interval_us()
 * of the returned rate. Returns MARLIN_MIN_STEP_RATE and clears jog_active
 * when the jog has fully stopped (caller should stop the pin + alarm).
 */
uint32_t motion_jog_tick(int axis, uint32_t delta_us);

/** True while a continuous jog is active on `axis` (running or stopping). */
bool motion_axis_jogging(int axis);

/** Direction (+1/-1) of the running jog on `axis`. */
int motion_jog_dir(int axis);

/** Count one step (rising edge) into the running jog's position. */
void motion_jog_edge(int axis);
void motion_move(int axis, int32_t delta_steps);
void motion_move_absolute(int axis, int32_t target);
void motion_stop(int axis);
void motion_stop_all(void);
bool motion_is_moving(int axis);
bool motion_any_moving(void);
int32_t motion_get_position(int axis);
void motion_set_position(int axis, int32_t pos);
int32_t motion_get_target(int axis);
uint32_t motion_get_speed(int axis);

/**
 * Advance the elapsed time for an axis by `delta_us` (the interval the
 * caller's rate timer just fired at) and return the current step rate.
 * The caller toggles the step pin at marlin_edge_interval_us(rate) and
 * re-arms its timer.
 */
uint32_t motion_advance(int axis, uint32_t delta_us);

/**
 * Called by the caller's timer callback when a rising edge is emitted:
 * counts a step (decrements remaining, advances position). Returns false
 * when the move is complete (caller should stop the timer and de-energize).
 */
bool motion_edge_fired(int axis);

uint32_t motion_get_interval_us(int axis);

#endif
