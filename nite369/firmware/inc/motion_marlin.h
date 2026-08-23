#ifndef MOTION_MARLIN_H
#define MOTION_MARLIN_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Marlin-derived trapezoidal motion planner for single-axis moves,
 * TIME-BASED (hardware rate timers).
 *
 * The old port indexed the trapezoid by "step index" and assumed a fixed
 * 1kHz tick where one step fires per tick — which hard-capped the rate at
 * 1000 steps/s and conflated step count with elapsed time (breaking the
 * deceleration ramp: the last step fired at whatever rate the index math
 * produced, giving a hard stop).
 *
 * This version sizes the trapezoid in TIME:
 *   - t_accel: time to ramp initial_rate -> nominal_rate at `accel`
 *   - t_cruise: time at nominal_rate
 *   - t_decel: time to ramp nominal_rate -> final_rate at `decel`
 * The per-edge timer callback asks marlin_step_rate_time(block, elapsed_us)
 * for the rate at the current elapsed time, toggles the step pin at
 * 1/(2*rate), and re-arms itself with the new interval. The step count is
 * the integral of rate over time, so the move lands on `steps` and the
 * final rate is `final_rate` (near zero) — a real deceleration to stop.
 */

// Minimum step rate (steps/sec) — mirrors Marlin's MINIMAL_STEP_RATE.
#define MARLIN_MIN_STEP_RATE 20u

typedef struct {
    int32_t  steps;              // total steps in this move (>0)
    uint32_t initial_rate;       // steps/sec at block start
    uint32_t nominal_rate;       // cruise steps/sec
    uint32_t final_rate;         // steps/sec at block end (near MIN)

    // Phase durations in MICROSECONDS (derived from the rates and accel).
    uint32_t t_accel_us;         // ramp up
    uint32_t t_cruise_us;        // at nominal
    uint32_t t_decel_us;         // ramp down
    uint32_t t_total_us;         // sum

    uint32_t accel;              // steps/sec^2 (stored for the rate math)
    uint32_t decel;              // steps/sec^2
} marlin_block_t;

/**
 * Build a time-based trapezoid block for a single-axis move.
 * steps: total steps (abs value, >0). max_rate: desired cruise (steps/sec).
 * accel/decel: steps/sec^2. entry/exit: rates at block ends (0 = start/stop).
 * Short moves that can't reach nominal become triangles (t_cruise_us = 0,
 * peak rate < nominal).
 */
void marlin_plan_move(marlin_block_t *b, int32_t steps,
                      uint32_t max_rate, uint32_t accel, uint32_t decel,
                      uint32_t entry_rate, uint32_t exit_rate);

/**
 * Step rate (steps/sec) at elapsed time `elapsed_us` within the block.
 * Accel: initial + accel*t. Cruise: nominal. Decel: nominal - decel*t,
 * clamped to >= final_rate.
 */
uint32_t marlin_step_rate_time(const marlin_block_t *b, uint32_t elapsed_us);

/**
 * Edge interval in microseconds for a step rate: half a full step period
 * (pin toggles once per edge, one full step = 2 edges).
 */
uint32_t marlin_edge_interval_us(uint32_t step_rate);

#endif
