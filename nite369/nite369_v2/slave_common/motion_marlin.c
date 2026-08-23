/**
 * motion_marlin.c — TIME-BASED Marlin-derived trapezoidal motion planner.
 *
 * Sizes the trapezoid in time and computes the step rate from elapsed time,
 * so a hardware rate timer can drive the step pin at the true commanded
 * rate (no 1kHz ceiling, correct deceleration to near-zero).
 */

#include "motion_marlin.h"
#include <math.h>

static inline uint32_t _max_u32(uint32_t a, uint32_t b) { return a > b ? a : b; }

// Time (us) to change rate v1->v2 at accel `a`: (v2-v1)/a * 1e6
static uint32_t _rate_time_us(uint32_t v1, uint32_t v2, uint32_t a) {
    if (a == 0 || v2 <= v1) return 0;
    uint64_t dv = (uint64_t)(v2 - v1);
    return (uint32_t)((dv * 1000000ull) / a);
}

// Steps to change rate v1->v2 at accel `a`: (v2^2 - v1^2) / (2*a)
static float _rate_steps(uint32_t v1, uint32_t v2, uint32_t a) {
    if (a == 0) return 0.0f;
    float n1 = (float)v1 * (float)v1;
    float n2 = (float)v2 * (float)v2;
    return (n2 - n1) / (2.0f * (float)a);
}

void marlin_plan_move(marlin_block_t *b, int32_t steps,
                      uint32_t max_rate, uint32_t accel, uint32_t decel,
                      uint32_t entry_rate, uint32_t exit_rate) {
    if (steps < 0) steps = -steps;
    if (steps == 0) steps = 1;

    b->steps = steps;
    b->accel = accel ? accel : 1;
    b->decel = decel ? decel : b->accel;
    b->nominal_rate = _max_u32(max_rate, MARLIN_MIN_STEP_RATE);

    uint32_t initial_rate = _max_u32(entry_rate, MARLIN_MIN_STEP_RATE);
    uint32_t final_rate   = _max_u32(exit_rate, MARLIN_MIN_STEP_RATE);
    if (initial_rate > b->nominal_rate) initial_rate = b->nominal_rate;
    if (final_rate   > b->nominal_rate) final_rate   = b->nominal_rate;
    b->initial_rate = initial_rate;
    b->final_rate   = final_rate;

    // Full-trapezoid phase durations (us) at nominal.
    uint32_t t_accel_us = _rate_time_us(initial_rate, b->nominal_rate, b->accel);
    uint32_t t_decel_us = _rate_time_us(final_rate, b->nominal_rate, b->decel);

    // Steps consumed by the ramps (area under the rate-vs-time curve).
    float accel_steps = _rate_steps(initial_rate, b->nominal_rate, b->accel);
    float decel_steps = _rate_steps(final_rate, b->nominal_rate, b->decel);
    float ramp_steps = accel_steps + decel_steps;

    if (ramp_steps >= (float)steps) {
        // Can't reach nominal: triangle. Solve for peak rate v:
        //   (v^2 - vi^2)/(2a) + (v^2 - vf^2)/(2d) = steps
        //   v^2 * (1/(2a) + 1/(2d)) = steps + vi^2/(2a) + vf^2/(2d)
        float a = b->accel, d = b->decel;
        float vi = (float)initial_rate, vf = (float)final_rate;
        float coeff = 1.0f / (2.0f * a) + 1.0f / (2.0f * d);
        float rhs = (float)steps + (vi * vi) / (2.0f * a) + (vf * vf) / (2.0f * d);
        float peak = (coeff > 0.0f) ? sqrtf(rhs / coeff) : (float)b->nominal_rate;
        if (peak > (float)b->nominal_rate) peak = (float)b->nominal_rate;
        if (peak < (float)MARLIN_MIN_STEP_RATE) peak = (float)MARLIN_MIN_STEP_RATE;
        uint32_t peak_rate = (uint32_t)peak;

        b->t_accel_us = _rate_time_us(initial_rate, peak_rate, b->accel);
        b->t_decel_us = _rate_time_us(final_rate, peak_rate, b->decel);
        b->t_cruise_us = 0;
        b->nominal_rate = peak_rate;  // actual peak reached
        b->t_total_us = b->t_accel_us + b->t_decel_us;
    } else {
        // Full trapezoid: cruise fills the remaining distance.
        float cruise_steps = (float)steps - ramp_steps;
        if (cruise_steps < 0.0f) cruise_steps = 0.0f;
        uint32_t t_cruise_us = (uint32_t)(cruise_steps / (float)b->nominal_rate * 1000000.0f);
        b->t_accel_us = t_accel_us;
        b->t_decel_us = t_decel_us;
        b->t_cruise_us = t_cruise_us;
        b->t_total_us = t_accel_us + t_cruise_us + t_decel_us;
    }

    // Guard against a zero-duration profile (shouldn't happen, but safety).
    if (b->t_total_us == 0) b->t_total_us = 1;
}

uint32_t marlin_step_rate_time(const marlin_block_t *b, uint32_t elapsed_us) {
    if (elapsed_us >= b->t_total_us) {
        return b->final_rate;  // past the end — final rate
    }
    if (elapsed_us < b->t_accel_us) {
        // Acceleration ramp: initial + accel * t.
        // 32-bit math only (M0+ has no hardware divide; the 64-bit version
        // starved the bit-bang SPI loop during a 2-motor move).
        // Exact split so nothing overflows u32:
        //   a*t/1e6 = a*(t/1000)/1000 + a*(t%1000)/1000000
        //   a*(t/1000) max = 50000*100 = 5e6 (fits); /1000 gives the ms-scale term
        uint32_t rate = b->initial_rate +
            (b->accel * (elapsed_us / 1000u)) / 1000u +
            (b->accel * (elapsed_us % 1000u)) / 1000000u;
        if (rate > b->nominal_rate) rate = b->nominal_rate;
        return rate;
    }
    uint32_t after_accel = elapsed_us - b->t_accel_us;
    if (after_accel < b->t_cruise_us) {
        // Cruise at nominal.
        return b->nominal_rate;
    }
    // Deceleration ramp: nominal - decel * t.
    uint32_t t_decel = after_accel - b->t_cruise_us;
    if (t_decel > b->t_decel_us) t_decel = b->t_decel_us;
    uint32_t drop =
        (b->decel * (t_decel / 1000u)) / 1000u +
        (b->decel * (t_decel % 1000u)) / 1000000u;
    int32_t rate = (int32_t)b->nominal_rate - (int32_t)drop;
    if (rate < (int32_t)b->final_rate) rate = b->final_rate;
    if (rate < (int32_t)MARLIN_MIN_STEP_RATE) rate = MARLIN_MIN_STEP_RATE;
    return (uint32_t)rate;
}

uint32_t marlin_edge_interval_us(uint32_t step_rate) {
    if (step_rate == 0) step_rate = MARLIN_MIN_STEP_RATE;
    // One full step = 2 edges; each edge interval = 1e6 / (2*rate).
    uint32_t us = 1000000u / (step_rate * 2u);
    if (us < 5u) us = 5u;             // 200 kHz max edge rate
    if (us > 1000000u) us = 1000000u; // 0.5 Hz min
    return us;
}
