#include "motion_profile.h"
#include "motion_marlin.h"

static motion_axis_t axes[MOTION_MAX_AXES];

#define DEFAULT_MAX_SPEED 2000
#define DEFAULT_ACCEL     500

void motion_init(void) {
    for (int i = 0; i < MOTION_MAX_AXES; i++) {
        axes[i] = (motion_axis_t){
            .max_speed = DEFAULT_MAX_SPEED,
            .accel = DEFAULT_ACCEL,
            .decel = DEFAULT_ACCEL,
            .speed_fraction = 0,
            .jog_active = false,
            .jog_stopping = false,
            .jog_target_rate = 0,
            .jog_accel = DEFAULT_ACCEL * 2,
            .jog_decel = DEFAULT_ACCEL * 4,
            .jog_dir = 1,
        };
    }
}

void motion_set_limits(int axis, uint32_t max_speed, uint32_t accel) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    axes[axis].max_speed = max_speed;
    axes[axis].accel = accel;
}

void motion_set_decel(int axis, uint32_t decel) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    axes[axis].decel = decel > 0 ? decel : axes[axis].accel;
}

void motion_set_jog_decel(int axis, uint32_t decel) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    axes[axis].jog_decel = decel > 0 ? decel : axes[axis].accel;
}

void motion_set_jog_accel(int axis, uint32_t accel) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    axes[axis].jog_accel = accel > 0 ? accel : axes[axis].accel;
}

void motion_jog_start(int axis, int dir, uint32_t rate) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    motion_axis_t *a = &axes[axis];

    // Cancel any finite move already on this axis.
    a->active = false;
    a->current_speed = 0;
    a->speed_fraction = 0;
    a->elapsed_us = 0;
    a->edge_high = false;

    a->jog_active = true;
    a->jog_stopping = false;
    a->jog_dir = (dir > 0) ? 1 : -1;
    a->jog_target_rate = (rate > 0) ? rate : 1;
    a->jog_decel = a->jog_decel > 0 ? a->jog_decel : a->accel;
    a->timer_interval_us = marlin_edge_interval_us(a->current_speed > 0
                                                   ? a->current_speed
                                                   : MARLIN_MIN_STEP_RATE);
}

void motion_jog_stop(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    motion_axis_t *a = &axes[axis];
    if (!a->jog_active) return;
    // Enter the rapid-decel phase. The caller's rate alarm keeps running
    // until the rate reaches zero, then stops the pin.
    a->jog_stopping = true;
}

/**
 * Advance a continuous jog by `delta_us` and return the current step rate.
 * Called by the rate alarm while jog_active is set.
 *  - Accel phase: current_speed += accel * dt, clamped to jog_target_rate.
 *  - Cruise: holds jog_target_rate.
 *  - Stopping: current_speed -= jog_decel * dt; at 0 the jog ends and the
 *    caller stops the step pin.
 */
uint32_t motion_jog_tick(int axis, uint32_t delta_us) {
    motion_axis_t *a = &axes[axis];
    if (!a->jog_active) return MARLIN_MIN_STEP_RATE;

    if (!a->jog_stopping) {
        // Accelerate toward the target rate using the jog-specific ramp-up.
        // 32-bit math only: the M0+ has no hardware divide, and the 64-bit
        // version (~100+ cycles) was starving the bit-bang SPI loop during a
        // 2-motor differential jog (J5/J6 -> slave 2 frames corrupted).
        // max accel*delta = 100000 * 25000 (first 25ms fire) = 2.5e9 < 2^32,
        // so uint32 multiply is safe for all realistic jog values.
        uint32_t add = (uint32_t)a->jog_accel * delta_us / 1000000u;
        uint32_t ns = a->current_speed + add;
        a->current_speed = (ns > a->jog_target_rate) ? a->jog_target_rate : ns;
    } else {
        // Rapid deceleration to stop.
        uint32_t sub = (uint32_t)a->jog_decel * delta_us / 1000000u;
        if (sub >= a->current_speed) {
            a->current_speed = 0;
            a->jog_active = false;
            a->jog_stopping = false;
            return MARLIN_MIN_STEP_RATE;  // jog finished — caller stops pin
        }
        a->current_speed -= sub;
    }

    a->timer_interval_us = marlin_edge_interval_us(a->current_speed > 0
                                                   ? a->current_speed
                                                   : MARLIN_MIN_STEP_RATE);
    return a->current_speed;
}

bool motion_axis_jogging(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return false;
    return axes[axis].jog_active;
}

int motion_jog_dir(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return 1;
    return axes[axis].jog_dir > 0 ? 1 : -1;
}

void motion_jog_edge(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    motion_axis_t *a = &axes[axis];
    if (!a->jog_active) return;
    a->current_pos += a->jog_dir;
    a->target_pos = a->current_pos;
}

void motion_move(int axis, int32_t delta) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    motion_axis_t *a = &axes[axis];

    if (delta == 0) { a->active = false; return; }

    // Cancel any running jog and stop current motion first.
    a->jog_active = false;
    a->jog_stopping = false;
    a->active = false;
    a->current_speed = 0;
    a->speed_fraction = 0;
    a->elapsed_us = 0;
    a->edge_high = false;

    a->dir = (delta > 0) ? 1 : -1;
    a->total_steps = delta > 0 ? delta : -delta;
    a->remaining = a->total_steps;
    a->target_pos = a->current_pos + delta;

    // Build a time-based Marlin trapezoid block for this move.
    marlin_plan_move(&a->block, a->total_steps,
                     a->max_speed, a->accel, a->decel, 0, 0);

    a->current_speed = a->block.initial_rate;
    a->timer_interval_us = marlin_edge_interval_us(a->current_speed);
    a->active = true;
}

void motion_move_absolute(int axis, int32_t target) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    motion_move(axis, target - axes[axis].current_pos);
}

void motion_stop(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    axes[axis].active = false;
    axes[axis].jog_active = false;
    axes[axis].jog_stopping = false;
    axes[axis].current_speed = 0;
    axes[axis].speed_fraction = 0;
}

void motion_stop_all(void) {
    for (int i = 0; i < MOTION_MAX_AXES; i++) motion_stop(i);
}

bool motion_is_moving(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return false;
    return axes[axis].active;
}

bool motion_any_moving(void) {
    for (int i = 0; i < MOTION_MAX_AXES; i++)
        if (axes[i].active) return true;
    return false;
}

int32_t motion_get_position(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return 0;
    return axes[axis].current_pos;
}

void motion_set_position(int axis, int32_t pos) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return;
    axes[axis].current_pos = pos;
    axes[axis].target_pos = pos;
}

int32_t motion_get_target(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return 0;
    return axes[axis].target_pos;
}

uint32_t motion_get_speed(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return 0;
    return axes[axis].current_speed;
}

uint32_t motion_advance(int axis, uint32_t delta_us) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return MARLIN_MIN_STEP_RATE;
    motion_axis_t *a = &axes[axis];

    if (!a->active || a->remaining <= 0) {
        return MARLIN_MIN_STEP_RATE;
    }

    a->elapsed_us += delta_us;
    if (a->elapsed_us >= a->block.t_total_us) {
        a->elapsed_us = a->block.t_total_us;
    }

    uint32_t rate = marlin_step_rate_time(&a->block, a->elapsed_us);
    a->current_speed = rate;
    a->timer_interval_us = marlin_edge_interval_us(rate);
    return rate;
}

bool motion_edge_fired(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return false;
    motion_axis_t *a = &axes[axis];

    if (!a->active) return false;
    if (a->remaining <= 0) {
        a->active = false;
        a->current_speed = 0;
        return false;
    }

    // A rising edge = one full step (dir pin is set by the caller).
    a->remaining--;
    a->current_pos += a->dir;

    if (a->remaining <= 0) {
        a->active = false;
        a->current_speed = 0;
        // Advance elapsed to the end so any final read reports final_rate.
        if (a->elapsed_us < a->block.t_total_us) a->elapsed_us = a->block.t_total_us;
        return false;  // move complete — caller stops the timer
    }
    return true;  // keep going
}

uint32_t motion_get_interval_us(int axis) {
    if (axis < 0 || axis >= MOTION_MAX_AXES) return 100000;
    return axes[axis].timer_interval_us > 0 ? axes[axis].timer_interval_us : 100000;
}
