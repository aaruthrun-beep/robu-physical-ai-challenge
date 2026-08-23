#include "homing.h"
#include "motion_profile.h"

#define MAX_AXES 8

static homing_axis_t home_cfg[MAX_AXES];
static int num_axes = 0;

void homing_init(int n) {
    num_axes = n > MAX_AXES ? MAX_AXES : n;
    for (int i = 0; i < num_axes; i++) {
        home_cfg[i] = (homing_axis_t){
            .search_speed = 1000,
            .creep_speed  = 100,
            .backoff_steps = 200,
            .invert_limit = true,   // NC switch: LOW = triggered
            .invert_dir   = false,
            .state = HOME_IDLE,
            .homed = false,
            .home_offset = 0,
        };
    }
}

void homing_set_config(int axis, uint32_t search_speed, uint32_t creep_speed,
                       int32_t backoff_steps, bool invert_limit, bool invert_dir) {
    if (axis < 0 || axis >= num_axes) return;
    home_cfg[axis].search_speed = search_speed;
    home_cfg[axis].creep_speed = creep_speed;
    home_cfg[axis].backoff_steps = backoff_steps;
    home_cfg[axis].invert_limit = invert_limit;
    home_cfg[axis].invert_dir = invert_dir;
}

// Read an axis's homing configuration (v2 OP_CFG_READ fields 8-10).
// Returns false if the axis is out of range.
bool homing_get_config(int axis, uint32_t *search_speed, uint32_t *creep_speed,
                       int32_t *backoff_steps) {
    if (axis < 0 || axis >= num_axes) return false;
    if (search_speed)  *search_speed  = home_cfg[axis].search_speed;
    if (creep_speed)   *creep_speed   = home_cfg[axis].creep_speed;
    if (backoff_steps) *backoff_steps = home_cfg[axis].backoff_steps;
    return true;
}

void homing_start(int axis) {
    if (axis < 0 || axis >= num_axes) return;
    homing_axis_t *h = &home_cfg[axis];
    h->state = HOME_SEARCH_FAST;
    h->homed = false;

    int dir = h->invert_dir ? 1 : -1;
    motion_set_limits(axis, h->search_speed, h->search_speed);
    motion_move(axis, dir * 100000);  // long move toward limit
}

void homing_start_all(void) {
    for (int i = 0; i < num_axes; i++) homing_start(i);
}

void homing_stop(int axis) {
    if (axis < 0 || axis >= num_axes) return;
    home_cfg[axis].state = HOME_IDLE;
    motion_stop(axis);
}

// Call from main loop. limit_raw = raw GPIO read (true = switch pressed).
// Returns true when homing is complete for this axis.
bool homing_update(int axis, bool limit_raw) {
    if (axis < 0 || axis >= num_axes) return false;
    homing_axis_t *h = &home_cfg[axis];

    // Apply invert: limit_hit = switch is actually triggered
    bool limit_hit = h->invert_limit ? !limit_raw : limit_raw;

    switch (h->state) {
    case HOME_IDLE:
    case HOME_FOUND:
    case HOME_ERROR:
        return h->state == HOME_FOUND;

    case HOME_SEARCH_FAST:
        // Moving toward limit. When hit, stop and back off.
        if (limit_hit) {
            motion_stop(axis);
            int dir = h->invert_dir ? -1 : 1;
            motion_set_limits(axis, h->creep_speed, h->creep_speed);
            motion_move(axis, dir * h->backoff_steps);
            h->state = HOME_BACKOFF;
        }
        return false;

    case HOME_BACKOFF:
        // Moving away from limit. When motion stops (backoff complete), creep toward limit.
        if (!motion_is_moving(axis)) {
            int dir = h->invert_dir ? 1 : -1;
            motion_set_limits(axis, h->creep_speed, h->creep_speed / 2);
            motion_move(axis, dir * 100000);
            h->state = HOME_CREEP;
        }
        return false;

    case HOME_CREEP:
        // Slowly approaching limit. When hit, set home position.
        if (limit_hit) {
            motion_stop(axis);
            motion_set_position(axis, h->home_offset);
            h->homed = true;
            h->state = HOME_FOUND;
            return true;
        }
        // Safety: if we moved too far without hitting limit, error
        if (!motion_is_moving(axis)) {
            h->state = HOME_ERROR;
        }
        return false;

    default:
        return false;
    }
}

bool homing_is_homed(int axis) {
    if (axis < 0 || axis >= num_axes) return false;
    return home_cfg[axis].homed;
}

bool homing_any_active(void) {
    for (int i = 0; i < num_axes; i++) {
        home_state_t s = home_cfg[i].state;
        if (s == HOME_SEARCH_FAST || s == HOME_BACKOFF || s == HOME_CREEP)
            return true;
    }
    return false;
}

home_state_t homing_get_state(int axis) {
    if (axis < 0 || axis >= num_axes) return HOME_IDLE;
    return home_cfg[axis].state;
}

void homing_clear_homed(int axis) {
    if (axis < 0 || axis >= num_axes) return;
    home_cfg[axis].homed = false;
}
