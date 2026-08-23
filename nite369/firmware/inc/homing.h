#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    HOME_IDLE,
    HOME_SEARCH_FAST,
    HOME_BACKOFF,
    HOME_CREEP,
    HOME_FOUND,
    HOME_ERROR,
} home_state_t;

typedef struct {
    // Config
    uint32_t search_speed;   // fast search speed (steps/sec)
    uint32_t creep_speed;    // slow approach speed (steps/sec)
    int32_t backoff_steps;   // steps to back off after first hit
    bool invert_limit;       // true = NC switch (LOW = triggered)
    bool invert_dir;         // true = home in negative direction

    // State
    home_state_t state;
    bool homed;
    int32_t home_offset;     // steps from limit switch to actual zero
} homing_axis_t;

// Initialize homing module
void homing_init(int num_axes);

// Configure an axis for homing
void homing_set_config(int axis, uint32_t search_speed, uint32_t creep_speed,
                       int32_t backoff_steps, bool invert_limit, bool invert_dir);

// Start homing on an axis
void homing_start(int axis);

// Start homing on all axes
void homing_start_all(void);

// Stop homing on an axis
void homing_stop(int axis);

// Update homing state (call from main loop or timer)
// limit_state: true = limit switch triggered
// Returns true if homing is complete for this axis
bool homing_update(int axis, bool limit_state);

// Check if axis is homed
bool homing_is_homed(int axis);

// Check if any axis is homing
bool homing_any_active(void);

// Get homing state
home_state_t homing_get_state(int axis);

// Reset homed status
void homing_clear_homed(int axis);

#endif
