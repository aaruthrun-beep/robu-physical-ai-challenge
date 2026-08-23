#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdint.h>
#include <stdbool.h>

// Stored in the last 4KB sector of the app flash region.
// The linker constrains the app to PICO_FLASH_SIZE_BYTES = 2MB - 64KB
// (0x1F0000) so the app and config never collide. The config sector must be
// INSIDE that bound: hardware_flash hard_asserts
// flash_offs + count <= PICO_FLASH_SIZE_BYTES, so an offset at physical
// 2MB-4KB (0x1FF000) trips the assert and the slave hard-faults on save.
// 0x1EF000 = 2MB - 64KB reserve - 4KB sector.
#define CONFIG_FLASH_OFFSET  (2u * 1024u * 1024u - 64u * 1024u - 4096u)  // 0x1EF000
#define CONFIG_MAGIC         0x4E333639  // "N369"

// Absolute upper bound for commanded step rates (steps/sec). Every write
// path (studio, master, slave) clamps max_speed to this value.
#define CONFIG_MAX_SPEED_LIMIT 8000u

typedef struct {
    uint32_t magic;
    uint32_t version;

    // Per-axis config (up to 8 axes)
    struct {
        uint32_t steps_per_rev;    // microsteps per full revolution
        uint32_t gear_ratio;       // gear ratio * 100 (e.g. 500 = 5.00:1)
        uint32_t max_speed;        // steps/sec
        uint32_t accel;            // steps/sec^2
        uint32_t decel;            // steps/sec^2
        uint32_t jog_accel;        // steps/sec^2 — continuous-jog accel (ramp-up)
        uint32_t jog_decel;        // steps/sec^2 — rapid stop for continuous jog
        int32_t  home_offset;      // steps from limit switch to zero
        bool     home_invert_dir;  // home direction
        bool     home_invert_lim;  // limit switch polarity (true = NC)
        bool     dir_inverted;     // invert motor direction
        int32_t  lim_min_deg;      // soft limit min (degrees * 10)
        int32_t  lim_max_deg;      // soft limit max (degrees * 10)
        uint8_t  _pad[1];
    } axes[8];

    // Global config
    uint32_t servo_rate;           // servo update rate in Hz
    uint32_t encoder_enabled;      // bitmask: bit N = encoder on axis N
    uint8_t  _reserved[64];

    uint32_t checksum;
} config_t;

// Initialize config store (loads from flash if valid)
void config_init(void);

// Get pointer to active config (read-only access)
const config_t *config_get(void);

// Get mutable pointer (for direct editing)
config_t *config_get_mut(void);

// Save current config to flash
bool config_save(void);

// Reset config to factory defaults
void config_reset(void);

// Check if config is loaded and valid
bool config_is_valid(void);

// Utility: compute steps/degree from config for an axis
uint32_t config_steps_per_deg(int axis);

// Soft-limit check: returns the clamped delta (steps) so the move stays
// within [lim_min_deg, lim_max_deg]. Returns 0 if the move is blocked.
int32_t config_clamp_move(int axis, int32_t delta_steps);

#endif
