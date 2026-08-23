#include "config_store.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "motion_profile.h"
#include <string.h>

static config_t active_config;
static bool config_loaded = false;

// Compute simple checksum (sum of all uint32_t words except the checksum field itself)
static uint32_t compute_checksum(const config_t *cfg) {
    const uint32_t *data = (const uint32_t *)cfg;
    uint32_t sum = 0;
    // Sum everything except the last word (checksum)
    int count = (sizeof(config_t) - sizeof(uint32_t)) / sizeof(uint32_t);
    for (int i = 0; i < count; i++) {
        sum ^= data[i];  // XOR is better than addition for checksums
    }
    return sum;
}

static void load_defaults(void) {
    memset(&active_config, 0, sizeof(config_t));
    active_config.magic = CONFIG_MAGIC;
    active_config.version = 1;
    active_config.servo_rate = 1000;
    active_config.encoder_enabled = 0;

    // Default axis configs
    for (int i = 0; i < 8; i++) {
        active_config.axes[i].steps_per_rev = 3200;  // 200 * 16 microsteps
        active_config.axes[i].gear_ratio = 100;       // 1:1 (default)
        active_config.axes[i].max_speed = 2000;
        active_config.axes[i].accel = 500;
        active_config.axes[i].decel = 500;
        active_config.axes[i].jog_accel = 1000;       // continuous-jog ramp-up (2x accel)
        active_config.axes[i].jog_decel = 2000;       // rapid stop (4x accel)
        active_config.axes[i].home_offset = 0;
        active_config.axes[i].home_invert_dir = false;
        active_config.axes[i].home_invert_lim = true;  // NC switch
        active_config.axes[i].dir_inverted = false;
        active_config.axes[i].lim_min_deg = -1800;  // -180.0° (deg*10)
        active_config.axes[i].lim_max_deg = 1800;   // +180.0° (deg*10)
    }
    // Axis 0 (J1) uses a 1:22.8 gearbox (gear_ratio stored *100 = 2280).
    active_config.axes[0].gear_ratio = 2280;
    // Axis 1 (J2) uses a 1:45.96 gearbox (stored *100 = 4596).
    active_config.axes[1].gear_ratio = 4596;
    // Axis 2 (J3) uses a 1:40.78125 gearbox (stored *100 = 4078.125 -> 4078).
    active_config.axes[2].gear_ratio = 4078;

    active_config.checksum = compute_checksum(&active_config);
}

void config_init(void) {
    // Read from flash
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + CONFIG_FLASH_OFFSET);
    const config_t *stored = (const config_t *)flash_data;

    if (stored->magic == CONFIG_MAGIC) {
        // Verify checksum
        uint32_t expected = compute_checksum(stored);
        if (expected == stored->checksum) {
            memcpy(&active_config, stored, sizeof(config_t));
            config_loaded = true;
            return;
        }
    }

    // Invalid or corrupted — load defaults
    load_defaults();
    config_loaded = true;
}

const config_t *config_get(void) {
    return &active_config;
}

config_t *config_get_mut(void) {
    return &active_config;
}

// The RP2040 flash API requires flash_range_program() to write whole 256-byte
// pages. config_t is ~308 bytes, so store it in a 512-byte page: erase 4KB,
// program the first 512 bytes (config + 0xFF padding). Must run from RAM
// because flash programming stalls XIP while the code is being fetched.
#define CONFIG_FLASH_PAGE_SIZE 512
static uint8_t config_flash_buf[CONFIG_FLASH_PAGE_SIZE] __attribute__((aligned(CONFIG_FLASH_PAGE_SIZE)));

bool __no_inline_not_in_flash_func(config_save)(void) {
    active_config.checksum = compute_checksum(&active_config);

    // Build a full page: config followed by 0xFF padding (erased flash = 0xFF).
    memset(config_flash_buf, 0xFF, sizeof(config_flash_buf));
    memcpy(config_flash_buf, &active_config, sizeof(config_t));

    // Disable interrupts during flash write (required by the SDK).
    uint32_t ints = save_and_disable_interrupts();

    // Erase the 4KB sector, then program the 512-byte page.
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, config_flash_buf, CONFIG_FLASH_PAGE_SIZE);

    restore_interrupts(ints);

    // Verify read-back
    const uint8_t *flash_data = (const uint8_t *)(XIP_BASE + CONFIG_FLASH_OFFSET);
    const config_t *stored = (const config_t *)flash_data;

    if (stored->magic != CONFIG_MAGIC) return false;
    if (compute_checksum(stored) != stored->checksum) return false;

    return true;
}

void config_reset(void) {
    load_defaults();
}

bool config_is_valid(void) {
    return config_loaded && active_config.magic == CONFIG_MAGIC;
}

uint32_t config_steps_per_deg(int axis) {
    if (axis < 0 || axis >= 8) return 0;
    const config_t *cfg = &active_config;
    // steps_per_rev / 360, accounting for gear ratio
    // gear_ratio is stored as ratio * 100 (e.g. 500 = 5:1)
    uint32_t steps_per_rev = cfg->axes[axis].steps_per_rev;
    uint32_t gr = cfg->axes[axis].gear_ratio;
    if (gr == 0) gr = 100;
    // Output steps per degree = (steps_per_rev * gr / 100) / 360
    return (steps_per_rev * gr) / (100 * 360);
}

int32_t config_clamp_move(int axis, int32_t delta_steps) {
    if (axis < 0 || axis >= 8) return delta_steps;
    const config_t *cfg = &active_config;
    uint32_t spd = config_steps_per_deg(axis);
    if (spd == 0) return delta_steps;
    // current position in degrees*10 (steps -> deg*10)
    int32_t cur = motion_get_position(axis);
    int64_t cur_deg10 = ((int64_t)cur * 10) / spd;
    int64_t delta_deg10 = ((int64_t)delta_steps * 10) / spd;
    int64_t new_deg10 = cur_deg10 + delta_deg10;
    int32_t lo = cfg->axes[axis].lim_min_deg;
    int32_t hi = cfg->axes[axis].lim_max_deg;
    if (new_deg10 < lo || new_deg10 > hi) {
        // clamp delta so the move stops at the limit
        int64_t allowed = (delta_steps > 0)
            ? ((int64_t)hi - cur_deg10) * spd / 10
            : ((int64_t)lo - cur_deg10) * spd / 10;
        return (int32_t)allowed;
    }
    return delta_steps;
}
