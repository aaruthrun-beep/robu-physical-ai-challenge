#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/platform_defs.h"
#include "freq_generator.pio.h"
#include "nite_stepgen.h"

static PIO pio_inst;
static uint pio_offset;

typedef struct {
    int32_t target_pos;
    int32_t current_pos;
    int32_t velocity;
    int32_t max_velocity;
    int32_t acceleration;
    uint32_t pio_delay;
    bool enabled;
    bool dir_inverted;
    int32_t step_count;
    int32_t step_fraction;    // Fractional step accumulator (1/10000 step units)
    int32_t accel_fraction;   // Fractional velocity accumulator (1/10000 Hz units)
    uint step_pin;
    uint dir_pin;
    uint en_pin;
} axis_t;

static axis_t axes[NITE_STEPGEN_NUM_AXES];
static absolute_time_t next_ctrl_time;

static inline uint32_t hz_to_delay(uint32_t hz) {
    if (hz < 1) return 0x3FFFFF;
    if (hz > 50000) hz = 50000;
    int32_t d = ((int32_t)(SYS_CLK_HZ / hz) - 268) / 2;
    if (d < 1) d = 1;
    if (d > 0x3FFFFF) d = 0x3FFFFF;
    return (uint32_t)d;
}

static inline void pio_send_steps(PIO pio, uint sm, uint16_t steps, uint32_t delay_cycles) {
    if (steps == 0 || delay_cycles == 0x3FFFFF) return;
    uint32_t payload = (uint32_t)steps | ((delay_cycles & 0x3FFFFF) << 10);
    pio_sm_put_blocking(pio, sm, payload);
}

void nite_stepgen_init(PIO pio, uint offset, const nite_stepgen_config_t configs[NITE_STEPGEN_NUM_AXES]) {
    pio_inst = pio;
    pio_offset = offset;

    for (uint8_t a = 0; a < NITE_STEPGEN_NUM_AXES; a++) {
        axes[a].target_pos = 0;
        axes[a].current_pos = 0;
        axes[a].velocity = 0;
        axes[a].max_velocity = configs[a].max_velocity;
        axes[a].acceleration = configs[a].acceleration;
        axes[a].pio_delay = hz_to_delay(configs[a].max_velocity);
        axes[a].enabled = false;
        axes[a].dir_inverted = configs[a].dir_inverted;
        axes[a].step_count = 0;
        axes[a].step_fraction = 0;
        axes[a].accel_fraction = 0;
        axes[a].step_pin = configs[a].step_pin;
        axes[a].dir_pin = configs[a].dir_pin;
        axes[a].en_pin = configs[a].en_pin;

        gpio_init(configs[a].step_pin); gpio_set_dir(configs[a].step_pin, GPIO_OUT); gpio_put(configs[a].step_pin, 0);
        gpio_init(configs[a].dir_pin);  gpio_set_dir(configs[a].dir_pin, GPIO_OUT);  gpio_put(configs[a].dir_pin, 0);
        gpio_init(configs[a].en_pin);   gpio_set_dir(configs[a].en_pin, GPIO_OUT);   gpio_put(configs[a].en_pin, 0);

        pio_gpio_init(pio, configs[a].step_pin);
        pio_sm_set_consecutive_pindirs(pio, a, configs[a].step_pin, 1, true);

        pio_sm_config c = freq_generator_program_get_default_config(offset);
        sm_config_set_set_pins(&c, configs[a].step_pin, 1);
        sm_config_set_out_shift(&c, false, false, 32);
        pio_sm_init(pio, a, offset, &c);
        pio_sm_set_enabled(pio, a, false);
    }

    next_ctrl_time = make_timeout_time_us(NITE_STEPGEN_CTRL_PERIOD_US);
}

void nite_stepgen_enable(uint8_t a, bool en) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].enabled = en;
    gpio_put(axes[a].en_pin, en ? 0 : 1);   // Active LOW: 0 = enabled, 1 = disabled
    if (en) {
        pio_sm_clear_fifos(pio_inst, a);
        pio_sm_set_enabled(pio_inst, a, true);
    } else {
        pio_sm_set_enabled(pio_inst, a, false);
    }
}

void nite_stepgen_set_target(uint8_t a, int32_t pos) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].target_pos = pos;
}

void nite_stepgen_set_velocity(uint8_t a, uint32_t vel) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].max_velocity = vel;
    axes[a].pio_delay = hz_to_delay(vel);
}

void nite_stepgen_set_accel(uint8_t a, uint32_t accel) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].acceleration = accel;
}

void nite_stepgen_invert_dir(uint8_t a, bool inv) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].dir_inverted = inv;
}

void nite_stepgen_zero(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].current_pos = 0;
    axes[a].step_count = 0;
}

void nite_stepgen_home(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return;
    axes[a].target_pos = 0;
}

int32_t nite_stepgen_get_pos(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return 0;
    return axes[a].current_pos;
}

int32_t nite_stepgen_get_velocity(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return 0;
    return axes[a].velocity;
}

int32_t nite_stepgen_get_step_count(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return 0;
    return axes[a].step_count;
}

bool nite_stepgen_get_enabled(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return false;
    return axes[a].enabled;
}

int32_t nite_stepgen_get_target(uint8_t a) {
    if (a >= NITE_STEPGEN_NUM_AXES) return 0;
    return axes[a].target_pos;
}

static int32_t clamp(int32_t v, int32_t min, int32_t max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

void nite_stepgen_run(void) {
    if (absolute_time_diff_us(get_absolute_time(), next_ctrl_time) > 0) {
        return;
    }
    next_ctrl_time = delayed_by_us(next_ctrl_time, NITE_STEPGEN_CTRL_PERIOD_US);

    for (uint8_t a = 0; a < NITE_STEPGEN_NUM_AXES; a++) {
        if (!axes[a].enabled) continue;

        int32_t error = axes[a].target_pos - axes[a].current_pos;
        if (error == 0) {
            axes[a].velocity = 0;
            continue;
        }

        bool dir = error > 0;
        bool actual_dir = axes[a].dir_inverted ? !dir : dir;
        gpio_put(axes[a].dir_pin, actual_dir ? 1 : 0);

        int32_t abs_error = error >= 0 ? error : -error;

        // --- Fractional acceleration accumulation ---
        // Each 100µs cycle contributes: accel[Hz/s] * 100µs / 1,000,000µs
        // = accel / 10000 Hz per cycle. Accumulate in 1/10000 Hz units.
        // When accumulator reaches >= 10000, velocity changes by 1 Hz.
        axes[a].accel_fraction += axes[a].acceleration;
        int32_t vel_delta = axes[a].accel_fraction / 10000;
        axes[a].accel_fraction -= vel_delta * 10000;

        if (dir) {
            axes[a].velocity = clamp(axes[a].velocity + vel_delta,
                                    -axes[a].max_velocity, axes[a].max_velocity);
        } else {
            axes[a].velocity = clamp(axes[a].velocity - vel_delta,
                                    -axes[a].max_velocity, axes[a].max_velocity);
        }

        // --- Deceleration ramp: limit velocity based on remaining distance ---
        // Formula: v_max = sqrt(2 * a * d)  where a=accel[Hz/s], d=distance[steps]
        // This gives the max safe velocity for stopping within remaining distance.
        int32_t max_safe_vel = 0;
        if (abs_error > 0) {
            max_safe_vel = (int32_t)sqrtf(2.0f * (float)axes[a].acceleration * (float)abs_error);
        }
        if (dir) {
            if (axes[a].velocity > max_safe_vel) axes[a].velocity = max_safe_vel;
        } else {
            if (axes[a].velocity < -max_safe_vel) axes[a].velocity = -max_safe_vel;
        }

        int32_t speed = axes[a].velocity >= 0 ? axes[a].velocity : -axes[a].velocity;

        // --- Fractional step accumulation ---
        // Each 100µs cycle contributes: speed[steps/sec] * 100µs / 1,000,000µs
        // = speed / 10000 steps per cycle. Accumulate in 1/10000 step units.
        // When accumulator >= 10000, send that many whole steps.
        axes[a].step_fraction += speed;
        int32_t steps_to_send = axes[a].step_fraction / 10000;
        axes[a].step_fraction -= steps_to_send * 10000;

        // Cap burst size to PIO FIFO limits (1023 max per payload)
        if (steps_to_send > 1023) steps_to_send = 1023;

        // Update pio_delay dynamically from current speed
        // Floor at 30 Hz to avoid hz_to_delay returning 0x3FFFFF sentinel
        // (hz_to_delay returns sentinel for speeds < ~15 Hz, which silently drops steps)
        if (speed > 0) {
            uint32_t pio_speed = (uint32_t)(speed < 30 ? 30 : speed);
            axes[a].pio_delay = hz_to_delay(pio_speed);
        }

        // Clamp to remaining error to prevent overshoot
        if (abs_error > 0 && steps_to_send >= abs_error) {
            steps_to_send = abs_error;
        }

        axes[a].current_pos += dir ? steps_to_send : -steps_to_send;
        axes[a].step_count += steps_to_send;

        // Snap to target exactly once we arrive or overshoot
        if ((dir && axes[a].current_pos >= axes[a].target_pos) ||
            (!dir && axes[a].current_pos <= axes[a].target_pos)) {
            axes[a].current_pos = axes[a].target_pos;
            axes[a].velocity = 0;
            axes[a].step_fraction = 0;
        }

        if (steps_to_send > 0) {
            pio_send_steps(pio_inst, a, (uint16_t)steps_to_send, axes[a].pio_delay);
        }
    }
}
