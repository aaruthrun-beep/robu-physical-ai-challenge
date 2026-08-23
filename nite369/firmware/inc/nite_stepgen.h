#ifndef NITE_STEPGEN_H
#define NITE_STEPGEN_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NITE_STEPGEN_NUM_AXES    4
#define NITE_STEPGEN_CTRL_PERIOD_US 100

typedef struct {
    uint step_pin;
    uint dir_pin;
    uint en_pin;
    uint32_t max_velocity;
    uint32_t acceleration;
    bool dir_inverted;
} nite_stepgen_config_t;

void nite_stepgen_init(PIO pio, uint pio_offset, const nite_stepgen_config_t configs[NITE_STEPGEN_NUM_AXES]);

void nite_stepgen_enable(uint8_t axis, bool en);
void nite_stepgen_set_target(uint8_t axis, int32_t pos);
void nite_stepgen_set_velocity(uint8_t axis, uint32_t hz);
void nite_stepgen_set_accel(uint8_t axis, uint32_t accel);
void nite_stepgen_invert_dir(uint8_t axis, bool inv);
void nite_stepgen_zero(uint8_t axis);
void nite_stepgen_home(uint8_t axis);

int32_t nite_stepgen_get_pos(uint8_t axis);
int32_t nite_stepgen_get_velocity(uint8_t axis);
int32_t nite_stepgen_get_step_count(uint8_t axis);
bool nite_stepgen_get_enabled(uint8_t axis);
int32_t nite_stepgen_get_target(uint8_t axis);

void nite_stepgen_run(void);

#ifdef __cplusplus
}
#endif

#endif
