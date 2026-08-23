#ifndef NITE_COMMON_TYPES_H
#define NITE_COMMON_TYPES_H

#include <stdint.h>

#define MAX_AXES       6
#define AXES_PER_SLAVE 3

// Joint IDs (0-5 = J1-J6). 0xFF = broadcast to all axes on the slave.
#define JOINT_NONE    0xFF

// Slave IDs on the SPI bus (matches master CS lines)
#define SLAVE_1       1
#define SLAVE_2       2

// Motion status bits (byte 5 of the legacy 0x51 status reply, reused)
#define MOTION_FLAG_MOVING   0x01
#define MOTION_FLAG_HOMED    0x02
#define MOTION_FLAG_FAULT    0x04
#define MOTION_FLAG_LIMIT    0x08

// Fault reason codes (OP_FAULT payload)
typedef enum {
    FAULT_NONE        = 0x00,
    FAULT_TMC         = 0x01,   // TMC driver fault (OTS/OTPW/S2GA/UV)
    FAULT_ENCODER     = 0x02,   // encoder read error / no lock
    FAULT_LIMIT       = 0x03,   // hard limit switch tripped
    FAULT_SOFT_LIMIT  = 0x04,   // soft limit clamp (no move)
    FAULT_BAD_FRAME   = 0x05,   // CRC failure (never executed)
    FAULT_TIMEOUT     = 0x06,   // command timed out
} fault_reason_t;

// Response status codes (compat with legacy nite_spi_proto.h)
#define NITE_RSP_OK        0x00  // command executed OK
#define NITE_RSP_STALE     0x4F  // idle/PONG — slave wasn't processing (retry)
#define NITE_RSP_BAD_FRAME 0xF0  // CRC mismatch — command NOT executed (retry)
#define NITE_RSP_BUSY      0xFE  // axis busy (retry)
#define NITE_RSP_ERR       0xFF  // hard error (do not retry)
#define NITE_RSP_LIMIT     0xFD  // move clamped at a soft limit (no motion)

// Homing states are defined in slave_common/homing.h (home_state_t) —
// that header is the source of truth for the homing state machine.
// See homing.h for HOME_IDLE .. HOME_ERROR.

// Axis config fields for OP_CFG_READ/OP_CFG_WRITE (payload encodes value).
// The numbering matches the v1 slaves' 0x60/0x61 field codes (0=spr,
// 1=gr, 2=di, 3=jog_decel, 4=jog_accel) plus the v1 0x43/0x44 profile
// fields (max_speed/accel/decel) so the migration maps 1:1.
typedef enum {
    CFG_FIELD_STEPS_REV   = 0,   // steps per revolution (gear-aware)
    CFG_FIELD_GEAR_RATIO  = 1,   // gear ratio *100 (e.g. 2280 = 1:22.8)
    CFG_FIELD_DIR_INVERT  = 2,   // 1 = inverted
    CFG_FIELD_JOG_DECEL   = 3,   // steps/sec^2
    CFG_FIELD_JOG_ACCEL   = 4,   // steps/sec^2
    CFG_FIELD_MAX_SPEED   = 5,   // steps/sec
    CFG_FIELD_ACCEL       = 6,   // steps/sec^2
    CFG_FIELD_DECEL       = 7,   // steps/sec^2
    CFG_FIELD_HOME_SEARCH = 8,   // homing search speed (steps/sec)
    CFG_FIELD_HOME_CREEP  = 9,   // homing creep speed (steps/sec)
    CFG_FIELD_HOME_BACKOFF= 10,  // homing backoff (steps)
    CFG_FIELD_LIM_MIN     = 11,  // soft limit min (deg*10)
    CFG_FIELD_LIM_MAX     = 12,  // soft limit max (deg*10)
} cfg_field_t;

#endif // NITE_COMMON_TYPES_H