/**
 * @file nite_tmc2209.h
 * @brief TMC2209 UART Configuration Driver for Nite 369
 *
 * Half-duplex UART protocol for configuring TMC2209 stepper drivers.
 * Wrist Pico (Slave 2) uses this to configure 4x TMC2209 modules:
 *   J4 (Forearm Roll), J5 (Wrist Pitch), J6 (Wrist Roll), Gripper
 *
 * Wiring (shared half-duplex bus):
 *   GP0 (UART TX) ──[1kΩ]──┬── TMC2209#0 (J4,  ADDR 0)
 *                           ├── TMC2209#1 (J5,  ADDR 1)
 *   GP1 (UART RX) ─────────┐├── TMC2209#2 (J6,  ADDR 2)
 *                           │└── TMC2209#3 (GRIP, ADDR 3)
 *   GP28 (ENABLE) ─────────┘── All drivers EN (active LOW)
 *
 * Protocol: 115200 baud, 8N1, half-duplex
 *   Read:  send [0x05, addr, reg, CRC]       → recv [0x05, 0xFF, reg, d3..d0, CRC]
 *   Write: send [0x05, addr|0x80, reg, d3..d0, CRC]
 */

#ifndef NITE_TMC2209_H
#define NITE_TMC2209_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================================================================
// Configuration
// ==========================================================================

/** Number of TMC2209 drivers on the shared UART bus */
#define TMC2209_NUM_DRIVERS     4

/** UART baud rate (max supported by TMC2209 is ~500k, 115200 is reliable) */
#define TMC2209_BAUD            115200

/** TMC2209 driver addresses (set by MS1/MS2 solder jumpers on modules) */
#define TMC2209_ADDR0           0x00    // MS1=GND, MS2=GND  → J4  (Forearm Roll)
#define TMC2209_ADDR1           0x01    // MS1=VCC, MS2=GND → J5  (Wrist Pitch)
#define TMC2209_ADDR2           0x02    // MS1=GND, MS2=VCC → J6  (Wrist Roll)
#define TMC2209_ADDR3           0x03    // MS1=VCC, MS2=VCC → Gripper

// ==========================================================================
// Register Map
// ==========================================================================

#define TMC2209_REG_GCONF       0x00    // Global configuration
#define TMC2209_REG_GSTAT       0x01    // Global status (reset, UV, OT)
#define TMC2209_REG_IFCNT       0x02    // Interface counter (increments on each access)
#define TMC2209_REG_SLAVECONF   0x03    // Slave configuration
#define TMC2209_REG_IHOLD_IRUN  0x10    // Hold current / Run current / delay
#define TMC2209_REG_TPOWERDOWN  0x11    // Power-down delay
#define TMC2209_REG_TPWMTHRS    0x13    // StealthChop → SpreadCycle threshold
#define TMC2209_REG_VACTUAL     0x22    // Constant velocity mode (for direct speed)
#define TMC2209_REG_CHOPCONF    0x6C    // Chopper configuration
#define TMC2209_REG_DRV_STATUS  0x6F    // Driver status (OT, stall, etc.)

// ==========================================================================
// GCONF Bit Definitions
// ==========================================================================

#define TMC2209_GCONF_I_SCALE_ANALOG    (1 << 0)   // 0=internal ref, 1=VREF pin
#define TMC2209_GCONF_INTERNAL_RSENSE   (1 << 1)   // 0=ext sense, 1=int sense
#define TMC2209_GCONF_EN_SPREADCYCLE    (1 << 2)   // 1=SpreadCycle, 0=StealthChop
#define TMC2209_GCONF_SHAFT             (1 << 3)   // 1=reverse motor direction
#define TMC2209_GCONF_INDEX_OTPW        (1 << 4)   // INDEX shows overtemp warning
#define TMC2209_GCONF_INDEX_STEP        (1 << 5)   // INDEX shows step pulses
#define TMC2209_GCONF_PDN_DISABLE       (1 << 6)   // 0=PDN/UART enabled, 1=power-down
#define TMC2209_GCONF_MSTEP_REG_SELECT  (1 << 7)   // 1=UART controls microstep, 0=MS1/MS2
#define TMC2209_GCONF_MULTISTEP_FILT    (1 << 8)   // 1=enable edge filter for step

// ==========================================================================
// IHOLD_IRUN Bit Definitions
// ==========================================================================

#define TMC2209_IRUN(ival)      ((ival) & 0x1F)            // bits 4:0
#define TMC2209_IHOLD(ival)     (((ival) & 0x1F) << 8)     // bits 12:8
#define TMC2209_IHOLDDELAY(d)   (((d) & 0x0F) << 16)       // bits 19:16

// ==========================================================================
// CHOPCONF Bit Definitions
// ==========================================================================

// Microstep resolution table:
//   MRES=0 → full step, MRES=1 → 1/2, MRES=2 → 1/4, MRES=3 → 1/8,
//   MRES=4 → 1/16, MRES=5 → 1/32, MRES=6 → 1/64, MRES=7 → 1/128,
//   MRES=8 → 1/256
#define TMC2209_MRES(m)         (((m) & 0x0F) << 24)       // bits 27:24
#define TMC2209_INTPOL          (1 << 28)                   // 256x interpolation
#define TMC2209_DEDGE           (1 << 29)                   // double edge step pulses
#define TMC2209_DISS2G          (1 << 30)                   // short-to-GND protection

// CHOPCONF chopper defaults (SpreadCycle)
#define TMC2209_TOFF(t)         ((t) & 0x0F)               // bits 3:0 — off time
#define TMC2209_HSTRT(h)        (((h) & 0x0F) << 4)        // bits 7:4 — hysteresis start
#define TMC2209_HEND(h)         (((h) & 0x0F) << 8)        // bits 11:8 — hysteresis end
#define TMC2209_RNDTF           (1 << 13)                   // random off time
#define TMC2209_CHM             (1 << 14)                   // chopper mode
#define TMC2209_TBL(t)          (((t) & 0x03) << 16)       // bits 17:16 — blanking time
#define TMC2209_VHIGHFS         (1 << 20)                   // fullstep at high velocity
#define TMC2209_VHIGHCHM        (1 << 21)                   // alternative chopper at high v

// ==========================================================================
// Default Configuration Values
// ==========================================================================

/** GCONF: SpreadCycle + UART microstep control (no VREF, ext sense) */
#define TMC2209_GCONF_DEFAULT   \
    (TMC2209_GCONF_EN_SPREADCYCLE | TMC2209_GCONF_MSTEP_REG_SELECT)

/** CHOPCONF: 1/8 microsteps (MRES=3), 256x interpolation, standard chopper */
#define TMC2209_CHOPCONF_1_8    \
    (TMC2209_TOFF(3) | TMC2209_HSTRT(5) | TMC2209_HEND(2) | \
     TMC2209_TBL(1) | TMC2209_MRES(3) | TMC2209_INTPOL)

/** CHOPCONF: 1/16 microsteps (MRES=4) */
#define TMC2209_CHOPCONF_1_16   \
    (TMC2209_TOFF(3) | TMC2209_HSTRT(5) | TMC2209_HEND(2) | \
     TMC2209_TBL(1) | TMC2209_MRES(4) | TMC2209_INTPOL)

/** CHOPCONF: 1/32 microsteps (MRES=5) */
#define TMC2209_CHOPCONF_1_32   \
    (TMC2209_TOFF(3) | TMC2209_HSTRT(5) | TMC2209_HEND(2) | \
     TMC2209_TBL(1) | TMC2209_MRES(5) | TMC2209_INTPOL)

/** IHOLD_IRUN: IRUN=16 (~1.0A with 0.11Ω sense), IHOLD=8 (50%), delay=4 */
#define TMC2209_IHOLD_IRUN_1A   \
    (TMC2209_IRUN(16) | TMC2209_IHOLD(8) | TMC2209_IHOLDDELAY(4))

/** IHOLD_IRUN: IRUN=12 (~0.75A), IHOLD=6 (50%), delay=4 */
#define TMC2209_IHOLD_IRUN_075A \
    (TMC2209_IRUN(12) | TMC2209_IHOLD(6) | TMC2209_IHOLDDELAY(4))

/** IHOLD_IRUN: IRUN=8 (~0.5A), IHOLD=4 (50%), delay=4 */
#define TMC2209_IHOLD_IRUN_05A  \
    (TMC2209_IRUN(8) | TMC2209_IHOLD(4) | TMC2209_IHOLDDELAY(4))

// ==========================================================================
// Driver Address Table
// ==========================================================================

/** Human-readable names for each driver */
extern const char *tmc2209_driver_names[TMC2209_NUM_DRIVERS];

/** I2C-style addresses for each driver (0x00 - 0x03) */
extern const uint8_t tmc2209_driver_addrs[TMC2209_NUM_DRIVERS];

// ==========================================================================
// API Functions
// ==========================================================================

/**
 * @brief Initialize UART for TMC2209 communication.
 * Must be called once before any read/write operations.
 * Sets up UART0 on TX=GP0, RX=GP1 at 115200 baud.
 * Also initializes ENABLE pin (GP28) to active LOW.
 */
void tmc2209_uart_init(void);

/**
 * @brief Read a 32-bit register from a TMC2209 driver.
 * @param addr  Driver address (0x00 - 0x03)
 * @param reg   Register address
 * @param value Output: register value read from driver
 * @return true if read was successful (valid response received)
 */
bool tmc2209_read_reg(uint8_t addr, uint8_t reg, uint32_t *value);

/**
 * @brief Write a 32-bit value to a TMC2209 register.
 * @param addr  Driver address (0x00 - 0x03)
 * @param reg   Register address
 * @param value 32-bit value to write
 * @return true if write was acknowledged (verified via IFCNT)
 */
bool tmc2209_write_reg(uint8_t addr, uint8_t reg, uint32_t value);

/**
 * @brief Configure a single driver with specified GCONF, CHOPCONF, IHOLD_IRUN.
 * Calls write for each register, then verifies with readback.
 * @param addr     Driver address
 * @param gconf    GCONF value
 * @param chopconf CHOPCONF value
 * @param ihold_irun IHOLD_IRUN value
 * @param name     Human-readable name for debug output (NULL to skip)
 * @return true if all registers configured and verified
 */
bool tmc2209_configure(uint8_t addr, uint32_t gconf, uint32_t chopconf,
                       uint32_t ihold_irun, const char *name);

/**
 * @brief Configure all 4 TMC2209 drivers with the same settings.
 * Prints status for each driver over USB serial.
 * @param gconf    GCONF value for all drivers
 * @param chopconf CHOPCONF value for all drivers
 * @param ihold_irun IHOLD_IRUN value for all drivers
 * @return number of drivers successfully configured (0-4)
 */
int tmc2209_configure_all(uint32_t gconf, uint32_t chopconf, uint32_t ihold_irun);

/**
 * @brief Quick one-shot: configure all 4 drivers with default settings.
 * Uses TMC2209_GCONF_DEFAULT, CHOPCONF_1_8, and IHOLD_IRUN_1A.
 * @return number of drivers successfully configured (0-4)
 */
int tmc2209_configure_all_defaults(void);

/**
 * @brief Read DRV_STATUS and print diagnostic info.
 * @param addr Driver address
 * @param name Human-readable name (or NULL)
 */
void tmc2209_print_status(uint8_t addr, const char *name);

/**
 * @brief Toggle ENABLE pin. LOW = drivers enabled.
 * @param enabled true to enable drivers, false to disable
 */
void tmc2209_set_enabled(bool enabled);

/**
 * @brief Returns true if ENABLE pin is currently asserted (drivers enabled).
 */
bool tmc2209_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif // NITE_TMC2209_H
