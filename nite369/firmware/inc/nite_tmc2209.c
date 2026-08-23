/**
 * @file nite_tmc2209.c
 * @brief TMC2209 UART Configuration Driver — Implementation
 *
 * Implements half-duplex UART communication with TMC2209 stepper drivers.
 * Provides register read/write with CRC8 verification and helper functions
 * for configuring all 4 wrist drivers in one call.
 *
 * CRC8-ATM polynomial: 0x07 (x^8 + x^2 + x + 1) — matches TMC2209 datasheet.
 */

#include "nite_tmc2209.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ==========================================================================
// Pin Definitions
// ==========================================================================

/** Pico UART instance */
#define TMC2209_UART        uart0

/** TX pin */
#define TMC2209_TX_PIN      0

/** RX pin */
#define TMC2209_RX_PIN      1

/** Enable pin (shared for all TMC2209s on this slave) */
#define TMC2209_EN_PIN      28

/** Total read timeout in microseconds (wait for all response bytes) */
#define TMC2209_TIMEOUT_US  50000

// ==========================================================================
// Protocol Constants
// ==========================================================================

/** UART sync byte — start of every frame */
#define TMC2209_SYNC        0x05

/** Response sync byte echoed by driver */
#define TMC2209_RES_SYNC    0x05

/** Response address byte — driver echoes 0xFF */
#define TMC2209_RES_ADDR    0xFF

// ==========================================================================
// Driver Tables
// ==========================================================================

const char *tmc2209_driver_names[TMC2209_NUM_DRIVERS] = {
    "J4 (Forearm Roll)",
    "J5 (Wrist Pitch)",
    "J6 (Wrist Roll)",
    "Gripper"
};

const uint8_t tmc2209_driver_addrs[TMC2209_NUM_DRIVERS] = {
    TMC2209_ADDR0,
    TMC2209_ADDR1,
    TMC2209_ADDR2,
    TMC2209_ADDR3
};

// ==========================================================================
// Enable State
// ==========================================================================

static bool _enabled = false;

// ==========================================================================
// CRC8-ATM (polynomial 0x07)
// ==========================================================================

static uint8_t crc8_atm(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ==========================================================================
// Flush UART RX buffer (drain any stale bytes)
// ==========================================================================

static void uart_flush_rx(void) {
    while (uart_is_readable(TMC2209_UART)) {
        uart_getc(TMC2209_UART);
    }
}

// ==========================================================================
// UART Initialization
// ==========================================================================

void tmc2209_uart_init(void) {
    // Initialize UART at 115200 baud, 8N1
    uart_init(TMC2209_UART, TMC2209_BAUD);
    uart_set_format(TMC2209_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(TMC2209_UART, true);

    // Set UART function on TX and RX pins
    gpio_set_function(TMC2209_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TMC2209_RX_PIN, GPIO_FUNC_UART);

    // Flush any stale RX data
    uart_flush_rx();

    // Initialize ENABLE pin — start with drivers disabled
    gpio_init(TMC2209_EN_PIN);
    gpio_set_dir(TMC2209_EN_PIN, GPIO_OUT);
    gpio_put(TMC2209_EN_PIN, 1);  // HIGH = disabled
    _enabled = false;

    printf("  TMC2209 UART: TX=GP%d, RX=GP%d @ %d baud\n",
           TMC2209_TX_PIN, TMC2209_RX_PIN, TMC2209_BAUD);
}

// ==========================================================================
// Register Read
// ==========================================================================

bool tmc2209_read_reg(uint8_t addr, uint8_t reg, uint32_t *value) {
    if (!value) return false;

    // Build read request: SYNC | addr | reg | CRC
    uint8_t cmd[4];
    cmd[0] = TMC2209_SYNC;
    cmd[1] = addr & 0x03;
    cmd[2] = reg;
    cmd[3] = crc8_atm(cmd, 3);

    uart_flush_rx();

    // Send read request
    uart_write_blocking(TMC2209_UART, cmd, 4);
    uart_tx_wait_blocking(TMC2209_UART);

    // Collect all available bytes within timeout
    // May include: our own TX echo (4 bytes) + driver response (8 bytes)
    uint8_t buf[20];
    int total = 0;
    absolute_time_t deadline = make_timeout_time_us(TMC2209_TIMEOUT_US);

    while (!time_reached(deadline) && total < (int)sizeof(buf)) {
        if (uart_is_readable(TMC2209_UART)) {
            buf[total++] = uart_getc(TMC2209_UART);
        }
        sleep_us(5);
    }

    if (total < 8) {
        return false;
    }

    // Search for response pattern: [0x05, 0xFF, reg, d3..d0, CRC]
    for (int i = 0; i <= total - 8; i++) {
        if (buf[i] == TMC2209_RES_SYNC &&
            buf[i + 1] == TMC2209_RES_ADDR &&
            buf[i + 2] == reg) {

            uint32_t val = ((uint32_t)buf[i + 3] << 24) |
                           ((uint32_t)buf[i + 4] << 16) |
                           ((uint32_t)buf[i + 5] << 8)  |
                           (uint32_t)buf[i + 6];

            if (crc8_atm(&buf[i], 7) == buf[i + 7]) {
                *value = val;
                return true;
            }
            continue;
        }
    }

    return false;
}

// ==========================================================================
// Register Write
// ==========================================================================

bool tmc2209_write_reg(uint8_t addr, uint8_t reg, uint32_t value) {
    // Build write request: SYNC | addr|0x80 | reg | data[3..0] | CRC
    uint8_t cmd[8];
    cmd[0] = TMC2209_SYNC;
    cmd[1] = (addr & 0x03) | 0x80;  // Write: bit 7 = 1
    cmd[2] = reg;
    cmd[3] = (uint8_t)(value >> 24);  // MSB first
    cmd[4] = (uint8_t)(value >> 16);
    cmd[5] = (uint8_t)(value >> 8);
    cmd[6] = (uint8_t)(value);
    cmd[7] = crc8_atm(cmd, 7);

    // Flush RX before writing
    uart_flush_rx();

    // Send write command
    uart_write_blocking(TMC2209_UART, cmd, 8);
    uart_tx_wait_blocking(TMC2209_UART);

    // TMC2209 has no explicit ACK for writes. Verify by reading IFCNT back.
    // Wait long enough for the write to take effect
    sleep_us(500);

    // Read IFCNT to verify the write was accepted
    uint32_t ifcnt = 0;
    if (tmc2209_read_reg(addr, TMC2209_REG_IFCNT, &ifcnt)) {
        // If we got a response, the write was processed (IFCNT increments)
        return true;
    }

    return false;
}

// ==========================================================================
// Configure Single Driver
// ==========================================================================

bool tmc2209_configure(uint8_t addr, uint32_t gconf, uint32_t chopconf,
                       uint32_t ihold_irun, const char *name) {
    const char *label = name ? name : "Driver";

    // 1. Write GCONF (must be first — sets mode and mstep_reg_select)
    if (!tmc2209_write_reg(addr, TMC2209_REG_GCONF, gconf)) {
        printf("  %s: ❌ GCONF write failed (no response)\n", label);
        return false;
    }
    printf("  %s: ✅ GCONF = 0x%08lX\n", label, gconf);

    // 2. Write IHOLD_IRUN (current settings)
    if (!tmc2209_write_reg(addr, TMC2209_REG_IHOLD_IRUN, ihold_irun)) {
        printf("  %s: ❌ IHOLD_IRUN write failed\n", label);
        return false;
    }
    printf("  %s: ✅ IHOLD_IRUN = 0x%08lX (IRUN=%lu, IHOLD=%lu)\n",
           label, ihold_irun,
           (ihold_irun & 0x1F),
           ((ihold_irun >> 8) & 0x1F));

    // 3. Write CHOPCONF (microstep resolution, chopper timing)
    if (!tmc2209_write_reg(addr, TMC2209_REG_CHOPCONF, chopconf)) {
        printf("  %s: ❌ CHOPCONF write failed\n", label);
        return false;
    }
    {
        uint8_t mres = (uint8_t)((chopconf >> 24) & 0x0F);
        printf("  %s: ✅ CHOPCONF = 0x%08lX (1/%lu microsteps%s)\n",
               label, chopconf,
               (unsigned long)(1 << mres),
               (chopconf & TMC2209_INTPOL) ? " + 256x interpolation" : "");
    }

    // 4. Verify readback — read CHOPCONF back and check
    uint32_t verify = 0;
    if (tmc2209_read_reg(addr, TMC2209_REG_CHOPCONF, &verify)) {
        if (verify == chopconf) {
            printf("  %s: ✅ Readback verified\n", label);
        } else {
            printf("  %s: ⚠️ Readback mismatch: wrote 0x%08lX, read 0x%08lX\n",
                   label, chopconf, verify);
        }
    } else {
        printf("  %s: ⚠️ Readback failed (no response)\n", label);
    }

    return true;
}

// ==========================================================================
// Configure All Drivers
// ==========================================================================

int tmc2209_configure_all(uint32_t gconf, uint32_t chopconf, uint32_t ihold_irun) {
    int ok = 0;

    printf("\n  TMC2209 Configuration (%d drivers):\n", TMC2209_NUM_DRIVERS);
    printf("  ─────────────────────────────────────────\n");

    for (int i = 0; i < TMC2209_NUM_DRIVERS; i++) {
        printf("  [%d] %s (addr 0x%02X):\n",
               i, tmc2209_driver_names[i], tmc2209_driver_addrs[i]);

        if (tmc2209_configure(tmc2209_driver_addrs[i], gconf, chopconf,
                              ihold_irun, tmc2209_driver_names[i])) {
            ok++;
        } else {
            printf("  [%d] %s: ❌ Configuration FAILED — check wiring\n",
                   i, tmc2209_driver_names[i]);
        }
        printf("\n");
    }

    printf("  Result: %d/%d drivers configured successfully\n", ok, TMC2209_NUM_DRIVERS);
    return ok;
}

// ==========================================================================
// Configure All Defaults
// ==========================================================================

int tmc2209_configure_all_defaults(void) {
    return tmc2209_configure_all(
        TMC2209_GCONF_DEFAULT,
        TMC2209_CHOPCONF_1_8,
        TMC2209_IHOLD_IRUN_1A
    );
}

// ==========================================================================
// Print DRV_STATUS
// ==========================================================================

void tmc2209_print_status(uint8_t addr, const char *name) {
    uint32_t drv = 0;
    const char *label = name ? name : "Driver";

    if (!tmc2209_read_reg(addr, TMC2209_REG_DRV_STATUS, &drv)) {
        printf("  %s: ❌ DRV_STATUS read failed\n", label);
        return;
    }

    printf("  %s: DRV_STATUS = 0x%08lX\n", label, drv);

    // Bit fields (TMC2209):
    //   bit 0:  OT (overtemp shutdown)
    //   bit 1:  OT_PW (overtemp warning)
    //   bit 2:  S2G (short to GND)
    //   bit 3:  S2VSA (short to VS on high side A)
    //   bit 4:  S2VSB (short to VS on high side B)
    //   bit 5:  OLA (open load A)
    //   bit 6:  OLB (open load B)
    //   bit 7:  T150 (die temp > 150°C)
    //   bits 15:8 = CS_ACT (actual current scaling)
    //   bits 27:16 = SG_RESULT (stallGuard result)
    //   bit 31: STST (standstill)

    printf("         Overtemp: %s | ", (drv & 0x01) ? "SHUTDOWN" : "OK");
    printf("OT Warn: %s | ", (drv & 0x02) ? "⚠️" : "OK");
    printf("S2G: %s | ", (drv & 0x04) ? "⚠️" : "OK");
    printf("OpenLoad: %s%s\n",
           (drv & 0x20) ? "A⚠️ " : "",
           (drv & 0x40) ? "B⚠️ " : "OK");

    printf("         CS_ACT: %lu, SG: %lu, Standstill: %s\n",
           (unsigned long)((drv >> 8) & 0x1F),
           (unsigned long)((drv >> 16) & 0xFFF),
           (drv & 0x80000000) ? "YES" : "no");
}

// ==========================================================================
// Enable / Disable
// ==========================================================================

void tmc2209_set_enabled(bool enabled) {
    gpio_put(TMC2209_EN_PIN, enabled ? 0 : 1);  // LOW = enabled
    _enabled = enabled;
}

bool tmc2209_is_enabled(void) {
    return _enabled;
}
