/**
 * @file bb_spi.h
 * @brief Fresh bit-bang SPI slave library (written from scratch, no reuse).
 *
 * Emulates SPI mode 0 (CPOL=0, CPHA=0) on arbitrary GPIO pins, driven by the
 * master's SCK. The master toggles CS per byte (CS-per-byte framing); between
 * bytes CS idles high (~500us), between frames CS idles high ~50ms.
 *
 * Timing contract (matched to the master):
 *   - SCK idles LOW.
 *   - Master drives MOSI on SCK falling edge.
 *   - Master samples MISO on SCK rising edge.
 *   - Slave must drive MISO while SCK is LOW, before the rising edge.
 *   - Slave samples MOSI after SCK goes HIGH (master's data is settled).
 *
 * Frame resync: if CS stays high longer than BB_FRAME_GAP_US, it is the
 * inter-frame gap, not an inter-byte gap — the next CS-low starts byte 0.
 */

#ifndef BB_SPI_H
#define BB_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define BB_FRAME_GAP_US  2000u   // >2ms CS-high = frame boundary

typedef struct {
    uint sck;    // input  (from master)
    uint mosi;   // input  (from master)
    uint miso;   // output (to master, tri-state when CS high)
    uint cs;     // input  (active low)
} bb_spi_pins_t;

/**
 * Receive one full frame of `len` bytes (CS-per-byte). Each received byte is
 * stored in rx[i]; the byte tx[i] is transmitted back during the same
 * transaction (full duplex). MISO is tri-stated whenever CS is high so the
 * wire can be shared with other slaves.
 *
 * Returns true if a complete frame was received, false if CS never asserted.
 */
static inline bool bb_spi_slave_frame(const bb_spi_pins_t *p,
                                      const uint8_t *tx, uint8_t *rx, int len)
{
    for (int i = 0; i < len; i++) {
        // ---- wait for CS low (start of byte); resync on long CS-high ----
        {
            uint64_t t0 = time_us_64();
            while (gpio_get(p->cs)) {
                // Not selected: MISO must be high-Z (shared wire).
                gpio_set_dir(p->miso, GPIO_IN);
                gpio_put(p->miso, 0);
                tight_loop_contents();
            }
            if (time_us_64() - t0 > BB_FRAME_GAP_US) {
                i = -1;               // inter-frame gap -> byte 0 next
                continue;
            }
        }

        // ---- clock out/in one byte, MSB first (SPI mode 0) ----
        gpio_set_dir(p->miso, GPIO_OUT);
        uint8_t in = 0;
        for (int bit = 7; bit >= 0; bit--) {
            // wait for SCK low (falling edge / idle low)
            while (gpio_get(p->sck)) { tight_loop_contents(); }
            // drive MISO while SCK is low -> stable before the rising edge
            gpio_put(p->miso, (tx[i] >> bit) & 1u);
            // wait for SCK high (rising edge)
            while (!gpio_get(p->sck)) { tight_loop_contents(); }
            // master's MOSI is settled now -> sample it
            if (gpio_get(p->mosi)) in |= (1u << bit);
        }
        rx[i] = in;

        // ---- wait for CS high (end of byte), then release MISO ----
        {
            uint64_t t0 = time_us_64();
            while (!gpio_get(p->cs)) { tight_loop_contents(); }
            if (time_us_64() - t0 > BB_FRAME_GAP_US) {
                i = -1;               // inter-frame gap -> byte 0 next
            }
        }
        gpio_set_dir(p->miso, GPIO_IN);
        gpio_put(p->miso, 0);
    }
    return true;
}

/**
 * Initialise the slave SPI pins for bit-bang use.
 * - sck, mosi, cs: inputs (cs gets a pull-up so it holds deasserted high)
 * - miso: input/tri-state until a byte is being exchanged
 */
static inline void bb_spi_slave_init(const bb_spi_pins_t *p)
{
    gpio_init(p->sck);  gpio_set_dir(p->sck,  GPIO_IN);
    gpio_init(p->mosi); gpio_set_dir(p->mosi, GPIO_IN);
    gpio_init(p->cs);   gpio_set_dir(p->cs,   GPIO_IN);
    gpio_pull_up(p->cs);
    gpio_init(p->miso); gpio_set_dir(p->miso, GPIO_IN);
    gpio_put(p->miso, 0);
}

#endif // BB_SPI_H
