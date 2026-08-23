#include "ws2812.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// PIO program for WS2812B (800kHz, GRB)
// Auto-pulled, 32 bits per pixel
static const uint16_t ws2812_program_instructions[] = {
    0x7820, // .wrap_target: out x, 20    [T1]
    0x0142, //              jmp !x, done   [T2]
    0x0343, //              jmp x--, ok    [T3]
    0x0143, // done:        jmp x--, reset [T3]
    0x0140, //              jmp 0          [T3]
    0x0042, // ok:          set x, 0       [T2]
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 6,
    .origin = -1,
};

static PIO ws2812_pio;
static uint ws2812_sm;
static uint32_t ws2812_buf[WS2812_NUM_LEDS];
static uint8_t ws2812_brightness = WS2812_BRIGHTNESS;

static inline uint32_t ws2812_encode(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t val = 0;
    // WS2812B: GRB order, MSB first
    for (int i = 7; i >= 0; i--) {
        val |= ((g >> i) & 1) ? (1u << (i * 3 + 2)) : 0;
        val |= ((r >> i) & 1) ? (1u << (i * 3 + 1)) : 0;
        val |= ((b >> i) & 1) ? (1u << (i * 3 + 0)) : 0;
    }
    return val;
}

void ws2812_init(void) {
    ws2812_pio = pio0;
    uint offset = pio_add_program(ws2812_pio, &ws2812_program);
    ws2812_sm = pio_claim_unused_sm(ws2812_pio, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 5);
    sm_config_set_set_pins(&c, WS2812_PIN, 1);
    sm_config_set_out_pins(&c, WS2812_PIN, 1);
    sm_config_set_sideset_pins(&c, WS2812_PIN);

    // 800kHz = 125MHz / (T1+T2+T3) / 4
    // T1=3, T2=3, T3=4 → 125M / (3+3+4) / 4 = 3.125 MHz per bit → 800kHz
    sm_config_set_clkdiv(&c, 1.0f);

    pio_gpio_init(ws2812_pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(ws2812_pio, ws2812_sm, WS2812_PIN, 1, true);
    pio_sm_init(ws2812_pio, ws2812_sm, offset, &c);
    pio_sm_set_enabled(ws2812_pio, ws2812_sm, true);

    // Clear all pixels
    ws2812_clear();
}

void ws2812_set_pixel(unsigned int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < WS2812_NUM_LEDS) {
        // Apply brightness
        r = (r * ws2812_brightness) >> 8;
        g = (g * ws2812_brightness) >> 8;
        b = (b * ws2812_brightness) >> 8;
        ws2812_buf[index] = ws2812_encode(r, g, b);
    }
}

void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t val = ws2812_encode(
        (r * ws2812_brightness) >> 8,
        (g * ws2812_brightness) >> 8,
        (b * ws2812_brightness) >> 8
    );
    for (uint i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_buf[i] = val;
    }
}

void ws2812_set_brightness(uint8_t brightness) {
    ws2812_brightness = brightness;
}

void ws2812_show(void) {
    for (uint i = 0; i < WS2812_NUM_LEDS; i++) {
        pio_sm_put_blocking(ws2812_pio, ws2812_sm, ws2812_buf[i]);
    }
    // Reset: >50µs low
    busy_wait_us(60);
}

void ws2812_clear(void) {
    for (uint i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_buf[i] = 0;
    }
    ws2812_show();
}
