#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include <stdbool.h>

#define WS2812_PIN      23
#define WS2812_NUM_LEDS 8
#define WS2812_BRIGHTNESS 32

void ws2812_init(void);
void ws2812_set_pixel(unsigned int index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_brightness(uint8_t brightness);
void ws2812_show(void);
void ws2812_clear(void);

#endif
