#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define WS2812_NUM_LEDS  8

void ws2812_init(SPI_HandleTypeDef *hspi);
void ws2812_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);
void ws2812_show(void);
void ws2812_clear(void);

/* Convenience: look up mission index (0–9) and update all LEDs */
void ws2812_set_mission_color(uint8_t mission_index);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
