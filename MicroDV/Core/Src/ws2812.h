#ifndef WS2812_H
#define WS2812_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define NUM_LEDS        8
#define BYTES_PER_LED   24
#define RESET_BYTES     64
#define BUFFER_SIZE     (NUM_LEDS * BYTES_PER_LED + RESET_BYTES)

#define WS2812_ONE      0xE0   // 0b11100000
#define WS2812_ZERO     0x80   // 0b10000000

typedef struct {
    uint8_t r, g, b;
} RGB_t;

void WS2812_Init(SPI_HandleTypeDef *hspi);
void WS2812_SetLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Show(void);
void WS2812_Clear(void);

#endif