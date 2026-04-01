#include "ws2812.h"
#include <string.h>

static uint8_t spi_buffer[BUFFER_SIZE];
static SPI_HandleTypeDef *_hspi;
static RGB_t leds[NUM_LEDS];

static void encode_byte(uint8_t byte, uint8_t *out) {
    for (int bit = 7; bit >= 0; bit--) {
        *out++ = (byte & (1 << bit)) ? WS2812_ONE : WS2812_ZERO;
    }
}

void WS2812_Init(SPI_HandleTypeDef *hspi) {
    _hspi = hspi;
    memset(spi_buffer, 0, sizeof(spi_buffer));
    memset(leds, 0, sizeof(leds));
}

void WS2812_SetLED(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= NUM_LEDS) return;
    leds[index].r = r;
    leds[index].g = g;
    leds[index].b = b;
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].r = r;
        leds[i].g = g;
        leds[i].b = b;
    }
}

void WS2812_Show(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t *p = &spi_buffer[i * BYTES_PER_LED];
        encode_byte(leds[i].g, p + 0);
        encode_byte(leds[i].r, p + 8);
        encode_byte(leds[i].b, p + 16);
    }
    HAL_SPI_Transmit(_hspi, spi_buffer, BUFFER_SIZE, HAL_MAX_DELAY);
}

void WS2812_Clear(void) {
    memset(leds, 0, sizeof(leds));
    WS2812_Show();
}