/**
 ******************************************************************************
 * @file    ws2812.h
 * @brief   WS2812 RGB LED driver (SPI bit-banged)
 ******************************************************************************
 */
#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define WS2812_NUM_LEDS 40

/**
 * @brief Initialize WS2812 driver with SPI handle
 * @param hspi Pointer to SPI_HandleTypeDef (SPI1)
 */
void ws2812_init(SPI_HandleTypeDef *hspi);

/**
 * @brief Set individual LED color (R, G, B)
 * @param index LED index (0-7)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void ws2812_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set all LEDs to the same color
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Send pixel buffer to LEDs via SPI
 */
void ws2812_show(void);

/**
 * @brief Clear all LEDs (turn off)
 */
void ws2812_clear(void);

/**
 * @brief Set all LEDs to mission-specific color
 * @param mission_index Mission index (0-9)
 */
void ws2812_set_mission_color(uint8_t mission_index);

#endif /* WS2812_H */
