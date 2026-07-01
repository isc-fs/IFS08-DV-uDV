/**
 ******************************************************************************
 * @file    ws2812.h
 * @brief   ASSI LED commands sent over USART10 to an Arduino Nano.
 *
 * The STM32 no longer drives the WS2812 strip directly. It sends a one-byte
 * command to an Arduino Nano over USART10; the Arduino renders the color on
 * the physical strip. See ws2812.c for the wire protocol.
 ******************************************************************************
 */
#ifndef WS2812_H
#define WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Tell the Arduino to turn the ASSI LEDs off. */
void leds_off(void);

/** @brief Tell the Arduino to set the ASSI LEDs blue. */
void leds_blue(void);

/** @brief Tell the Arduino to set the ASSI LEDs yellow. */
void leds_yellow(void);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
