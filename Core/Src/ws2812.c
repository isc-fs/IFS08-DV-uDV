/**
 ******************************************************************************
 * @file    ws2812.c
 * @brief   ASSI LED commands over USART10 to an Arduino Nano.
 *
 * Wire protocol (STM32H7 -> Arduino Nano, USART10 TX, 115200 8N1):
 *   one ASCII command character, newline-terminated for framing:
 *     'O' + '\n'  ->  LEDs off
 *     'B' + '\n'  ->  LEDs blue
 *     'Y' + '\n'  ->  LEDs yellow
 *
 * The Arduino reads a line and drives the physical WS2812 strip. Commands are
 * (re)sent every ASSI refresh (~2-5 Hz), so a dropped byte self-heals on the
 * next refresh. Single-char keeps the link trivial to parse and easy to debug.
 *
 * USART10 is configured by CubeMX (MX_USART10_UART_Init, huart10). Transmit is
 * the HAL blocking call; at 115200 two bytes take ~0.17 ms, negligible in the
 * ASSI task, and it does not disable interrupts.
 *
 * Arduino side (sketch), for reference:
 *   void loop() {
 *     if (Serial.available()) {
 *       char c = Serial.read();
 *       if      (c == 'O') setStrip(0, 0, 0);
 *       else if (c == 'B') setStrip(0, 0, 255);
 *       else if (c == 'Y') setStrip(255, 255, 0);
 *     }
 *   }
 ******************************************************************************
 */
#include "ws2812.h"
#include "usart.h"   /* huart10 (CubeMX) */

#define LEDS_TX_TIMEOUT_MS  10U

static void leds_send(const char *cmd /* 2 bytes: letter + '\n' */)
{
    HAL_UART_Transmit(&huart10, (const uint8_t *)cmd, 2U, LEDS_TX_TIMEOUT_MS);
}

void leds_off(void)
{
    leds_send("a\n");
}

void leds_blue(void)
{
    leds_send("c\n");
}

void leds_yellow(void)
{
    leds_send("b\n");
}
