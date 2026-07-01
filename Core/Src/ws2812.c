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
 * next refresh. Single-char keeps the link trivial to parse and easy to debug
 * in a serial monitor.
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
#include "usart.h"

void leds_off(void)
{
    usart10_write((const uint8_t *)"O\n", 2);
}

void leds_blue(void)
{
    usart10_write((const uint8_t *)"B\n", 2);
}

void leds_yellow(void)
{
    usart10_write((const uint8_t *)"Y\n", 2);
}
