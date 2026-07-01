#ifndef ASSI_H
#define ASSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * ASSI (Autonomous System Status Indicator) LED driver — FS-Rules T14.9.
 *
 * The ASSI LEDs are WS2812-family addressable RGB LEDs on a single data line
 * driven from D5 / PB8, level-shifted 3.3V -> 5V by the SN74AHCT125 buffer
 * (net ASSI_DIN). PB8 is a plain GPIO (no SPI/timer reaches it), so the frame
 * is bit-banged with cycle-accurate timing from the DWT counter.
 *
 * Colours/flashing are decided by the AS state machine (app_task.cpp); this
 * file is only the transport. Nothing else in the firmware drives the LEDs.
 * ------------------------------------------------------------------------- */

/* Number of daisy-chained ASSI LEDs on the D5/PB8 data line.
 * The rules (T14.9.2) require three ASSIs; on this car they share one data
 * line, so a chain of 3 would all show the same colour. Only 1 is populated
 * for now — change this single value (or override with -DASSI_NUM_LEDS=N at
 * build time) when the full chain is wired. */
#ifndef ASSI_NUM_LEDS
#define ASSI_NUM_LEDS 1
#endif

/* Initialise the DWT cycle counter and drive the chain to OFF. */
void assi_init(void);

/* Set every LED in the chain to the same colour (latched on the next show). */
void assi_set_all(uint8_t r, uint8_t g, uint8_t b);

/* Clock the current pixel buffer out on PB8 (blocking, ~1.25us per bit +
 * >50us reset; IRQs masked during the frame). */
void assi_show(void);

/* Convenience: all LEDs black + show. */
void assi_off(void);

#ifdef __cplusplus
}
#endif

#endif /* ASSI_H */
