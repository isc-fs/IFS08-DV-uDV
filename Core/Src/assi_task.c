/**
 * ASSI task — controls the on-board WS2812 status LEDs according to AS mode.
 *
 * This file implements a simple FreeRTOS task that exposes a setter API to
 * change the AS mode. The task applies the corresponding color/flash pattern
 * to the WS2812 strip using the existing ws2812 driver.
 */

#include "assi_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "ws2812.h"
#include <stdbool.h>

static volatile assi_mode_t current_mode = AS_MODE_OFF;

void assi_set_mode(assi_mode_t mode)
{
    taskENTER_CRITICAL();
    current_mode = mode;
    taskEXIT_CRITICAL();
}

assi_mode_t assi_get_mode(void)
{
    assi_mode_t m;
    taskENTER_CRITICAL();
    m = current_mode;
    taskEXIT_CRITICAL();
    return m;
}

/* Flash timing (FS-Rules T14.9.1: 2-5 Hz at 50 % duty). One loop pass per
 * half-period: 250 ms on + 250 ms off => 2 Hz. Bench-validated rate (the
 * Arduino bridge renders the toggle cleanly here; the earlier 150 ms / 3.3 Hz
 * looked erratic on the real strip). The SAME constant is used for both
 * phases (and for the steady states' refresh), so the duty cycle is 50 % by
 * construction — if you change the rate, change only this constant, never one
 * phase's delay alone. */
#define ASSI_HALF_PERIOD_MS  250u

void StartAssiTask(void *argument)
{
    (void)argument;

    bool flash_state = false;

    for (;;)
    {
        assi_mode_t mode = assi_get_mode();

        switch (mode)
        {
            case AS_MODE_OFF:
                leds_off();
                break;

            case AS_MODE_READY:
                /* Yellow continuous */
                leds_yellow();
                break;

            case AS_MODE_DRIVING:
                /* Yellow flashing, 3.3 Hz / 50 % duty */
                flash_state = !flash_state;
                if (flash_state) leds_yellow();
                else             leds_off();
                break;

            case AS_MODE_EMERGENCY:
                /* Blue flashing, 3.3 Hz / 50 % duty */
                flash_state = !flash_state;
                if (flash_state) leds_blue();
                else             leds_off();
                break;

            case AS_MODE_FINISHED:
                /* Blue continuous */
                leds_blue();
                break;

            default:
                leds_off();
                break;
        }

        /* Single delay shared by every state: steady modes just re-send
         * their colour (a dropped UART byte self-heals), flashing modes
         * advance one half-period. */
        osDelay(ASSI_HALF_PERIOD_MS);
    }
}
