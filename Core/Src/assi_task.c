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
                osDelay(200);
                break;

            case AS_MODE_READY:
                /* Yellow continuous */
                leds_yellow();
                osDelay(200);
                break;

            case AS_MODE_DRIVING:
                /* Yellow flashing */
                flash_state = !flash_state;
                if (flash_state) leds_yellow();
                else             leds_off();
                osDelay(500);
                break;

            case AS_MODE_EMERGENCY:
                /* Blue flashing (faster) */
                flash_state = !flash_state;
                if (flash_state) leds_blue();
                else             leds_off();
                osDelay(250);
                break;

            case AS_MODE_FINISHED:
                /* Blue continuous */
                leds_blue();
                osDelay(200);
                break;

            default:
                leds_off();
                osDelay(200);
                break;
        }
    }
}
