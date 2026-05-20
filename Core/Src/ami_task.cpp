/**
 * @file    ami_task.cpp
 * @brief   AMI task implementation - WS2812 mission indication
 */

#include "ami_task.h"

#include <cstdint>

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "cmsis_os.h"
    #include "spi.h"
    #include "ws2812.h"
}

#include "can_globals.h"

void StartAmiTask(void *argument)
{
    (void)argument;

    ws2812_init(&hspi1);

    /* Idle demo: dim white to show the node is alive */
    ws2812_set_all(20, 20, 20);
    ws2812_show();

    int last_mission = -1;

    for (;;)
    {
        /* Wait for mission updates from canTask; timeout allows periodic refresh. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

        int current = g_can_mission_id.load();

        if (current != last_mission)
        {
            if (current <= 0)
            {
                /* No mission selected: keep idle dim white */
                ws2812_set_all(20, 20, 20);
                ws2812_show();
            }
            else
            {
                ws2812_set_mission_color((uint8_t)current);
            }
            last_mission = current;
        }
    }
}
