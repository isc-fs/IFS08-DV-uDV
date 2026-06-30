/**
 ******************************************************************************
 * @file    ami_task.c
 * @brief   AMI FreeRTOS task: WS2812 LED colour from the mission index.
 *
 * Body extracted verbatim from freertos.c (StartAmiTask) so the CubeMX-managed
 * file stays thin. The task is still created and scheduled by MX_FREERTOS_Init().
 ******************************************************************************
 */
#include "ami_task.h"

#include "FreeRTOS.h"
#include "task.h"          /* ulTaskNotifyTake / pdTRUE / pdMS_TO_TICKS */
#include "cmsis_os.h"
#include "main.h"
#include "spi.h"           /* hspi1 */
#include "ws2812.h"
#include "can_globals.h"   /* can_c_get_mission_index */

void StartAmiTask(void *argument)
{
  ws2812_init(&hspi1);

  /* Idle demo: dim white to show the node is alive */
  ws2812_set_all(20, 20, 20);
  ws2812_show();

  uint8_t last_mission = 0xFF;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

    uint8_t current = (uint8_t)can_c_get_mission_index();

    if (current != last_mission)
    {
      if (current == 0xFF)
      {
        ws2812_set_all(20, 20, 20);
        ws2812_show();
      }
      else
      {
        ws2812_set_mission_color(current);
      }
      last_mission = current;
    }
  }
}
