/**
 ******************************************************************************
 * @file    ami_task.c
 * @brief   AMI FreeRTOS task.
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

void StartAmiTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
  }
}
