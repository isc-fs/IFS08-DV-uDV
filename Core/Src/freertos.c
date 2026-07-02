/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ros_task.h"

#include "imu_service.h"
#include "can_globals.h"
#include "i2c.h"
#include "cordic.h"
#include "tim.h"
#include "safety_monitor.h"
#include "dwt_time.h"
#include "imu_task.h"
#include "usart.h"
#include "assi_task.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_QUEUE_DEPTH 16   /* depth of imuQueueHandle (IMU task -> ROS task) */

/* ROS-task tuning macros moved to ros_task.c; dwt_micros() to dwt_time.c. */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Safety supervisor: highest app priority so it still runs (and refreshes
 * the IWDG / detects stalls) under load. Yields via osDelay each cycle. */
osThreadId_t safetyTaskHandle;
const osThreadAttr_t safetyTask_attributes = {
  .name = "safetyTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Application state machine (AS state machine + EBS init sequence). Loops
 * ~1 ms; monitored by the safety supervisor via SAFETY_TASK_APP. */
osThreadId_t appTaskHandle;
const osThreadAttr_t appTask_attributes = {
  .name = "appTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t assiTaskHandle;
const osThreadAttr_t assiTask_attributes = {
  .name = "assiTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osMessageQueueId_t imuQueueHandle;
osMessageQueueId_t canRxQueueHandle;
osMessageQueueId_t resRxQueueHandle;
osMessageQueueId_t debugQueueHandle;
osSemaphoreId_t imuSemHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartImuTask(void *argument);
void StartCanTask(void *argument);
void StartAppTask(void *argument);   /* state machine, in app_task.cpp */
void StartAssiTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  imuSemHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  imuQueueHandle   = osMessageQueueNew(IMU_QUEUE_DEPTH, sizeof(imu_sample_t), NULL);
  canRxQueueHandle = osMessageQueueNew(32, sizeof(can_msg_t), NULL);
  resRxQueueHandle = osMessageQueueNew(8, sizeof(can_msg_t), NULL);
  debugQueueHandle = osMessageQueueNew(8, 128, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);
  canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);
  appTaskHandle = osThreadNew(StartAppTask, NULL, &appTask_attributes);
  assiTaskHandle = osThreadNew(StartAssiTask, NULL, &assiTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* USART10 (ASSI LED bridge to the Arduino) is initialized in main(). */
  //assi_set_mode(AS_MODE_DRIVING);

  ros_task_run();   /* micro-ROS node; defined in ros_task.c; never returns */
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask; (void)pcTaskName;
  // Fatal: fire the EBS (D1/D2 are the EBS actuators; LOW = fire, the
  // fail-safe level) and hang so the IWDG resets us into the latched
  // safe state.
  HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
  for (;;) {}
}

/* StartImuTask is defined in imu_task.c */
/* StartCanTask is defined in can_task.cpp (extern "C") */

/* USER CODE END Application */

