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
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>

#include "imu_service.h"
#include "i2c.h"
#include "cordic.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define G_TO_MS2   9.80665f
#define DPS_TO_RAD (float)(M_PI / 180.0)
#define IMU_QUEUE_DEPTH 16
#define TIME_SYNC_TIMEOUT_MS  1000
#define TIME_SYNC_INTERVAL    4000  // re-sync every N samples (~10s at 400Hz)

// High-resolution microsecond timestamp using DWT cycle counter
static inline uint64_t dwt_micros(void)
{
  return (uint64_t)DWT->CYCCNT / (SystemCoreClock / 1000000U);
}
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

osMessageQueueId_t imuQueueHandle;
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
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void StartImuTask(void *argument);
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
  imuQueueHandle = osMessageQueueNew(IMU_QUEUE_DEPTH, sizeof(imu_sample_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);
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

  // Enable DWT cycle counter for high-res timestamps
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Wait for USB CDC to enumerate on the host
  osDelay(2000);

  // micro-ROS custom transport (USB CDC, HDLC framing)
  rmw_uros_set_custom_transport(
    true,
    NULL,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  // Set FreeRTOS-compatible allocators for micro-ROS
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    for (;;) { osDelay(1000); }
  }

  // micro-ROS node
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // Time synchronization: compute offset between local clock and agent epoch
  // Retry until sync succeeds, then capture the epoch offset
  while (!rmw_uros_epoch_synchronized()) {
    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
    if (!rmw_uros_epoch_synchronized()) {
      osDelay(100);
    }
  }
  // Capture offset: agent_epoch - local_dwt at the sync moment
  int64_t sync_epoch_ns = rmw_uros_epoch_nanos();
  uint64_t sync_dwt_us = dwt_micros();
  int64_t epoch_offset_ns = sync_epoch_ns - (int64_t)sync_dwt_us * 1000LL;

  // IMU publisher (best-effort QoS for maximum throughput)
  rcl_publisher_t imu_pub;
  rclc_publisher_init_best_effort(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw");

  // IMU debug status publisher
  rcl_publisher_t imu_debug_pub;
  std_msgs__msg__Int32 debug_msg;
  debug_msg.data = 0;
  rclc_publisher_init_default(
    &imu_debug_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "imu/status");

  // Prepare IMU message (set static fields once)
  sensor_msgs__msg__Imu imu_msg;
  memset(&imu_msg, 0, sizeof(imu_msg));
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // Orientation not available from complementary filter (no quaternion)
  imu_msg.orientation_covariance[0] = -1.0;

  for (;;)
  {
    imu_sample_t sample;
    if (osMessageQueueGet(imuQueueHandle, &sample, NULL, osWaitForever) == osOK)
    {
      // Timestamp: DWT capture moment + epoch offset from sync
      int64_t stamp_ns = epoch_offset_ns + (int64_t)sample.timestamp_us * 1000LL;
      imu_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
      imu_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

      // Linear acceleration: g -> m/s^2
      imu_msg.linear_acceleration.x = sample.imu.ax_g * G_TO_MS2;
      imu_msg.linear_acceleration.y = sample.imu.ay_g * G_TO_MS2;
      imu_msg.linear_acceleration.z = sample.imu.az_g * G_TO_MS2;

      // Angular velocity: dps -> rad/s
      imu_msg.angular_velocity.x = sample.imu.gx_dps * DPS_TO_RAD;
      imu_msg.angular_velocity.y = sample.imu.gy_dps * DPS_TO_RAD;
      imu_msg.angular_velocity.z = sample.imu.gz_dps * DPS_TO_RAD;

      (void)rcl_publish(&imu_pub, &imu_msg, NULL);

      // Periodic time re-sync and debug publish (~every 10s)
      static uint16_t sync_counter = 0;
      if (++sync_counter >= TIME_SYNC_INTERVAL)
      {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        // Refresh epoch offset
        int64_t new_epoch_ns = rmw_uros_epoch_nanos();
        uint64_t new_dwt_us = dwt_micros();
        epoch_offset_ns = new_epoch_ns - (int64_t)new_dwt_us * 1000LL;

        extern volatile int32_t imu_debug_status;
        debug_msg.data = imu_debug_status;
        (void)rcl_publish(&imu_debug_pub, &debug_msg, NULL);
        sync_counter = 0;
      }
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Shared debug status visible from defaultTask via orientation.w
volatile int32_t imu_debug_status = -99;

void StartImuTask(void *argument)
{
  extern I2C_HandleTypeDef hi2c2;
  extern CORDIC_HandleTypeDef hcordic;

  imu_service_t imu_svc;
  imu_service_init(&imu_svc, &hi2c2,
                   IMU_SCL_GPIO_Port, IMU_SCL_Pin,
                   IMU_SDA_GPIO_Port, IMU_SDA_Pin,
                   &hcordic, 0.98f);
  bmi088_status_t init_st = imu_service_start(&imu_svc);
  imu_debug_status = (int32_t)init_st;

  // Start TIM2 interrupt for deterministic 400Hz sampling
  HAL_TIM_Base_Start_IT(&htim2);

  for (;;)
  {
    // Wait for TIM2 ISR to release the semaphore (400Hz, zero jitter)
    osSemaphoreAcquire(imuSemHandle, osWaitForever);

    // Capture high-res timestamp at exact sampling moment
    uint64_t ts_us = dwt_micros();

    imu_sample_t sample;
    bmi088_status_t step_st = imu_service_step(&imu_svc, &sample);
    imu_debug_status = (int32_t)step_st;
    if (step_st == BMI088_OK)
    {
      sample.timestamp_us = ts_us;
      osMessageQueuePut(imuQueueHandle, &sample, 0, 0);
    }
  }
}

/* USER CODE END Application */

