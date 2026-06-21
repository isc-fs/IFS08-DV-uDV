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
#include <std_srvs/srv/empty.h>
#include <std_srvs/srv/set_bool.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>

#include "imu_service.h"
#include "can_globals.h"
#include "ws2812.h"
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
#define SLOW_PUB_INTERVAL     40    // publish steering/RES/AMI every 40 IMU samples (~10 Hz)
#define DL_TX_INTERVAL_MS     100   // Data Logger TX period
#define RES_TIMEOUT_MS        150   // RES PDO timeout (expect every 30 ms)

// High-resolution microsecond timestamp using DWT cycle counter (wrap-safe)
static uint32_t dwt_last = 0;
static uint64_t dwt_overflow_count = 0;

static inline uint64_t dwt_micros(void)
{
  uint32_t now = DWT->CYCCNT;
  if (now < dwt_last) dwt_overflow_count++;
  dwt_last = now;
  uint64_t total_cycles = (dwt_overflow_count << 32) | (uint64_t)now;
  return total_cycles / (SystemCoreClock / 1000000U);
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

osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t amiTaskHandle;
const osThreadAttr_t amiTask_attributes = {
  .name = "amiTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osMessageQueueId_t imuQueueHandle;
osMessageQueueId_t canRxQueueHandle;
osMessageQueueId_t resRxQueueHandle;
osMessageQueueId_t debugQueueHandle;
osSemaphoreId_t imuSemHandle;

/* Mission index shared between canTask (writer) and amiTask (reader) */
//volatile uint8_t g_mission_index = 0xFF;  /* 0xFF = no mission received */
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
void StartCanTask(void *argument);
void StartAmiTask(void *argument);
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
  amiTaskHandle = osThreadNew(StartAmiTask, NULL, &amiTask_attributes);
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

/* --------------------------------------------------------------------------
 * /cmd_test subscriber callback: toggle OK_STATUS LED (PD14)
 * -------------------------------------------------------------------------- */
static void cmd_test_callback(const void *msgin)
{
  (void)msgin;
  HAL_GPIO_TogglePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin);
}

// --- Service callback for /activate_steering ---

void activate_steering_callback(const void * req, void * res)
{
  // Casteamos los punteros a los tipos correctos de SetBool
  const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

  char srv_msg[128];
  
  // Evaluamos el bool que nos llega en request->data
  if (request->data) {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Servicio llamado con TRUE. Activando modo especial.");
    // Ejemplo: Encender un LED o activar una bandera interna
    HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
  } else {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Servicio llamado con FALSE. Desactivando modo especial.");
    HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_RESET);
  }
  
  // Enviamos el mensaje al queue de debug
  osMessageQueuePut(debugQueueHandle, &srv_msg, 0, 0);
  
  // Respondemos al cliente de ROS 2 que todo ha salido bien
  response->success = true;
  // Opcional: rellenar el mensaje de retorno (requiere inicializar el string de la respuesta si se usa)
  // rosidl_runtime_c__String__assign(&response->message, "OK");
}

// --- Service callback for /force_ebs ---

void force_ebs_callback(const void * req, void * res)
{
  // Casteamos los punteros a los tipos correctos de SetBool
  const std_srvs__srv__SetBool_Request * request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response * response = (std_srvs__srv__SetBool_Response *)res;

  char srv_msg[128];
  
  // Evaluamos el bool que nos llega en request->data
  if (request->data) {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Forzando apertura de EBS");
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET); // Forzar EBS abierto
    HAL_GPIO_WritePin(D1_GPIO_Port, D2_Pin, GPIO_PIN_SET); // Forzar EBS abierto
  } else {
    snprintf(srv_msg, sizeof(srv_msg), "debug: Vuelta a estado normal");
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D1_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
  }
  
  // Enviamos el mensaje al queue de debug
  osMessageQueuePut(debugQueueHandle, &srv_msg, 0, 0);
  
  // Respondemos al cliente de ROS 2 que todo ha salido bien
  response->success = true;
}

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

  // Time synchronization
  while (!rmw_uros_epoch_synchronized()) {
    rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
    if (!rmw_uros_epoch_synchronized()) {
      osDelay(100);
    }
  }
  int64_t sync_epoch_ns = rmw_uros_epoch_nanos();
  uint64_t sync_dwt_us = dwt_micros();
  int64_t epoch_offset_ns = sync_epoch_ns - (int64_t)sync_dwt_us * 1000LL;

  // --- Publishers ---

  // IMU publisher (best-effort QoS for maximum throughput)
  rcl_publisher_t imu_pub;
  rclc_publisher_init_best_effort(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw");

  // IMU debug status publisher
  rcl_publisher_t imu_debug_pub;
  std_msgs__msg__Int32 debug_msg;
  debug_msg.data = 0;
  rclc_publisher_init_default(
    &imu_debug_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "imu/status");

  // Steering angle publisher (~10 Hz)
  rcl_publisher_t steering_pub;
  std_msgs__msg__Float32 steering_msg;
  steering_msg.data = 0.0f;
  rclc_publisher_init_best_effort(
    &steering_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "steering/angle_sensor");

  // RES status publisher (~10 Hz)
  rcl_publisher_t res_pub;
  std_msgs__msg__Int32 res_msg;
  res_msg.data = 0;
  rclc_publisher_init_best_effort(
    &res_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "res/status");

  // AMI mission publisher (~10 Hz)
  rcl_publisher_t ami_pub;
  std_msgs__msg__Int32 ami_msg;
  ami_msg.data = -1;
  rclc_publisher_init_best_effort(
    &ami_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ami/mission");

  // GO signal publisher (~10 Hz): 0=no GO, 1=GO active
  rcl_publisher_t go_pub;
  std_msgs__msg__Int32 go_msg;
  go_msg.data = 0;
  rclc_publisher_init_best_effort(
    &go_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "res/go");

  // Debug string publisher
  rcl_publisher_t debug_pub;
  std_msgs__msg__String debug_str_msg;
  memset(&debug_str_msg, 0, sizeof(debug_str_msg));
  rclc_publisher_init_default(
    &debug_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug");

  // --- Subscriber ---

  // /cmd_test subscriber (toggle LED on receive)
  rcl_subscription_t cmd_test_sub;
  std_msgs__msg__Int32 cmd_test_msg;
  rclc_subscription_init_default(
    &cmd_test_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cmd_test");

  // --- SERVICIOS ---
  rcl_service_t Activate_stearing;
  std_srvs__srv__SetBool_Request act_steer_srv_req = {0};
  std_srvs__srv__SetBool_Response act_steer_srv_res = {0};
  
  rclc_service_init_default(
    &Activate_stearing, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "activate_steering"
  );

  rcl_service_t Force_EBS;
  std_srvs__srv__SetBool_Request force_ebs_srv_req = {0};
  std_srvs__srv__SetBool_Response force_ebs_srv_res = {0};
  
  rclc_service_init_default(
    &Force_EBS, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "force_ebs"
  );

  // --- Executor ---
  // Executor for subscriber callbacks
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 3, &allocator);

  rclc_executor_add_subscription(
    &executor, &cmd_test_sub, &cmd_test_msg,
    &cmd_test_callback, ON_NEW_DATA);

  rclc_executor_add_service(
    &executor, &Activate_stearing, &act_steer_srv_req, &act_steer_srv_res, 
    &activate_steering_callback);

  rclc_executor_add_service(
    &executor, &Force_EBS, &force_ebs_srv_req, &force_ebs_srv_res, 
    &force_ebs_callback);

  // Prepare IMU message (set static fields once)
  sensor_msgs__msg__Imu imu_msg;
  memset(&imu_msg, 0, sizeof(imu_msg));
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // Orientation not available
  imu_msg.orientation_covariance[0] = -1.0;

  // BMI088 datasheet noise specs (diagonal covariance matrices)
  imu_msg.linear_acceleration_covariance[0] = 8.25e-4;
  imu_msg.linear_acceleration_covariance[4] = 8.25e-4;
  imu_msg.linear_acceleration_covariance[8] = 8.25e-4;
  imu_msg.angular_velocity_covariance[0] = 1.37e-5;
  imu_msg.angular_velocity_covariance[4] = 1.37e-5;
  imu_msg.angular_velocity_covariance[8] = 1.37e-5;

  uint16_t slow_pub_counter = 0;

  for (;;)
  {
    imu_sample_t sample;
    if (osMessageQueueGet(imuQueueHandle, &sample, NULL, osWaitForever) == osOK)
    {
      // Timestamp
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

      // --- Slow publishers (~10 Hz) ---
      if (++slow_pub_counter >= SLOW_PUB_INTERVAL)
      {
        slow_pub_counter = 0;

        // Steering angle in degrees
        steering_msg.data = can_c_get_steering_angle_deg();
        (void)rcl_publish(&steering_pub, &steering_msg, NULL);

        // RES status: 0=OK, 1=E-STOP, -1=TIMEOUT
        uint32_t now_tick = osKernelGetTickCount();
        res_msg.data = can_c_get_res_status(now_tick, RES_TIMEOUT_MS);
        (void)rcl_publish(&res_pub, &res_msg, NULL);

        // AMI mission index
        ami_msg.data = can_c_get_mission_index();
        (void)rcl_publish(&ami_pub, &ami_msg, NULL);

        // GO signal from RES
        go_msg.data = (int32_t)can_c_get_go_signal();
        (void)rcl_publish(&go_pub, &go_msg, NULL);

        // Spin executor to process /cmd_test subscription
        // Publish any queued debug messages from CAN service
        char dbgbuf[128];
        while (osMessageQueueGet(debugQueueHandle, &dbgbuf, NULL, 0) == osOK) {
          rosidl_runtime_c__String__assign(&debug_str_msg.data, dbgbuf);
          (void)rcl_publish(&debug_pub, &debug_str_msg, NULL);
        }

        // Heartbeat / periodic debug message
        static uint32_t debug_beat = 0;
        if (debug_beat % 10 == 0) {  // Publish every 10 iterations
          char beatbuf[64];
          snprintf(beatbuf, sizeof(beatbuf), "debug: heartbeat %u", debug_beat++);
          rosidl_runtime_c__String__assign(&debug_str_msg.data, beatbuf);
          (void)rcl_publish(&debug_pub, &debug_str_msg, NULL);
        }

        rclc_executor_spin_some(&executor, 0);
      }

      // Periodic time re-sync and debug publish (~every 10s)
      static uint16_t sync_counter = 0;
      if (++sync_counter >= TIME_SYNC_INTERVAL)
      {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
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

// Shared debug status
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

  // Gyro bias calibration: collect 300 samples over 6s while stationary
  if (init_st == BMI088_OK)
  {
    const int CAL_SAMPLES = 300;
    const float GYR_LSB_PER_DPS = 16.4f;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < CAL_SAMPLES; i++)
    {
      bmi088_raw_t raw;
      if (bmi088_read_raw(&imu_svc.bmi, &raw) == BMI088_OK)
      {
        gx_sum += raw.gx;
        gy_sum += raw.gy;
        gz_sum += raw.gz;
      }
      osDelay(20);
    }

    float gx_bias = (float)gx_sum / (float)CAL_SAMPLES / GYR_LSB_PER_DPS;
    float gy_bias = (float)gy_sum / (float)CAL_SAMPLES / GYR_LSB_PER_DPS;
    float gz_bias = (float)gz_sum / (float)CAL_SAMPLES / GYR_LSB_PER_DPS;
    attitude_set_gyro_bias_dps(&imu_svc.att, gx_bias, gy_bias, gz_bias);
  }

  // Start TIM2 interrupt for deterministic 400Hz sampling
  HAL_TIM_Base_Start_IT(&htim2);

  for (;;)
  {
    osSemaphoreAcquire(imuSemHandle, osWaitForever);

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

/* StartCanTask is defined in can_task.cpp (extern "C") */

void StartAmiTask(void *argument)
{
  extern SPI_HandleTypeDef hspi1;
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

/* USER CODE END Application */
