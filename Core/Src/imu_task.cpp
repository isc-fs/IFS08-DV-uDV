/**
 * @file    imu_task.cpp
 * @brief   IMU task implementation - 400Hz BMI088 acquisition with CAN/ROS output
 */

#include "imu_task.h"

#include <cstring>
#include <math.h>

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "cmsis_os.h"
    #include "imu_service.h"
    #include "tim.h"
}

#include "can_interface.hpp"

/* Defines */
#define G_TO_MS2   9.80665f
#define DPS_TO_RAD (float)(M_PI / 180.0)

/* External declarations (defined in freertos.c) */
extern osMessageQueueId_t imuQueueHandle;
extern osSemaphoreId_t imuSemHandle;
extern volatile int32_t imu_debug_status;

/* High-resolution microsecond timestamp using DWT cycle counter (wrap-safe)
 * DWT->CYCCNT is 32-bit at 528MHz, wraps every ~8.13s.
 * Must be called at least once per wrap period (400Hz imuTask guarantees this).
 */
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
    const float GYR_LSB_PER_DPS = 16.4f;  // ±2000 dps range
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
      
      // Publish to ROS via imuQueueHandle (consumed by defaultTask)
      osMessageQueuePut(imuQueueHandle, &sample, 0, 0);
      
      // Pack IMU data to CAN frame (accel: milli-g, gyro: 0.1 dps)
      uint8_t can_data[8];
      int16_t ax_scaled = (int16_t)(sample.imu.ax_g * 1000.0f);
      int16_t ay_scaled = (int16_t)(sample.imu.ay_g * 1000.0f);
      int16_t az_scaled = (int16_t)(sample.imu.az_g * 1000.0f);
      int16_t gx_scaled = (int16_t)(sample.imu.gx_dps * 10.0f);
      
      memcpy(&can_data[0], &ax_scaled, sizeof(int16_t));
      memcpy(&can_data[2], &ay_scaled, sizeof(int16_t));
      memcpy(&can_data[4], &az_scaled, sizeof(int16_t));
      memcpy(&can_data[6], &gx_scaled, sizeof(int16_t));
      
      // Send to CAN bus via generic interface
      CanInterface::sendRawCANFrame(0x001, can_data, 8);
    }
  }
}
