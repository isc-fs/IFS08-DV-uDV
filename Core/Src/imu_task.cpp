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
#include "can_globals.h"

/* Defines */
#define G_TO_MS2   9.80665f
#define DPS_TO_RAD (float)(M_PI / 180.0)

/* External declarations (defined in freertos.c) */
extern osMessageQueueId_t imuQueueHandle;
extern osSemaphoreId_t imuSemHandle;

/* Shared IMU debug status visible to ROS diagnostics. */
volatile int32_t imu_debug_status = -99;

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

      // TODO: set vehicle standstill atomic
      // Determine vehicle standstill from IMU: low angular rates and
      // acceleration magnitude near 1g (gravity) within small tolerance
      float ax = sample.imu.ax_g;
      float ay = sample.imu.ay_g;
      float az = sample.imu.az_g;
      float acc_g = sqrtf(ax*ax + ay*ay + az*az);
      float gx = sample.imu.gx_dps;
      float gy = sample.imu.gy_dps;
      float gz = sample.imu.gz_dps;
      float gyro_max = fmaxf(fmaxf(fabsf(gx), fabsf(gy)), fabsf(gz));

      const float ACC_TOL_G = 0.05f;   // 0.05 g tolerance
      const float GYRO_TOL_DPS = 2.0f; // 2 dps tolerance

      bool imu_standstill = (fabsf(acc_g - 1.0f) <= ACC_TOL_G) && (gyro_max <= GYRO_TOL_DPS);
      g_imu_vehicle_standstill.store(imu_standstill);
      // Publish to ROS via imuQueueHandle (consumed by defaultTask)
      osMessageQueuePut(imuQueueHandle, &sample, 0, 0);
      
      // Send IMU data to CAN via the dedicated interface
      CanInterface::sendIMU(sample.imu);
    }
  }
}
