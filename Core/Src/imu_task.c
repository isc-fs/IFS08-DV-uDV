/**
 ******************************************************************************
 * @file    imu_task.c
 * @brief   IMU FreeRTOS task: BMI088 read -> attitude -> imuQueueHandle.
 *
 * Body extracted verbatim from freertos.c (StartImuTask) so the CubeMX-managed
 * file stays thin. The task is still created and scheduled by MX_FREERTOS_Init().
 ******************************************************************************
 */
#include "imu_task.h"

#include "cmsis_os.h"
#include "main.h"
#include "imu_service.h"
#include "bmi088.h"
#include "attitude.h"
#include "tim.h"               /* htim2 */
#include "safety_monitor.h"    /* safety_arm / safety_heartbeat / SAFETY_TASK_IMU */
#include "dwt_time.h"          /* dwt_micros */
#include "can_globals.h"       /* can_c_send_imu (50 Hz ECU broadcast) */
#include "zupt.h"              /* zero-velocity (standstill) detector */

#include <math.h>             /* sqrtf */
#include <stdbool.h>

/* RTOS objects created in freertos.c (MX_FREERTOS_Init). */
extern osMessageQueueId_t imuQueueHandle;
extern osSemaphoreId_t    imuSemHandle;

/* Shared debug status — read by the ROS task and published on /imu/status. */
volatile int32_t imu_debug_status = -99;

/* IMU zero-velocity (standstill) detector state + published verdict. The
 * detector (zupt.h) runs every 400 Hz sample so it sees the vibration; the
 * debounced verdict + its tick are read by app_task via imu_zupt_standstill()
 * to decide when to stop centring the steering in an emergency. */
static zupt_t            s_zupt;
static volatile bool     s_imu_standstill      = false;
static volatile uint32_t s_imu_standstill_tick = 0u;

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

  zupt_reset(&s_zupt);

  // Start TIM2 interrupt for deterministic 400Hz sampling
  HAL_TIM_Base_Start_IT(&htim2);

  /* Calibration done and 400 Hz sampling armed: from here the safety
   * monitor watches this loop. A stall (e.g. TIM2 stops, I2C wedges)
   * now trips the watchdog emergency. */
  safety_arm(SAFETY_TASK_IMU);

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

      /* Zero-velocity detection (standstill) for the emergency steering-centre.
       * Feed |accel| (g, incl. gravity ≈1 at rest) and |gyro| (dps, with the
       * calibrated bias removed) — the two quietness signals ZUPT thresholds. */
      {
        const float gx = sample.imu.gx_dps - imu_svc.att.gx_off_dps;
        const float gy = sample.imu.gy_dps - imu_svc.att.gy_off_dps;
        const float gz = sample.imu.gz_dps - imu_svc.att.gz_off_dps;
        const float gyro_mag  = sqrtf(gx * gx + gy * gy + gz * gz);
        const float accel_mag = sqrtf(sample.imu.ax_g * sample.imu.ax_g +
                                      sample.imu.ay_g * sample.imu.ay_g +
                                      sample.imu.az_g * sample.imu.az_g);
        s_imu_standstill      = zupt_update(&s_zupt, sample.t_ms, accel_mag, gyro_mag);
        s_imu_standstill_tick = sample.t_ms;
      }

      /* IMU broadcast to the ECU (CAN 0x512 on the ACU bus), downsampled
       * 400 -> 50 Hz (every 8th sample). Lives here — not in ros_task —
       * so the ECU keeps its IMU feed with no DVPC/agent connected. */
      static uint8_t imu_can_div = 0;
      if (++imu_can_div >= 8U)
      {
        imu_can_div = 0;
        can_c_send_imu(&sample.imu);
      }
    }

    /* Liveness beat: one per 400 Hz wake, whether or not the sample
     * read succeeded — proves the loop is still being serviced. */
    safety_heartbeat(SAFETY_TASK_IMU);
  }
}

bool imu_zupt_standstill(uint32_t now_ms, uint32_t window_ms)
{
  const uint32_t last = s_imu_standstill_tick;
  if (last == 0u) return false;                              /* no sample yet */
  if ((uint32_t)(now_ms - last) > window_ms) return false;   /* stale -> don't trust */
  return s_imu_standstill;
}
