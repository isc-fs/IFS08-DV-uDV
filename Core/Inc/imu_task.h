#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS entry point for the IMU task.
 *
 * Created and scheduled by MX_FREERTOS_Init() in freertos.c; the body lives
 * here so the CubeMX-generated file stays thin. Reads the BMI088 at 400 Hz
 * (TIM2-paced), updates attitude, and pushes imu_sample_t to imuQueueHandle.
 */
void StartImuTask(void *argument);

/**
 * @brief Latest IMU init/step status (bmi088_status_t cast to int32_t).
 *
 * Written by the IMU task, read by the ROS task to publish on /imu/status.
 */
extern volatile int32_t imu_debug_status;

/**
 * @brief IMU zero-velocity (standstill) verdict for the emergency steering-centre.
 *
 * True when the ZUPT detector (zupt.h), run at the IMU sample rate, currently
 * reports the car stationary AND was updated within `window_ms` (a stale reading
 * returns false — the caller must not treat a dead IMU as "stopped"). Consumed
 * by app_task's EMERGENCY case to decide when to stop centring.
 */
bool imu_zupt_standstill(uint32_t now_ms, uint32_t window_ms);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TASK_H */
