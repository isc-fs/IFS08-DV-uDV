#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>

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

#ifdef __cplusplus
}
#endif

#endif /* IMU_TASK_H */
