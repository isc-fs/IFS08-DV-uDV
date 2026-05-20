/**
 * @file    imu_task.h
 * @brief   IMU task header - FreeRTOS thread for BMI088 sensor acquisition
 * @details Reads BMI088 IMU at 400Hz via TIM2 interrupt, packs samples to
 *          both ROS queue and CAN frames. Includes gyro bias calibration.
 */

#ifndef INC_IMU_TASK_H_
#define INC_IMU_TASK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  IMU task entry point
 * @param  argument: Not used
 * @retval None
 */
void StartImuTask(void *argument);

/* Shared IMU debug status published over ROS diagnostics. */
extern volatile int32_t imu_debug_status;

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_TASK_H_ */
