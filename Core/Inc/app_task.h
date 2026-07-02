/**
 * @file    app_task.h
 * @brief   Application task header - FreeRTOS main coordinator
 * @details Coordinates state machine, EBS, CAN, and ROS subsystems
 */

#ifndef INC_APP_TASK_H_
#define INC_APP_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define STEER_PING_INTERVAL_MS  200
#define STEER_REPS_PER_ANGLE    5   // repetitions per angle (5 × 200 ms = 1 s each)

/**
 * @brief  Application task entry point
 * @param  argument: Not used
 * @retval None
 */
void StartAppTask(void *argument);
void move_steer_sin();

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_TASK_H_ */
