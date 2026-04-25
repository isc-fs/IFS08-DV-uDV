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

/**
 * @brief  Application task entry point
 * @param  argument: Not used
 * @retval None
 */
void StartAppTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_APP_TASK_H_ */
