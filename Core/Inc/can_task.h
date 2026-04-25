#ifndef CAN_TASK_H
#define CAN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C wrapper function to start the CAN task
 * @param argument Not used (FreeRTOS parameter)
 */
void StartCanTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // CAN_TASK_H
