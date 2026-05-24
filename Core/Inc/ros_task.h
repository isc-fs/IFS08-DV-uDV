#ifndef ROS_TASK_H
#define ROS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief C wrapper function to start the ROS task
 * @param argument Not used (FreeRTOS parameter)
 */
void StartRosTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif // ROS_TASK_H
