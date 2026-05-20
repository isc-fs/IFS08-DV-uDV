/**
 * @file    ami_task.h
 * @brief   AMI task header - LED mission indicator task
 */

#ifndef INC_AMI_TASK_H_
#define INC_AMI_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  AMI task entry point
 * @param  argument: Not used
 * @retval None
 */
void StartAmiTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_AMI_TASK_H_ */
