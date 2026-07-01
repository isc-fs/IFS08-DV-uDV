/*
 * assi_task.h
 * ASSI (status lights) FreeRTOS task API
 */
#ifndef ASSI_TASK_H
#define ASSI_TASK_H

#include <stdint.h>

/* AS modes */
typedef enum {
    AS_MODE_OFF = 0,
    AS_MODE_READY,
    AS_MODE_DRIVING,
    AS_MODE_EMERGENCY,
    AS_MODE_FINISHED,
} assi_mode_t;

/* Set/get current AS mode */
void assi_set_mode(assi_mode_t mode);
assi_mode_t assi_get_mode(void);

/* Task entry (created from MX_FREERTOS_Init) */
void StartAssiTask(void *argument);

#endif /* ASSI_TASK_H */
