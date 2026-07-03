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

#ifdef __cplusplus
extern "C" {
#endif

/* Set/get current AS mode */
void assi_set_mode(assi_mode_t mode);
assi_mode_t assi_get_mode(void);

/* Task entry */
void StartAssiTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ASSI_TASK_H */
