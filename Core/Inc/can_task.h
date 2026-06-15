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

/* CAN command types and helpers (moved from can_task_commands.h)
 * These are small C-compatible helpers used by application code to send
 * commands to the CAN task queue. Kept as inline helpers to avoid linking
 * issues from multiple translation units.
 */
#include "can_globals.h"
#include <stdint.h>
/* Legacy queue-based helpers removed — callers should call `CanInterface`
 * directly in C++ or implement appropriate C wrappers.
 */

#ifdef __cplusplus
}
#endif

#endif // CAN_TASK_H
