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

typedef enum {
	CAN_CMD_NONE = 0,
	CAN_CMD_SEND_CONTROL,
	CAN_CMD_SEND_ASSI_STATUS,
} CanCmd;

typedef struct {
	CanCmd cmd;
	float accel;
	float steer;
	uint8_t status;  // For ASSI status messages
} CanCommandMessage;

static inline void send_can_control(float accel, float steer)
{
	if (g_can_cmd_queue == NULL) return;

	CanCommandMessage msg;
	msg.cmd = CAN_CMD_SEND_CONTROL;
	msg.accel = accel;
	msg.steer = steer;
	msg.status = 0;

	xQueueSend(g_can_cmd_queue, &msg, 0);
}

static inline void send_can_assi_status(uint8_t status)
{
	if (g_can_cmd_queue == NULL) return;

	CanCommandMessage msg;
	msg.cmd = CAN_CMD_SEND_ASSI_STATUS;
	msg.accel = 0.0f;
	msg.steer = 0.0f;
	msg.status = status;

	xQueueSend(g_can_cmd_queue, &msg, 0);
}

#ifdef __cplusplus
}
#endif

#endif // CAN_TASK_H
