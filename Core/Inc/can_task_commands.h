#ifndef CAN_TASK_COMMANDS_H
#define CAN_TASK_COMMANDS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "can_globals.h"
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// CAN command types
typedef enum {
    CAN_CMD_NONE = 0,
    CAN_CMD_SEND_CONTROL,
} CanCmd;

typedef struct {
    CanCmd cmd;
    float accel;
    float steer;
} CanCommandMessage;

// Helper function to send commands to the CAN queue
static inline void send_can_control(float accel, float steer)
{
    if (g_can_cmd_queue == NULL) return;

    CanCommandMessage msg;
    msg.cmd = CAN_CMD_SEND_CONTROL;
    msg.accel = accel;
    msg.steer = steer;

    xQueueSend(g_can_cmd_queue, &msg, 0);
}

#ifdef __cplusplus
}
#endif

#endif // CAN_TASK_COMMANDS_H
