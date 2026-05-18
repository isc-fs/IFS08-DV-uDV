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
    CAN_CMD_SEND_ASSI_STATUS,
} CanCmd;

typedef struct {
    CanCmd cmd;
    float accel;
    float steer;
    uint8_t status;  // For ASSI status messages
} CanCommandMessage;

// Helper function to send commands to the CAN queue
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

/**
 * @brief Send ASSI (Autonomous System Status Indicator) status via CAN
 * @param status Status code (Check Statemanager)
 */
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

#endif // CAN_TASK_COMMANDS_H
