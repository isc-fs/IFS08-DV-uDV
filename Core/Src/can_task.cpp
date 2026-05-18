/**
 * @file can_task.cpp
 * @brief CAN task implementation for FreeRTOS
 * @note Handles CAN communication and command processing
 */

#include <cstdint>
#include <cstring>

// FreeRTOS includes
extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "cmsis_os.h"
    #include "main.h"
    #include "can_service.h"
}

// CAN-related includes
#include "can_task.h"
#include "can_task_commands.h"
#include "can_interface.hpp"
#include "can_globals.h"

/**
 * @brief C wrapper function that FreeRTOS calls
 * This function bridges between C and C++
 * Implements the CAN communication task
 */
extern "C" void StartCanTask(void *argument)
{
    (void)argument;  // Unused parameter

    // Create and initialize the CAN interface
    static CanInterface can;
    can.init();

    // Create the queue for AppTask to send commands
    g_can_cmd_queue = xQueueCreate(8, sizeof(CanCommandMessage));

    if (g_can_cmd_queue == NULL)
    {
        // Queue creation failed - enter error loop
        while (1)
        {
            osDelay(1000);
        }
    }

    CanCommandMessage msg;

    // Task loop - runs indefinitely until the task is deleted
    while (1)
    {
        // Process commands from AppTask (short timeout for responsiveness)
        if (xQueueReceive(g_can_cmd_queue, &msg, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (msg.cmd)
            {
                case CAN_CMD_SEND_CONTROL:
                    can.sendControl(msg.accel, msg.steer);
                    break;

                case CAN_CMD_SEND_ASSI_STATUS:
                    CanInterface::sendAssiStatus(msg.status);
                    break;

                default:
                    break;
            }
        }

        // Process incoming CAN messages from ISR queue
        can_msg_t rx_msg;
        if (osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, 0) == osOK)
        {
            can_rx_dispatch(&rx_msg);
        }

        osDelay(5);
    }
}
