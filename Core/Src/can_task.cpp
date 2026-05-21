/**
 * @file can_task.cpp
 * @brief CAN task implementation for FreeRTOS
 * @note Handles CAN communication and command processing
 */

#include <cstdint>
#include <cstring>

// FreeRTOS includes (C linkage)
extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "cmsis_os.h"
    #include "main.h"
}

#include "can_globals.h"

#include "can_interface.hpp"
#include "can_task.h"

/**
 * @brief C wrapper function that FreeRTOS calls
 * This function bridges between C and C++
 * Implements the CAN communication task
 */
extern "C" void StartCanTask(void *argument)
{
    (void)argument;  // Unused parameter

    // Initialize CAN interface namespace
    Can::init();

    // Task loop - runs indefinitely until the task is deleted
    while (1)
    {
        // Process incoming CAN messages from ISR queue and dispatch
        can_msg_t rx_msg;
        if (osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, 0) == osOK)
        {
            Can::rx_dispatch(&rx_msg);
        }

        osDelay(5);
    }
}
