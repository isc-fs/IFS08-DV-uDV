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
#include "safety_monitor.h"   /* heartbeat / arm (extern "C" guarded) */

/**
 * @brief C wrapper function that FreeRTOS calls
 * This function bridges between C and C++
 * Implements the CAN communication task
 */
/* DataLogger TX cadence: DS 2.2 specifies a 100 ms heartbeat for 0x500/501/502. */
#define DL_TX_INTERVAL_MS       100

extern "C" void StartCanTask(void *argument)
{
    (void)argument;  // Unused parameter

    // Initialize both CAN buses we own:
    //   FDCAN3 — AMI + steering sensor + autonomy control
    //   FDCAN1 — RES CANopen + DataLogger TX
    Can::init();
    Can::initRes();

    /* The steering motor is NOT started here. app_task owns the steering
     * motor lifecycle: it fires Can::sendSteeringStart() on the READY->DRIVING
     * (GO) edge and Can::sendSteeringStop() on entering FINISHED. Starting it
     * unconditionally at boot (the old bench-sweep behaviour) would energise
     * the steering with no AS-state gating. */

    /* Both buses up: the safety monitor now watches this loop. */
    safety_arm(SAFETY_TASK_CAN);

    uint32_t last_dl_tick    = osKernelGetTickCount();

    // Task loop - runs indefinitely until the task is deleted
    while (1)
    {
            /* Liveness beat — one per service loop (~5 ms). */
            safety_heartbeat(SAFETY_TASK_CAN);

            can_msg_t rx_msg;

        // FDCAN3 (AMI + steering) — non-blocking
        if (osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, 0) == osOK)
        {
            Can::rx_dispatch(&rx_msg);
        }

        // FDCAN1 (RES CANopen) — non-blocking
        if (resRxQueueHandle != NULL &&
            osMessageQueueGet(resRxQueueHandle, &rx_msg, NULL, 0) == osOK)
        {
            Can::resRxDispatch(&rx_msg);
        }

        uint32_t now = osKernelGetTickCount();

        // DataLogger TX every 100 ms (DS 2.2 cadence).
        if ((now - last_dl_tick) >= DL_TX_INTERVAL_MS)
        {
            Can::sendDataLogger();
            last_dl_tick = now;
        }

        osDelay(5);
    }
}
