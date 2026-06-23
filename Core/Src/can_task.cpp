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
/* DataLogger TX cadence: DS 2.2 specifies a 100 ms heartbeat for 0x500/501/502. */
#define DL_TX_INTERVAL_MS       100
#define STEER_PING_INTERVAL_MS  200
#define STEER_REPS_PER_ANGLE    5   // repetitions per angle (5 × 200 ms = 1 s each)

static const float STEER_SEQ[] = { 0, 60, 120, 180, 120, 180, 60, 0 };
static const uint8_t STEER_SEQ_LEN = sizeof(STEER_SEQ) / sizeof(STEER_SEQ[0]);

extern "C" void StartCanTask(void *argument)
{
    (void)argument;  // Unused parameter

    // Initialize both CAN buses we own:
    //   FDCAN3 — AMI + steering sensor + autonomy control
    //   FDCAN1 — RES CANopen + DataLogger TX
    Can::init();
    Can::initRes();

    Can::sendSteeringMotor(1);  // Motor start — sent once before angle loop

    uint32_t last_dl_tick    = osKernelGetTickCount();
    uint32_t last_ping_tick  = osKernelGetTickCount();
    uint8_t  seq_idx  = 0;
    uint8_t  seq_reps = 0;

    // Task loop - runs indefinitely until the task is deleted
    while (1)
    {
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

        // FDCAN3 steering angle sequence at 5 Hz, STEER_REPS_PER_ANGLE per step.
        if ((now - last_ping_tick) >= STEER_PING_INTERVAL_MS)
        {
            Can::sendSteeringAngle(STEER_SEQ[seq_idx]);
            if (++seq_reps >= STEER_REPS_PER_ANGLE) {
                seq_reps = 0;
                if (++seq_idx >= STEER_SEQ_LEN) seq_idx = 0;
            }
            last_ping_tick = now;
        }

        osDelay(5);
    }
}
