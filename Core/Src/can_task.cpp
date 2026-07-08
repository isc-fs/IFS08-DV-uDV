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
#include "pit_diag.h"
#include "safety_monitor.h"   /* heartbeat / arm (extern "C" guarded) */

/**
 * @brief C wrapper function that FreeRTOS calls
 * This function bridges between C and C++
 * Implements the CAN communication task
 */
/* DataLogger TX cadence: DS 2.2 specifies a 100 ms heartbeat for 0x500/501/502. */
#define DL_TX_INTERVAL_MS       100

/* RES bring-up re-arm. The RES is a CANopen node that only streams its 0x191
 * PDO (carrying the GO / e-stop bits) once commanded OPERATIONAL via NMT
 * (spec DS 3 "RES Technical Information": boot-up 0x711 -> master NMT
 * 0x000 [0x01, node] -> 0x191 every 30 ms). The boot-up handshake in
 * resRxDispatch only fires if we are listening when the RES boots; if the uDV
 * is reset/powered AFTER the RES, that one-shot boot-up frame is missed and the
 * RES sits pre-operational and SILENT (no PDO, so GO can never arrive — and
 * with BENCH_STUB_RES the car still reaches READY, hiding it). So re-send NMT
 * set-operational periodically until 0x191 is actually arriving. NMT to an
 * already-operational node is a no-op, and this self-quiesces once the PDO
 * stream is healthy (and re-kicks if the RES ever drops back to pre-op). */
#define RES_NMT_REARM_INTERVAL_MS  500   /* how often to (re)issue NMT while silent */
#define RES_PDO_STALE_MS           200   /* PDO gap that means "not streaming" (30 ms nominal) */

extern "C" void StartCanTask(void *argument)
{
    (void)argument;  // Unused parameter

    // Initialize the three CAN buses we own:
    //   FDCAN3 — AMI + steering (local DV peripherals)
    //   FDCAN1 — RES CANopen + DataLogger TX
    //   FDCAN2 — ACU bus (ECU/VCU + AMS): the ECU<->uDV contract
    Can::init();
    Can::initRes();
    Can::initEcu();

    /* The steering motor is NOT started here. app_task owns the steering
     * motor lifecycle: it fires Can::sendSteeringStart() on the READY->DRIVING
     * (GO) edge and Can::sendSteeringStop() on entering FINISHED. Starting it
     * unconditionally at boot (the old bench-sweep behaviour) would energise
     * the steering with no AS-state gating. */

    /* Both buses up: the safety monitor now watches this loop. */
    safety_arm(SAFETY_TASK_CAN);

    uint32_t last_dl_tick    = osKernelGetTickCount();
    uint32_t last_nmt_tick   = osKernelGetTickCount();

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

        /* RES bring-up: keep the RES commanded OPERATIONAL until its 0x191 PDO
         * is actually streaming, so GO can be received regardless of uDV/RES
         * power-up order (see the RES_NMT_* note above). Self-quiesces once the
         * PDO stream is fresh. */
        if ((now - last_nmt_tick) >= RES_NMT_REARM_INTERVAL_MS)
        {
            uint32_t last_pdo = g_res_last_rx_tick.load();
            if (last_pdo == 0U || (now - last_pdo) > RES_PDO_STALE_MS)
            {
                Can::sendNmtSetOperational();
            }
            last_nmt_tick = now;
        }

        /* Pit-diag: CAN-only observability stream on FDCAN2 (self-paced,
         * gated by the 0x7DE arm frame — no-op until armed). */
        pit_diag_service(now);

        // DataLogger TX every 100 ms (DS 2.2 cadence).
        if ((now - last_dl_tick) >= DL_TX_INTERVAL_MS)
        {
            Can::sendDataLogger();
            last_dl_tick = now;
        }

        osDelay(5);
    }
}
