#ifndef CAN_GLOBALS_H
#define CAN_GLOBALS_H

#include <atomic>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Atomics for simple CAN-derived signals (read-only from other tasks)
extern std::atomic<bool> g_can_r2d;
extern std::atomic<bool> g_can_vehicle_standstill;
extern std::atomic<int>  g_can_mission_id;
extern std::atomic<bool> g_can_listen_go;
extern std::atomic<bool> g_can_ts_active;
extern std::atomic<float> g_can_brake_pressure;
extern std::atomic<bool> g_reset_cmd;

// CAN command queue
extern QueueHandle_t g_can_cmd_queue;

#ifdef __cplusplus
}
#endif

#endif // CAN_GLOBALS_H
