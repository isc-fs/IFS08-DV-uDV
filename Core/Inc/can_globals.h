#ifndef CAN_GLOBALS_H
#define CAN_GLOBALS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os2.h"
#include <stdbool.h>

/* CAN message struct for queue transport (16 bytes, ISR-safe)
 * Fields:
 *  - id: 32-bit CAN identifier
 *  - data: 8 bytes of payload
 *  - dlc: data length code
 *  - bus: source FDCAN bus (1..3) — added for multi-bus support
 *  - _pad: padding to keep struct size 16 bytes
 */
typedef struct {
	uint32_t id;
	uint8_t  data[8];
	uint8_t  dlc;
	uint8_t  bus;     /* 1 = FDCAN1, 2 = FDCAN2, 3 = FDCAN3, 0 = unknown */
	uint8_t  _pad[2];
} can_msg_t;

/* Queue handle — created in freertos.c, used by ISR and canTask */
extern osMessageQueueId_t canRxQueueHandle;
/* CAN command queue removed — callers should call `CanInterface` directly. */

/* For C++ builds expose std::atomic variables; for C builds provide
 * volatile fallbacks. Do NOT wrap the C++ declarations in extern "C"
 * so that linkage matches the definitions in the C++ source file. */
#ifdef __cplusplus
#include <atomic>

extern std::atomic<bool> g_can_r2d;
extern std::atomic<bool> g_imu_vehicle_standstill;
extern std::atomic<int>  g_can_mission_id;
extern std::atomic<bool> g_can_listen_go;
extern std::atomic<bool> g_can_ts_active;
extern std::atomic<float> g_can_brake_pressure;
extern std::atomic<bool> g_can_sdc_res_open;
extern std::atomic<bool> g_reset_cmd;

#endif /* __cplusplus */

#endif // CAN_GLOBALS_H
