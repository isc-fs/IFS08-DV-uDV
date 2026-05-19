#ifndef CAN_GLOBALS_H
#define CAN_GLOBALS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os2.h"
#include <stdbool.h>

/* CAN message struct for queue transport (16 bytes, ISR-safe) */
typedef struct {
	uint32_t id;
	uint8_t  data[8];
	uint8_t  dlc;
	uint8_t  _pad[3];
} can_msg_t;

/* Queue handle — created in freertos.c, used by ISR and canTask */
extern osMessageQueueId_t canRxQueueHandle;

/* For C++ builds expose std::atomic variables; for C builds provide
 * volatile fallbacks. Do NOT wrap the C++ declarations in extern "C"
 * so that linkage matches the definitions in the C++ source file. */
#ifdef __cplusplus
#include <atomic>

extern std::atomic<bool> g_can_r2d;
extern std::atomic<bool> g_can_vehicle_standstill;
extern std::atomic<int>  g_can_mission_id;
extern std::atomic<bool> g_can_listen_go;
extern std::atomic<bool> g_can_ts_active;
extern std::atomic<float> g_can_brake_pressure;
extern std::atomic<bool> g_reset_cmd;

/* CAN command queue is a C type and can be declared here for both C/C++ */
extern QueueHandle_t g_can_cmd_queue;

#else /* C compilation */

extern volatile bool g_can_r2d;
extern volatile bool g_can_vehicle_standstill;
extern volatile int  g_can_mission_id;
extern volatile bool g_can_listen_go;
extern volatile bool g_can_ts_active;
extern volatile float g_can_brake_pressure;
extern volatile bool g_reset_cmd;

extern QueueHandle_t g_can_cmd_queue;

#endif /* __cplusplus */

#endif // CAN_GLOBALS_H
