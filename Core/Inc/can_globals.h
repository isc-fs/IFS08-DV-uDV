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

/* Queue handles — created in freertos.c, used by ISR and canTask.
 * canRxQueueHandle  : FDCAN3 (AMI + Steering bus)
 * resRxQueueHandle  : FDCAN1 (RES CANopen bus + DataLogger TX bus) */
extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t resRxQueueHandle;
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

/* --- Re-ported from v0.1 (dev's can_service.c, deleted in feat/5) --- */

/* Steering sensor (FDCAN3 ID 0x2B0).  Raw representation matches the LWS
 * sensor wire format so the CAN layer doesn't allocate floats in ISR.
 *   angle_raw  : 0.1 deg/bit, signed
 *   speed_raw  : 4 deg/s per bit, signed (decode casts the wire byte)
 *   status     : bit0=OK, bit1=CAL, bit2=TRIM
 *   last_rx_tick : osKernelGetTickCount() at the last received frame
 */
extern std::atomic<int16_t>  g_steering_angle_raw;
extern std::atomic<int8_t>   g_steering_speed_raw;
extern std::atomic<uint8_t>  g_steering_status;
extern std::atomic<uint32_t> g_steering_last_rx_tick;

/* Steering controller feedback (FDCAN3 ID 0x500, 20 Hz).
 *   angle_actual : int8 × 0.5 °/bit → actual LWS angle in degrees
 *   angle_target : int8 × 0.5 °/bit → last angle commanded by uDV
 *   angle_motor  : int8 × 0.5 °/bit → stepper position calculated by steps
 */
extern std::atomic<float>    g_steer_angle_actual;
extern std::atomic<float>    g_steer_angle_target;
extern std::atomic<float>    g_steer_angle_motor;

/* Steering controller motor status (0x500 byte 5): 0=OFF, 1=ON, -1=EMERGENCIA
 * (absolute cut, latched on the steering board, needs a physical reset).
 * g_steer_fb_last_rx_tick = osKernelGetTickCount() of the last 0x500 frame,
 * so a consumer can distinguish a live EMERGENCIA from a silent board. */
extern std::atomic<int8_t>   g_steer_motor_status;
extern std::atomic<uint32_t> g_steer_fb_last_rx_tick;

/* RES CANopen status (FDCAN1 ID 0x191 PDO).  See res_rx_dispatch in
 * can_interface.cpp for the bit layout — same as dev's res_service.
 */
extern std::atomic<bool>     g_res_estop;
extern std::atomic<uint8_t>  g_res_go_signal;
extern std::atomic<uint8_t>  g_res_radio_quality;
extern std::atomic<bool>     g_res_pre_alarm;
extern std::atomic<uint32_t> g_res_last_rx_tick;

#endif /* __cplusplus */

/* C-callable accessors — implemented in can_interface.cpp, usable from C files */
#ifdef __cplusplus
extern "C" {
#endif

float    can_c_get_steering_angle_deg(void);
int32_t  can_c_get_res_status(uint32_t now_tick, uint32_t timeout_ms);
int32_t  can_c_get_mission_index(void);
uint8_t  can_c_get_go_signal(void);
float    can_c_get_steer_angle_actual(void);
float    can_c_get_steer_angle_target(void);
float    can_c_get_steer_angle_motor(void);
uint8_t  can_c_get_assi_status_code(void);  /* AS state byte, FS-Rules T14.9 */

/* Raw steering motor-status byte: 0=OFF, 1=ON, -1=EMERGENCIA. */
int8_t   can_c_get_steer_motor_status(void);
/* Freshness-aware steering status (mirrors can_c_get_res_status):
 *   -2 = never received, -1 = timeout (silent board),
 *    0 = OFF, 1 = ON, 2 = EMERGENCIA (latched cut on the steering board). */
int32_t  can_c_get_steer_status(uint32_t now_tick, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // CAN_GLOBALS_H
