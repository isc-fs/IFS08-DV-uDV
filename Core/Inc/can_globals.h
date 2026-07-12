#ifndef CAN_GLOBALS_H
#define CAN_GLOBALS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "bmi088.h"   /* bmi088_scaled_t (can_c_send_imu) */
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

extern std::atomic<bool> g_can_r2d;             /* ECU 0x511 DV R2D confirm */
extern std::atomic<bool> g_imu_vehicle_standstill; /* |rpm| < thresh (ECU 0x506) */
extern std::atomic<int>  g_can_mission_id;
extern std::atomic<bool> g_can_listen_go;
/* ECU 0x504 VCU_ts_active byte0: the TRACTIVE SYSTEM is live. The ECU derives
 * it from its own ok_precharge / HV-live view (AMS 0x020 ACU_ok_precharge, set
 * iff the AMS FSM is in Run|Charge), so it means "precharge complete, AIRs
 * closed" — NOT "somebody flipped the TSMS". Read it through
 * can_ts_active_fresh(), never raw: a held-stale value is exactly the failure
 * this replaced. */
extern std::atomic<bool> g_can_ts_active;       /* ECU 0x504 */
/* hardware_io_now_ms() tick of the last 0x504 (0 = never received). MUST be
 * stamped with the SAME clock can_ts_active_fresh() compares against — HAL_GetTick
 * and osKernelGetTickCount have different origins (HAL starts at HAL_Init, the
 * RTOS tick at scheduler start), so mixing them would offset the age by the
 * whole pre-scheduler init and read permanently stale. */
extern std::atomic<uint32_t> g_can_ts_active_stamp_ms;
/* 0x504 is a 100 ms cyclic, so 400 ms = 4 missed frames: jitter-safe and under
 * the FS-Rules T11.9.4 500 ms detect-and-safe cap. Same window and reasoning as
 * DV_STATUS_STALE_MS on the pipeline heartbeat. */
#define TS_ACTIVE_STALE_MS 400u

/**
 * @brief  True iff a FRESH ECU 0x504 says the tractive system is live.
 *
 * Fail-safe by construction: never received, stale, or byte0 == 0 all read
 * false. A false while AS READY/DRIVING trips Emergency (as_transition), which
 * is the intended response to "the ECU stopped telling us the TS is up".
 *
 * @param now_ms MUST come from hardware_io_now_ms() — the clock the 0x504 stamp
 *               is taken with (see g_can_ts_active_stamp_ms).
 */
bool can_ts_active_fresh(uint32_t now_ms);
/* ECU 0x505 verdict: brake pressure above the hard-braking limit
 * (config::BrakeDvHardRaw, ECU-owned). Replaces the old float32 "bar"
 * reading — the ECU sends the verdict, not the value. */
extern std::atomic<bool> g_can_brake_over_limit;
/* ECU 0x506: mechanical shaft rpm (signed; negative = reverse). */
extern std::atomic<int32_t> g_can_motor_rpm;
/* No CAN source on the current ECU contract (the old 0x506 mapping now
 * carries motor rpm); stays false until a source exists — see
 * hardware_io_read_sdc_res_open()'s pin TODO. */
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

/* Steering controller feedback (FDCAN3 ID 0x528 DV_DRIVING_DYNAMICS_1, 100 Hz;
 * this is steering's "FDCAN1" — the same physical AMI+steering bus). Byte
 * layout mirrors IFS08-DV-STEERING comunicacion_direccion.c:
 *   [0..1] reserved (0)
 *   [2] angle_actual : int8 × 0.5 °/bit → actual LWS angle in degrees
 *   [3] angle_target : int8 × 0.5 °/bit → last angle commanded by uDV
 *   [4] angle_motor  : int8 × 0.5 °/bit → stepper position calculated by steps
 *   [5] motor_state  : int8 (ESTADO_MOTOR_*)  — OFF/ON/EMERGENCIA
 *   [6..7] unused
 */
extern std::atomic<float>    g_steer_angle_actual;
extern std::atomic<float>    g_steer_angle_target;
extern std::atomic<float>    g_steer_angle_motor;
/* Steering stepper-driver state (byte 5 of 0x528). Mirrors the steering
 * firmware's enum. EMERGENCIA is a grave fault ("corte absoluto, requiere
 * reset físico") — the AS state machine treats it as an emergency trigger. */
enum SteerMotorState : int8_t {
    ESTADO_MOTOR_OFF        = 0,
    ESTADO_MOTOR_ON         = 1,
    ESTADO_MOTOR_CALIBRANDO = 2,   /* end-stop calibration in progress — NOT operative,
                                      but healthy: must NOT trip AS emergency (#113) */
    ESTADO_MOTOR_EMERGENCIA = -1,
};
extern std::atomic<int8_t>   g_steer_motor_state;

/* Steering power-on zero-cal result (byte 6 of 0x528). 1 = the steering
 * confirmed the LWS re-zero at its power-up (wheel centred → 0 deg valid);
 * 0 = failed or not yet done. Broadcast continuously at 100 Hz. */
extern std::atomic<uint8_t>  g_steer_cal_arranque_ok;

/* Steering end-stop calibration status (0x529 from the steering, FDCAN3, #113).
 * phase: 0 idle, 1-2 capture end-stops, 3 return-to-center, 4-8 sweep, 9 OK,
 * 10 FAIL. err: 0 none,1 LWS,2 timeout,3 range,4 center,5 flash,6 divergence,
 * 7 aborted,8 emergency. center/half-range/limit are int16 ×0.1 deg. */
extern std::atomic<uint8_t>  g_steer_calib_phase;
extern std::atomic<uint8_t>  g_steer_calib_error;
extern std::atomic<int16_t>  g_steer_calib_center;
extern std::atomic<int16_t>  g_steer_calib_halfrange;
extern std::atomic<int16_t>  g_steer_calib_limit;
extern std::atomic<uint32_t> g_steer_calib_last_rx_tick;

/* Calibration RELAY diag (uDV side, #113 / steering #21): prove the 0x7DF trigger
 * (FDCAN2) reaches us and that we emit 0x522 (FDCAN3) to the steering. */
extern std::atomic<uint16_t> g_calib_trigger_rx_count; /* 0x7DF frames seen on FDCAN2 */
extern std::atomic<uint16_t> g_calib_relay_count;      /* 0x522 frames we TX'd on FDCAN3 */
extern std::atomic<uint8_t>  g_calib_last_cmd;         /* last 0x522 cmd relayed (1/2)   */

/* RES CANopen status (FDCAN1 ID 0x191 PDO).  See res_rx_dispatch in
 * can_interface.cpp for the bit layout — same as dev's res_service.
 */
extern std::atomic<bool>     g_res_estop;
extern std::atomic<uint8_t>  g_res_go_signal;
extern std::atomic<uint8_t>  g_res_radio_quality;
extern std::atomic<bool>     g_res_pre_alarm;
extern std::atomic<uint32_t> g_res_last_rx_tick;
extern std::atomic<uint8_t>  g_res_raw0;        /* last 0x191 data[0] (pit-diag) */
extern std::atomic<uint16_t> g_res_rx_frame_count; /* total FDCAN1 RES-queue frames drained (pit-diag) */
extern std::atomic<uint16_t> g_nmt_sent_count;     /* NMT set-operational frames we've TX'd (pit-diag) */

/* Diagnostic (diag/emergency-reason-debug): last AS EMERGENCY cause + the
 * received /dv/status age at that edge. Written by app_task on the EMERGENCY
 * edge, read via the can_c_get_* accessors below. Observational only — no
 * effect on the state machine. */
extern std::atomic<uint8_t>  g_as_emergency_reason;    /* enum AsEmergencyReason */
extern std::atomic<uint32_t> g_as_emergency_dv_age_ms; /* now_ms - g_dv_status_stamp_ms at the trip */

#endif /* __cplusplus */

/* Which AS-transition input latched the last EMERGENCY (mirrors as_next_state's
 * predicate order — see app_task). Plain enum so both the C++ setter and the C
 * /debug formatter can name it. */
enum AsEmergencyReason {
    EMERG_NONE = 0,
    EMERG_ASMS_OFF,
    EMERG_RES_ESTOP,
    EMERG_STEER_GRAVE,
    EMERG_TS_LOSS,
    EMERG_ASB_LOW,          /* added on salvage: either ASB tank < 3 bar while armed (#184) */
    EMERG_DV_STATUS_EMERG,
    EMERG_DV_LOST_HB,
    EMERG_UNKNOWN,
};

/* C-callable accessors — implemented in can_interface.cpp, usable from C files */
#ifdef __cplusplus
extern "C" {
#endif

float    can_c_get_steering_angle_deg(void);
int32_t  can_c_get_res_status(uint32_t now_tick, uint32_t timeout_ms);
int32_t  can_c_get_mission_index(void);
int32_t  can_c_get_motor_rpm(void);
/* 50 Hz IMU broadcast to the ECU (0x512, ACU bus) — called from imu_task. */
void     can_c_send_imu(const bmi088_scaled_t *imu);
uint8_t  can_c_get_go_signal(void);
float    can_c_get_steer_angle_actual(void);
float    can_c_get_steer_angle_target(void);
float    can_c_get_steer_angle_motor(void);
int8_t   can_c_get_steer_motor_state(void); /* ESTADO_MOTOR_* (byte 5 of 0x528) */
uint8_t  can_c_get_steer_cal_arranque_ok(void); /* power-on zero-cal OK (byte 6 of 0x528) */
uint8_t  can_c_get_assi_status_code(void);  /* AS state byte, FS-Rules T14.9 */
/* diag/emergency-reason-debug — surfaced on /debug by ros_task */
uint8_t  can_c_get_as_emergency_reason(void);    /* enum AsEmergencyReason */
uint32_t can_c_get_as_emergency_dv_age_ms(void); /* /dv/status age (ms) at last emergency */
uint8_t  can_c_get_steer_calib_phase(void);      /* 0x529 phase 0..10 */
uint8_t  can_c_get_steer_calib_error(void);      /* 0x529 err   0..8  */

#ifdef __cplusplus
}
#endif

#endif // CAN_GLOBALS_H
