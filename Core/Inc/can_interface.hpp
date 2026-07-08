#pragma once

#include "can_globals.h"
#include "bmi088.h"   /* bmi088_scaled_t (sendIMU) */
#include <cstdint>
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdbool.h>

namespace Can {

/* FDCAN3 bring-up (AMI + steering bus, local DV peripherals) */
void init();
/* FDCAN1 bring-up (RES CANopen + DataLogger TX bus) */
void initRes();
/* FDCAN2 bring-up (ACU bus: ECU/VCU + AMS — the ECU<->uDV contract) */
void initEcu();

/* 0x507 torque command to the ECU: input is the pipeline's normalised
 * throttle [-1..1], sent as int32 LE integer percent 0..100 (negative
 * clamps to 0). Expected cyclic at 20 ms; a stale stream reads as 0
 * torque on the ECU (never APPS). */
void sendAccel(float accel);
/* 0x510 DV ready-to-drive request to the ECU (byte0 != 0). The ECU
 * honours it only while its brake sensor confirms hard braking and
 * confirms on 0x511 (rx -> g_can_r2d). */
void sendR2dRequest(uint8_t request);
/* 0x512 IMU broadcast to the ECU (ACU bus): ax/ay/az int16 LE milli-g +
 * gx int16 LE 0.1 dps. Called at 50 Hz from imu_task via can_c_send_imu. */
void sendIMU(const bmi088_scaled_t &imu);
void sendAssiStatus(uint8_t status);

/* RES CANopen — send NMT "set operational" to RES node 0x11 on FDCAN1.
 * Sent automatically on receipt of the RES boot-up frame (0x711). */
void sendNmtSetOperational();

/* Steering controller TX (FDCAN3)
 *  sendSteeringStart/Stop: 0x010 motor control — byte[0] motor_start,
 *                          1=start / 0=stop, 4-byte payload. The steering
 *                          controller runs the motor only when byte[0] ==
 *                          MOTOR_CMD_ON (1); any other value is a clean stop
 *                          (see IFS08-DV-STEERING comunicacion_direccion.c).
 *  sendSteeringAngle:      0x020 — int32 LE in 0.01-deg units, 4-byte payload
 */
void sendSteeringStart();
void sendSteeringStop();
void sendSteeringAngle(float angle_deg);
void sendSteeringCalib(uint8_t cmd);   /* #113: 0x30 end-stop calib — 1=start, 2=abort */

/* DataLogger TX (FDCAN1) per FS-DV spec DS 2.2 — emits 0x500/0x501/0x502.
 * Intended to be called periodically (~10 Hz) from can_task. */
void sendDataLogger();

/* Called from HAL_FDCAN_RxFifo0Callback (ISR context) */
void isr_push_rx(FDCAN_HandleTypeDef *hfdcan);

/* Dispatch a received message by CAN ID — called from canTask.
 * rx_dispatch handles FDCAN3 traffic; resRxDispatch handles FDCAN1 (RES). */
void rx_dispatch(const can_msg_t *msg);
void resRxDispatch(const can_msg_t *msg);

} // namespace Can

extern "C" {
    void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan);
    void can_interface_send_assi_emergency_from_isr(void);
}