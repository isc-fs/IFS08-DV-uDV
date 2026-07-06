#pragma once

#include "can_globals.h"
#include "bmi088.h"
#include <cstdint>
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdbool.h>

namespace Can {

/* FDCAN3 bring-up (AMI + steering sensor + autonomy control bus) */
void init();
/* FDCAN1 bring-up (RES CANopen + DataLogger TX bus) */
void initRes();

void sendControl(float accel, float steer);
void sendSteer(float steer);
void sendIMU(const bmi088_scaled_t &imu);

void initEcu();
void sendAccel(float accel);
void sendR2dRequest(uint8_t request);

/* RES CANopen — send NMT "set operational" to RES node 0x11 on FDCAN1.
 * Sent automatically on receipt of the RES boot-up frame (0x711). */
void sendNmtSetOperational();

/* Steering controller TX (FDCAN3)
 *  sendSteeringMotor: 0x010 — 1=start, 0=stop, 1-byte payload
 *  sendSteeringAngle: 0x020 — int32 LE in 0.01-deg units, 4-byte payload
 */
void sendSteeringAngle(float angle_deg);
void sendSteeringStart();
void sendSteeringStop();

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
    //void can_interface_send_assi_emergency_from_isr(void);
}