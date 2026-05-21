#pragma once

#include "can_globals.h"
#include "bmi088.h"
#include <cstdint>
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdbool.h>

namespace Can {

void init();
void sendControl(float accel, float steer);
void sendAccel(float accel);
void sendSteer(float steer);
void sendR2D(bool r2d);
void sendIMU(const bmi088_scaled_t &imu);
void sendAssiStatus(uint8_t status);

/* Called from HAL_FDCAN_RxFifo0Callback (ISR context) */
void isr_push_rx(FDCAN_HandleTypeDef *hfdcan);

/* Dispatch a received message by CAN ID — called from canTask */
void rx_dispatch(const can_msg_t *msg);

} // namespace Can

extern "C" {
    void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan);
    void can_interface_send_assi_emergency_from_isr(void);
}