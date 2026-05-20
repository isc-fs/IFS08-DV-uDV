#pragma once

#include "can_globals.h"
#include "bmi088.h"
#include <cstdint>
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdbool.h>

class CanInterface {
public:
    static CanInterface& getInstance() {
        static CanInterface instance;
        return instance;
    }

    void init();
    static void sendControl(float accel, float steer);
    static void sendAccel(float accel);
    static void sendSteer(float steer);
    static void sendR2D(bool r2d);
    static void sendIMU(const bmi088_scaled_t &imu);
    static void sendAssiStatus(uint8_t status);

    /* Called from HAL_FDCAN_RxFifo0Callback (ISR context) */
    void isr_push_rx(FDCAN_HandleTypeDef *hfdcan);

    /* Dispatch a received message by CAN ID — called from canTask */
    void rx_dispatch(const can_msg_t *msg);

private:
    CanInterface() = default;
    ~CanInterface() = default;
    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;
};

extern "C" {
    void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan);
    void can_interface_send_assi_emergency_from_isr(void);
}
