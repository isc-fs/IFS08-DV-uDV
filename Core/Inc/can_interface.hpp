#pragma once

#include "can_globals.h"
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
    void sendControl(float accel, float steer);
    void sendR2D(bool r2d);
    static void sendRawCANFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);

    /**
     * @brief Send ASSI status message
     * @param status Status code (0x00=OFF, 0x02=READY, 0x03=DRIVING, 0x01=EMERGENCY, 0x04=FINISHED)
     */
    void sendAssiStatus(uint8_t status);

    /**
     * @brief Send ASSI emergency status message (triggered by watchdog)
     *        Immediately notifies other systems of emergency state
     */
    void sendAssiEmergency() { sendAssiStatus(0x01); }

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


