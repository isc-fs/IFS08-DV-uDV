#pragma once

#include "can_globals.h"
#include <cstdint>
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdbool.h>

/* C-compatible CAN message and queue handle */
#include "can_globals.h"

class CanInterface {
public:
    // Make CanInterface a singleton
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

    /* CAN-backed shared state setters/getters used by hardware_io and state code */
    void set_ts_active(bool active);
    bool get_ts_active(void);
    void set_brake_pressure(float pressure);
    float get_brake_pressure(void);

private:
    CanInterface() = default; // Private constructor
    ~CanInterface() = default;
    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;

    int m_mission_id{0};
    bool m_r2d{false};
    bool m_ts_active{false};
    float m_brake_pressure{0.0f};
};

// C-compatible wrapper for ISR
extern "C" void can_interface_rx_isr_callback(FDCAN_HandleTypeDef *hfdcan);

