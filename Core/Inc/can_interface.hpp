#pragma once

#include "can_globals.h"
#include <cstdint>

class CanInterface {
private:
    CanInterface() = default;
    ~CanInterface() = default;

    friend void StartCanTask(void *argument);

public:
    void init();
    void sendControl(float accel, float steer);
    static void sendR2D(bool r2d);
    static void sendRawCANFrame(uint32_t can_id, const uint8_t *data, uint8_t dlc);
    
    /**
     * @brief Send ASSI status message
     * @param status Status code (0x00=OFF, 0x02=READY, 0x03=DRIVING, 0x01=EMERGENCY, 0x04=FINISHED)
     */
    static void sendAssiStatus(uint8_t status);
    
    /**
     * @brief Send ASSI emergency status message (triggered by watchdog)
     *        Immediately notifies other systems of emergency state
     */
    static void sendAssiEmergency() { sendAssiStatus(0x01); }

private:
    int m_mission_id{0};
    bool m_r2d{false};
};
