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

private:
    int m_mission_id{0};
    bool m_r2d{false};
};
