#include "can_globals.h"

// Definitions of CAN atomic signals
std::atomic<bool> g_can_r2d{false};
std::atomic<bool> g_can_vehicle_standstill{true};
std::atomic<int>  g_can_mission_id{0};
std::atomic<bool> g_can_listen_go{false};
std::atomic<bool> g_can_ts_active{false};
std::atomic<float> g_can_brake_pressure{0.0f};
std::atomic<bool> g_reset_cmd{false};

// CAN command queue
QueueHandle_t g_can_cmd_queue = nullptr;

extern "C" void can_set_ts_active(bool active)
{
	g_can_ts_active.store(active);
}

extern "C" bool can_get_ts_active(void)
{
	return g_can_ts_active.load();
}

extern "C" void can_set_brake_pressure(float pressure)
{
	g_can_brake_pressure.store(pressure);
}

extern "C" float can_get_brake_pressure(void)
{
	return g_can_brake_pressure.load();
}
