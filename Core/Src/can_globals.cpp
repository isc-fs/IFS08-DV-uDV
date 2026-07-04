#include "can_globals.h"

// Definitions of CAN atomic signals
std::atomic<bool> g_can_r2d{false};
std::atomic<bool> g_imu_vehicle_standstill{true};
std::atomic<int>  g_can_mission_id{-1};  /* -1 = none received; 0 is a valid 0-based mission */
std::atomic<bool> g_can_listen_go{false};
std::atomic<bool> g_can_ts_active{false};
std::atomic<float> g_can_brake_pressure{0.0f};
std::atomic<bool> g_can_sdc_res_open{false};
std::atomic<bool> g_reset_cmd{false};

// Steering controller feedback (FDCAN3 0x500)
std::atomic<float> g_steer_angle_actual{0.0f};
std::atomic<float> g_steer_angle_target{0.0f};
std::atomic<float> g_steer_angle_motor{0.0f};

// Steering controller motor status (0x500 byte 5): 0=OFF, 1=ON, -1=EMERGENCIA.
// Init to OFF; g_steer_fb_last_rx_tick lets consumers detect a silent board.
std::atomic<int8_t>   g_steer_motor_status{0};
std::atomic<uint32_t> g_steer_fb_last_rx_tick{0};

// Steering sensor state (re-ported from v0.1)
std::atomic<int16_t>  g_steering_angle_raw{0};
std::atomic<int8_t>   g_steering_speed_raw{0};
std::atomic<uint8_t>  g_steering_status{0};
std::atomic<uint32_t> g_steering_last_rx_tick{0};

// RES CANopen state (re-ported from v0.1)
std::atomic<bool>     g_res_estop{false};
std::atomic<uint8_t>  g_res_go_signal{0};
std::atomic<uint8_t>  g_res_radio_quality{0};
std::atomic<bool>     g_res_pre_alarm{false};
std::atomic<uint32_t> g_res_last_rx_tick{0};


