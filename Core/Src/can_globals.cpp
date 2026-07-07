#include "can_globals.h"

// Definitions of CAN atomic signals
std::atomic<bool> g_can_r2d{false};
std::atomic<bool> g_imu_vehicle_standstill{true};
std::atomic<int>  g_can_mission_id{-1};  /* -1 = none received; 0 is a valid 0-based mission */
std::atomic<bool> g_can_listen_go{false};
std::atomic<bool> g_can_ts_active{false};
std::atomic<bool> g_can_brake_over_limit{false};
std::atomic<int32_t> g_can_motor_rpm{0};
std::atomic<bool> g_can_sdc_res_open{false};
std::atomic<bool> g_reset_cmd{false};

// Steering controller feedback (FDCAN3 0x500)
std::atomic<float> g_steer_angle_actual{0.0f};
std::atomic<float> g_steer_angle_target{0.0f};
std::atomic<float> g_steer_angle_motor{0.0f};
// Default OFF (not EMERGENCIA) so an absent 0x500 never fabricates an emergency.
std::atomic<int8_t> g_steer_motor_state{ESTADO_MOTOR_OFF};

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
std::atomic<uint8_t>  g_res_raw0{0};        /* last 0x191 data[0] (pit-diag) */


