#include "ros_globals.h"

// Definitions of atomic feedback from PC, owned by RosTask
std::atomic<float> g_accel_cmd{0.0f};
std::atomic<float> g_steer_cmd{0.0f};
std::atomic<bool>  g_finished_cmd{false};
std::atomic<bool>  g_emergency_cmd{false};
std::atomic<bool>  g_mission_going_cmd{false};
std::atomic<bool>  g_set_mission_in_progress{false};
std::atomic<bool>  g_set_mission_ready{false};

// State telemetry snapshot, owned by AppTask and published by RosTask
std::atomic<uint8_t> g_telemetry_as_state{0};
std::atomic<uint8_t> g_telemetry_ebs_init_state{0};
std::atomic<bool>    g_telemetry_asms_on{false};
std::atomic<bool>    g_telemetry_ts_active{false};
std::atomic<bool>    g_telemetry_sdc_res_open{false};
std::atomic<bool>    g_telemetry_brakes_engaged{false};
std::atomic<bool>    g_telemetry_r2d{false};
std::atomic<bool>    g_telemetry_vehicle_standstill{true};
std::atomic<bool>    g_telemetry_mission_selected{false};
std::atomic<bool>    g_telemetry_mission_finished{false};
std::atomic<bool>    g_telemetry_abs_checks_ok{false};
std::atomic<bool>    g_telemetry_ebs_activated{false};

// IMU drop counter — incremented by imu_task whenever osMessageQueuePut
// to imuQueueHandle returns non-OK (queue full), read by ros_interface
// during the ~10 s periodic re-sync.
std::atomic<uint32_t> g_imu_drop_count{0};

// ROS command queue
QueueHandle_t g_ros_cmd_queue = nullptr;
