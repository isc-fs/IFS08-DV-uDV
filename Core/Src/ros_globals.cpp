#include "ros_globals.h"

// Definitions of atomic feedback from PC, owned by RosTask
std::atomic<float> g_accel_cmd{0.0f};
std::atomic<float> g_steer_cmd{0.0f};
std::atomic<bool>  g_finished_cmd{false};
std::atomic<bool>  g_emergency_cmd{false};
std::atomic<bool>  g_mission_going_cmd{false};

// IMU drop counter — incremented by imu_task whenever osMessageQueuePut
// to imuQueueHandle returns non-OK (queue full), read by ros_interface
// during the ~10 s periodic re-sync.
std::atomic<uint32_t> g_imu_drop_count{0};

// ROS command queue
QueueHandle_t g_ros_cmd_queue = nullptr;
