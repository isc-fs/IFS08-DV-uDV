#pragma once

#include <atomic>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Atomic feedback from PC, owned by RosTask
extern std::atomic<float> g_accel_cmd;
extern std::atomic<float> g_steer_cmd;
extern std::atomic<bool>  g_finished_cmd;
extern std::atomic<bool>  g_emergency_cmd;
extern std::atomic<bool>  g_mission_going_cmd;

// Counts IMU samples the imu_task tried to enqueue but couldn't (queue
// full because the ros_task is back-pressured by USB CDC).  Published
// to ROS by ros_interface alongside the existing imu/status debug topic.
extern std::atomic<uint32_t> g_imu_drop_count;

// ROS command queue
extern QueueHandle_t g_ros_cmd_queue;

#ifdef __cplusplus
}
#endif
