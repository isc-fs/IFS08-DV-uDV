#pragma once

#include <cstdint>
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
extern std::atomic<bool>  g_set_mission_in_progress;
extern std::atomic<bool>  g_set_mission_ready;

// Stock-typed pipeline interface (see dv_interface.h). Written by the
// ros_task micro-ROS callbacks (via the C bridge), read by AppTask.
extern std::atomic<uint8_t>  g_dv_status;           // latest dv/status byte
extern std::atomic<uint32_t> g_dv_status_stamp_ms;  // its arrival tick (0 = never)
extern std::atomic<uint32_t> g_ctrl_cmd_stamp_ms;   // last ctrl/cmd tick (0 = never)

// State telemetry snapshot, owned by AppTask and published by RosTask
extern std::atomic<uint8_t> g_telemetry_as_state;
extern std::atomic<uint8_t> g_telemetry_ebs_init_state;
extern std::atomic<bool>    g_telemetry_asms_on;
extern std::atomic<bool>    g_telemetry_ts_active;
extern std::atomic<bool>    g_telemetry_sdc_res_open;
extern std::atomic<bool>    g_telemetry_brakes_engaged;
extern std::atomic<bool>    g_telemetry_r2d;
extern std::atomic<bool>    g_telemetry_vehicle_standstill;
extern std::atomic<bool>    g_telemetry_mission_selected;
extern std::atomic<bool>    g_telemetry_mission_finished;
extern std::atomic<bool>    g_telemetry_abs_checks_ok;
extern std::atomic<bool>    g_telemetry_ebs_activated;
// EBS air-tank storage pressures (bar), sampled by AppTask (sole ADC owner) so
// pit-diag/ros can publish them without racing hadc3. Real sensor reading even
// when BENCH_STUB_EBS_SENSORS fakes the gate — so the bench sees true pressure.
extern std::atomic<float>   g_telemetry_ebs_pressure1_bar;
extern std::atomic<float>   g_telemetry_ebs_pressure2_bar;

// Counts IMU samples the imu_task tried to enqueue but couldn't (queue
// full because the ros_task is back-pressured by USB CDC).  Published
// to ROS by ros_interface alongside the existing imu/status debug topic.
extern std::atomic<uint32_t> g_imu_drop_count;

// ROS command queue
extern QueueHandle_t g_ros_cmd_queue;

#ifdef __cplusplus
}
#endif
