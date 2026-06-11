#pragma once

#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rclc/action_client.h"
#include "dv_msgs/action/set_mission.h"
#include "dv_msgs/action/runtime_control.h"
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/float32.h>

#include "ros_globals.h"
#include "imu_service.h"

#include <cstdint>

// micro-ROS client for the DV pipeline.
//
// Two action clients are used:
//   - dv_msgs::SetMission     (mission lifecycle)
//       goal     : int32 mission_id
//       result   : bool success, string message
//       feedback : string stage, time stamp  (configure/activate progress)
//
//   - dv_msgs::RuntimeControl (runtime control loop)
//       goal     : (empty)
//       result   : string outcome, string message
//       feedback : float throttle, float steering,
//                  bool emergency, bool finished, time stamp
//
// The runtime-control feedback is what populates g_accel_cmd /
// g_steer_cmd / g_emergency_cmd / g_finished_cmd at the rate the
// autonomy stack publishes it.
class RosInterface {
public:
    RosInterface();
    ~RosInterface();

    void init();
    void spin_some();
    void publishImuSample(const imu_sample_t *sample);

    // High-level command surface used by ros_task — names stable across the
    // ros2_interface → dv_msgs migration so app_task doesn't need to change.
    void send_set_mission_action_goal(int mission_id);
    void send_runtime_control_action_goal();
    void cancel_mission_action();

private:
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rclc_executor_t executor;

    // SetMission action client (mission lifecycle)
    rclc_action_client_t set_mission_client;
    dv_msgs__action__SetMission_SendGoal_Request    goal_set_mission;
    dv_msgs__action__SetMission_GetResult_Response  res_set_mission;
    dv_msgs__action__SetMission_Feedback            fb_set_mission;
    rclc_action_goal_handle_t * current_set_mission_handle;

    // RuntimeControl action client (autonomous control feedback stream)
    rclc_action_client_t runtime_control_client;
    dv_msgs__action__RuntimeControl_SendGoal_Request   goal_runtime_control;
    dv_msgs__action__RuntimeControl_GetResult_Response res_runtime_control;
    dv_msgs__action__RuntimeControl_Feedback           fb_runtime_control;
    rclc_action_goal_handle_t * current_runtime_control_handle;

    // IMU Publisher
    rcl_publisher_t imu_pub;
    rcl_publisher_t imu_debug_pub;
    rcl_publisher_t imu_drop_count_pub;
    sensor_msgs__msg__Imu imu_msg;
    std_msgs__msg__Int32 debug_msg;
    std_msgs__msg__UInt32 drop_count_msg;

    // Slow (~10 Hz) telemetry — re-ported from v0.1 release
    rcl_publisher_t steering_pub;     // /steering/data (Float32, degrees)
    rcl_publisher_t res_pub;          // /res/status   (Int32: 0=OK, 1=E-STOP, -1=TIMEOUT)
    rcl_publisher_t ami_pub;          // /ami/mission  (Int32, mission index or -1)
    std_msgs__msg__Float32 steering_msg;
    std_msgs__msg__Int32   res_msg;
    std_msgs__msg__Int32   ami_msg;

    // /cmd_test subscriber (diagnostic — toggles OK_STATUS LED on each rx)
    rcl_subscription_t cmd_test_sub;
    std_msgs__msg__Int32 cmd_test_msg;

    // Slow-publisher cadence counter (incremented every IMU sample @400 Hz)
    uint16_t slow_pub_counter;

    // Time synchronization (DWT cycle counter lives in dwt_time.c as a
    // single source of truth shared with the IMU task)
    int64_t epoch_offset_ns;
    uint16_t sync_counter;

    // --- SetMission callbacks ---
    static void set_mission_goal_callback     (rclc_action_goal_handle_t *goal_handle, bool accepted, void *context);
    static void set_mission_feedback_callback (rclc_action_goal_handle_t *goal_handle, void *ros_feedback, void *context);
    static void set_mission_result_callback   (rclc_action_goal_handle_t *goal_handle, void *ros_result,   void *context);
    static void set_mission_cancel_callback   (rclc_action_goal_handle_t *goal_handle, bool cancelled,    void *context);

    // --- RuntimeControl callbacks ---
    static void runtime_control_goal_callback     (rclc_action_goal_handle_t *goal_handle, bool accepted, void *context);
    static void runtime_control_feedback_callback (rclc_action_goal_handle_t *goal_handle, void *ros_feedback, void *context);
    static void runtime_control_result_callback   (rclc_action_goal_handle_t *goal_handle, void *ros_result,   void *context);
    static void runtime_control_cancel_callback   (rclc_action_goal_handle_t *goal_handle, bool cancelled,    void *context);

    // --- Diagnostic subscriber ---
    static void cmd_test_callback(const void *msgin);
};
