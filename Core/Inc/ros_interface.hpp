#pragma once

#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rclc/action_client.h"
#include "ros2_interface/action/start_mission.h"
#include "ros2_interface/srv/set_mission.h"
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/float32.h>

#include "ros_globals.h"
#include "imu_service.h"

#include <cstdint>

class RosInterface {
public:
    RosInterface();
    ~RosInterface();

    void init();
    void spin_some();
    void publishImuSample(const imu_sample_t *sample);

    void call_set_mission_service(int mission_id);
    void send_start_mission_action_goal(int mission_id);
    void cancel_mission_action();

private:
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rclc_executor_t executor;

    // Service Client
    rcl_client_t set_mission_client;
    ros2_interface__srv__SetMission_Request req_set_mission;
    ros2_interface__srv__SetMission_Response res_set_mission;

    // Action Client
    rclc_action_client_t start_mission_client;
    ros2_interface__action__StartMission_SendGoal_Request goal_start_mission;
    ros2_interface__action__StartMission_GetResult_Response res_start_mission;
    ros2_interface__action__StartMission_Feedback msg_feedback;
    rclc_action_goal_handle_t * current_goal_handle;

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

    static void start_mission_goal_callback(rclc_action_goal_handle_t * goal_handle, bool accepted, void * context);
    static void start_mission_feedback_callback(rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context);
    static void start_mission_result_callback(rclc_action_goal_handle_t * goal_handle, void * ros_result, void * context);
    static void start_mission_cancel_callback(rclc_action_goal_handle_t * goal_handle, bool cancelled, void * context);
    static void set_mission_callback(const void * ros_service_response);
    static void cmd_test_callback(const void * msgin);
};
