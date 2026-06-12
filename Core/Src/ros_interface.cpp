#include "ros_interface.hpp"
#include "imu_task.h"
#include "can_globals.h"  // g_steering_*, g_res_*, g_can_mission_id atomics
#include <string.h>
#include <stdlib.h>
#include <cmath>

extern "C" {
    #include "cmsis_os.h"
    #include "main.h"
    #include "dwt_time.h"
    #include <rcl/rcl.h>
    #include <rcl/error_handling.h>
    #include <rclc/rclc.h>
    #include <uxr/client/transport.h>
    #include <rmw_microxrcedds_c/config.h>
    #include <rmw_microros/rmw_microros.h>
    #include <rmw_microros/time_sync.h>
    #include <rosidl_runtime_c/string_functions.h>
}

/* Defines for unit conversions */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define G_TO_MS2   9.80665f
#define DPS_TO_RAD (float)(M_PI / 180.0)
#define TIME_SYNC_TIMEOUT_MS  1000
#define TIME_SYNC_INTERVAL    4000  // re-sync every N samples (~10s at 400Hz)
#define SLOW_PUB_INTERVAL       40  // steering/RES/AMI publish cadence (~10 Hz at 400 Hz IMU)
#define RES_TIMEOUT_MS         150  // RES PDO timeout: expect every ~30 ms; allow 5x slack

// --- Helpers --------------------------------------------------------------

namespace {
    // Strict-equality check between an rclc string and a C literal.
    static bool ros_str_equals(const rosidl_runtime_c__String &s, const char *literal)
    {
        if (s.data == NULL || literal == NULL) return false;
        const size_t lit_len = strlen(literal);
        if (s.size != lit_len) return false;
        return strncmp(s.data, literal, lit_len) == 0;
    }
}

// --- Construction / destruction ------------------------------------------

RosInterface::RosInterface() {
    // Zero-initialize all members
    memset(&support, 0, sizeof(support));
    allocator = rcl_get_default_allocator();
    memset(&node, 0, sizeof(node));
    memset(&executor, 0, sizeof(executor));

    memset(&set_mission_client, 0, sizeof(set_mission_client));
    memset(&goal_set_mission, 0, sizeof(goal_set_mission));
    memset(&res_set_mission, 0, sizeof(res_set_mission));
    memset(&fb_set_mission, 0, sizeof(fb_set_mission));
    current_set_mission_handle = NULL;

    memset(&runtime_control_client, 0, sizeof(runtime_control_client));
    memset(&goal_runtime_control, 0, sizeof(goal_runtime_control));
    memset(&res_runtime_control, 0, sizeof(res_runtime_control));
    memset(&fb_runtime_control, 0, sizeof(fb_runtime_control));
    current_runtime_control_handle = NULL;

    // IMU publisher initialization
    memset(&imu_pub, 0, sizeof(imu_pub));
    memset(&imu_debug_pub, 0, sizeof(imu_debug_pub));
    memset(&imu_drop_count_pub, 0, sizeof(imu_drop_count_pub));
    memset(&imu_msg, 0, sizeof(imu_msg));
    memset(&debug_msg, 0, sizeof(debug_msg));
    memset(&drop_count_msg, 0, sizeof(drop_count_msg));
    debug_msg.data = 0;
    drop_count_msg.data = 0;

    // Slow (~10 Hz) publishers re-ported from v0.1
    memset(&steering_pub, 0, sizeof(steering_pub));
    memset(&res_pub, 0, sizeof(res_pub));
    memset(&ami_pub, 0, sizeof(ami_pub));
    memset(&steering_msg, 0, sizeof(steering_msg));
    memset(&res_msg, 0, sizeof(res_msg));
    memset(&ami_msg, 0, sizeof(ami_msg));
    steering_msg.data = 0.0f;
    res_msg.data      = -1;    // -1 until first PDO arrives
    ami_msg.data      = -1;    // -1 until first AMI mission frame
    slow_pub_counter  = 0;

    // /cmd_test subscriber state
    memset(&cmd_test_sub, 0, sizeof(cmd_test_sub));
    memset(&cmd_test_msg, 0, sizeof(cmd_test_msg));

    // Time sync initialization (DWT is initialized once in main.c via dwt_init())
    epoch_offset_ns = 0;
    sync_counter = 0;
}

RosInterface::~RosInterface() {
    // Clean up all allocated resources
    (void)rcl_publisher_fini(&imu_pub, &node);
    (void)rcl_publisher_fini(&imu_debug_pub, &node);
    (void)rcl_publisher_fini(&imu_drop_count_pub, &node);
    (void)rcl_publisher_fini(&steering_pub, &node);
    (void)rcl_publisher_fini(&res_pub, &node);
    (void)rcl_publisher_fini(&ami_pub, &node);
    (void)rcl_subscription_fini(&cmd_test_sub, &node);
    (void)rclc_action_client_fini(&set_mission_client, &node);
    (void)rclc_action_client_fini(&runtime_control_client, &node);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

// --- Initialization ------------------------------------------------------

void RosInterface::init() {
    // DWT cycle counter is initialized once in main() via dwt_init().

    // Initialize micro-ROS support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Initialize node
    rclc_node_init_default(&node, "supervisor_node", "", &support);

    // SetMission action client — mission lifecycle (configure/activate)
    rclc_action_client_init_default(
        &set_mission_client,
        &node,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(dv_msgs, SetMission),
        "set_mission");

    // RuntimeControl action client — runtime control loop (throttle/steering)
    rclc_action_client_init_default(
        &runtime_control_client,
        &node,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(dv_msgs, RuntimeControl),
        "runtime_control");

    // Executor: 2 action clients (each internally uses several slots) +
    // 1 subscription.  Keep slot count generous.
    rclc_executor_init(&executor, &support.context, 6, &allocator);

    // Wire both action clients into the executor
    rclc_executor_add_action_client(
        &executor,
        &set_mission_client,
        1,
        &res_set_mission,
        &fb_set_mission,
        &RosInterface::set_mission_goal_callback,
        &RosInterface::set_mission_feedback_callback,
        &RosInterface::set_mission_result_callback,
        &RosInterface::set_mission_cancel_callback,
        this);

    rclc_executor_add_action_client(
        &executor,
        &runtime_control_client,
        1,
        &res_runtime_control,
        &fb_runtime_control,
        &RosInterface::runtime_control_goal_callback,
        &RosInterface::runtime_control_feedback_callback,
        &RosInterface::runtime_control_result_callback,
        &RosInterface::runtime_control_cancel_callback,
        this);

    // Initialize IMU publishers (best-effort QoS for maximum throughput)
    rclc_publisher_init_best_effort(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw");

    rclc_publisher_init_default(
        &imu_debug_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "imu/status");

    rclc_publisher_init_default(
        &imu_drop_count_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "imu/drop_count");

    // Slow telemetry publishers (~10 Hz)
    rclc_publisher_init_best_effort(
        &steering_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "steering/data");

    rclc_publisher_init_best_effort(
        &res_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "res/status");

    rclc_publisher_init_best_effort(
        &ami_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ami/mission");

    // /cmd_test diagnostic subscriber — toggle OK_STATUS LED
    rclc_subscription_init_default(
        &cmd_test_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "cmd_test");
    rclc_executor_add_subscription(
        &executor,
        &cmd_test_sub,
        &cmd_test_msg,
        &RosInterface::cmd_test_callback,
        ON_NEW_DATA);

    // Prepare IMU message (set static fields once)
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1.0;  // Orientation not available

    // BMI088 datasheet noise specs (diagonal covariance matrices)
    imu_msg.linear_acceleration_covariance[0] = 8.25e-4;
    imu_msg.linear_acceleration_covariance[4] = 8.25e-4;
    imu_msg.linear_acceleration_covariance[8] = 8.25e-4;
    imu_msg.angular_velocity_covariance[0] = 1.37e-5;
    imu_msg.angular_velocity_covariance[4] = 1.37e-5;
    imu_msg.angular_velocity_covariance[8] = 1.37e-5;

    // Time synchronization: compute offset between local clock and agent epoch
    while (!rmw_uros_epoch_synchronized()) {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        if (!rmw_uros_epoch_synchronized()) {
            osDelay(100);
        }
    }

    int64_t sync_epoch_ns = rmw_uros_epoch_nanos();
    uint64_t sync_dwt_us = dwt_micros();
    epoch_offset_ns = sync_epoch_ns - (int64_t)sync_dwt_us * 1000LL;
    sync_counter = 0;
}

// --- IMU pipeline (unchanged from pre-migration) --------------------------

void RosInterface::publishImuSample(const imu_sample_t *sample)
{
    if (!sample) return;

    int64_t stamp_ns = epoch_offset_ns + (int64_t)sample->timestamp_us * 1000LL;
    imu_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

    imu_msg.linear_acceleration.x = sample->imu.ax_g * G_TO_MS2;
    imu_msg.linear_acceleration.y = sample->imu.ay_g * G_TO_MS2;
    imu_msg.linear_acceleration.z = sample->imu.az_g * G_TO_MS2;

    imu_msg.angular_velocity.x = sample->imu.gx_dps * DPS_TO_RAD;
    imu_msg.angular_velocity.y = sample->imu.gy_dps * DPS_TO_RAD;
    imu_msg.angular_velocity.z = sample->imu.gz_dps * DPS_TO_RAD;

    (void)rcl_publish(&imu_pub, &imu_msg, NULL);

    // Slow telemetry publishers (~10 Hz)
    if (++slow_pub_counter >= SLOW_PUB_INTERVAL)
    {
        slow_pub_counter = 0;

        steering_msg.data = g_steering_angle_raw.load() * 0.1f;
        (void)rcl_publish(&steering_pub, &steering_msg, NULL);

        uint32_t last_tick = g_res_last_rx_tick.load();
        uint32_t now_tick  = osKernelGetTickCount();
        if (last_tick == 0U) {
            res_msg.data = -1;
        } else if ((now_tick - last_tick) > RES_TIMEOUT_MS) {
            res_msg.data = -1;
        } else if (g_res_estop.load()) {
            res_msg.data = 1;
        } else {
            res_msg.data = 0;
        }
        (void)rcl_publish(&res_pub, &res_msg, NULL);

        ami_msg.data = g_can_mission_id.load();
        (void)rcl_publish(&ami_pub, &ami_msg, NULL);
    }

    // Periodic time re-sync and debug publish (~every 10s)
    if (++sync_counter >= TIME_SYNC_INTERVAL)
    {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        int64_t new_epoch_ns = rmw_uros_epoch_nanos();
        epoch_offset_ns = new_epoch_ns - (int64_t)sample->timestamp_us * 1000LL;

        debug_msg.data = imu_debug_status;
        (void)rcl_publish(&imu_debug_pub, &debug_msg, NULL);

        drop_count_msg.data = g_imu_drop_count.load(std::memory_order_relaxed);
        (void)rcl_publish(&imu_drop_count_pub, &drop_count_msg, NULL);

        sync_counter = 0;
    }
}

void RosInterface::spin_some() {
    rclc_executor_spin_some(&executor, 5);
}

// --- /cmd_test diagnostic subscriber --------------------------------------

void RosInterface::cmd_test_callback(const void *msgin)
{
    (void)msgin;
    HAL_GPIO_TogglePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin);
}

// --- SetMission action callbacks ------------------------------------------

void RosInterface::set_mission_goal_callback(rclc_action_goal_handle_t *goal_handle, bool accepted, void *context)
{
    RosInterface *self = static_cast<RosInterface*>(context);
    self->current_set_mission_handle = accepted ? goal_handle : NULL;
}

void RosInterface::set_mission_feedback_callback(rclc_action_goal_handle_t *goal_handle, void *ros_feedback, void *context)
{
    (void)goal_handle;
    (void)context;
    (void)ros_feedback;
    // SetMission feedback carries lifecycle stage strings; informational only.
    // The autonomy stack may stream "configuring", "activating", etc.
    // Nothing to wire to the firmware control loop here.
}

void RosInterface::set_mission_result_callback(rclc_action_goal_handle_t *goal_handle, void *ros_result, void *context)
{
    (void)goal_handle;
    RosInterface *self = static_cast<RosInterface*>(context);
    auto *res = (dv_msgs__action__SetMission_GetResult_Response *)ros_result;

    if (res && !res->result.success) {
        // SetMission rejected by the autonomy stack — surface as emergency
        // so the safety machine treats it as a fault.
        g_emergency_cmd.store(true);
    }
    self->current_set_mission_handle = NULL;
}

void RosInterface::set_mission_cancel_callback(rclc_action_goal_handle_t *goal_handle, bool cancelled, void *context)
{
    (void)goal_handle;
    (void)cancelled;
    RosInterface *self = static_cast<RosInterface*>(context);
    self->current_set_mission_handle = NULL;
}

// --- RuntimeControl action callbacks --------------------------------------

void RosInterface::runtime_control_goal_callback(rclc_action_goal_handle_t *goal_handle, bool accepted, void *context)
{
    RosInterface *self = static_cast<RosInterface*>(context);
    if (accepted) {
        self->current_runtime_control_handle = goal_handle;
        g_mission_going_cmd.store(true);
    } else {
        self->current_runtime_control_handle = NULL;
        g_mission_going_cmd.store(false);
    }
}

void RosInterface::runtime_control_feedback_callback(rclc_action_goal_handle_t *goal_handle, void *ros_feedback, void *context)
{
    (void)goal_handle;
    (void)context;
    auto *fb = (dv_msgs__action__RuntimeControl_Feedback *)ros_feedback;

    if (fb) {
        // throttle: [-1, 1] in autonomy stack; firmware reads g_accel_cmd as a
        // signed normalized command (negative = regen).
        g_accel_cmd.store(fb->throttle);
        g_steer_cmd.store(fb->steering);
        g_emergency_cmd.store(fb->emergency);
        g_finished_cmd.store(fb->finished);
    }
}

void RosInterface::runtime_control_result_callback(rclc_action_goal_handle_t *goal_handle, void *ros_result, void *context)
{
    (void)goal_handle;
    RosInterface *self = static_cast<RosInterface*>(context);
    auto *res = (dv_msgs__action__RuntimeControl_GetResult_Response *)ros_result;

    if (res) {
        // Outcome string from the autonomy stack.  Treat anything other than
        // "success"/"finished" as a fault and raise emergency.  Empty string =
        // success per the dv_msgs comment.
        const auto &outcome = res->result.outcome;
        const bool ok =
            outcome.data == NULL ||
            outcome.size == 0 ||
            ros_str_equals(outcome, "success") ||
            ros_str_equals(outcome, "finished");
        if (!ok) {
            g_emergency_cmd.store(true);
        }
        g_finished_cmd.store(true);
    }
    g_mission_going_cmd.store(false);
    self->current_runtime_control_handle = NULL;
}

void RosInterface::runtime_control_cancel_callback(rclc_action_goal_handle_t *goal_handle, bool cancelled, void *context)
{
    (void)goal_handle;
    RosInterface *self = static_cast<RosInterface*>(context);
    if (cancelled) {
        g_emergency_cmd.store(true);
        g_mission_going_cmd.store(false);
    }
    self->current_runtime_control_handle = NULL;
}

// --- Outgoing requests ----------------------------------------------------

void RosInterface::send_set_mission_action_goal(int mission_id)
{
    dv_msgs__action__SetMission_SendGoal_Request__init(&goal_set_mission);
    goal_set_mission.goal.mission_id = mission_id;
    for (int i = 0; i < 16; ++i) {
        goal_set_mission.goal_id.uuid[i] = (uint8_t)rand();
    }
    (void)rclc_action_send_goal_request(&set_mission_client, &goal_set_mission, NULL);
}

void RosInterface::send_runtime_control_action_goal()
{
    dv_msgs__action__RuntimeControl_SendGoal_Request__init(&goal_runtime_control);
    // RuntimeControl goal payload is empty by design.
    for (int i = 0; i < 16; ++i) {
        goal_runtime_control.goal_id.uuid[i] = (uint8_t)rand();
    }
    (void)rclc_action_send_goal_request(&runtime_control_client, &goal_runtime_control, NULL);
}

void RosInterface::cancel_mission_action()
{
    // Cancel whichever action is currently in flight.  RuntimeControl is the
    // hot path (it streams feedback); SetMission may also be open if the
    // autonomy stack is mid-configure.
    if (current_runtime_control_handle != NULL) {
        rclc_action_send_cancel_request(current_runtime_control_handle);
        current_runtime_control_handle = NULL;
    }
    if (current_set_mission_handle != NULL) {
        rclc_action_send_cancel_request(current_set_mission_handle);
        current_set_mission_handle = NULL;
    }
    g_mission_going_cmd.store(false);
}
