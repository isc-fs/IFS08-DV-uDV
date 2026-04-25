#include "ros_interface.hpp"
#include <string.h>
#include <stdlib.h>
#include <cmath>

extern "C" {
    #include "cmsis_os.h"
    #include "main.h"
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

RosInterface::RosInterface() {
    // Zero-initialize all members
    memset(&support, 0, sizeof(support));
    allocator = rcl_get_default_allocator();
    memset(&node, 0, sizeof(node));
    memset(&executor, 0, sizeof(executor));
    memset(&set_mission_client, 0, sizeof(set_mission_client));
    memset(&req_set_mission, 0, sizeof(req_set_mission));
    memset(&res_set_mission, 0, sizeof(res_set_mission));
    memset(&start_mission_client, 0, sizeof(start_mission_client));
    memset(&goal_start_mission, 0, sizeof(goal_start_mission));
    memset(&msg_feedback, 0, sizeof(msg_feedback));
    current_goal_handle = NULL;
    
    // IMU publisher initialization
    memset(&imu_pub, 0, sizeof(imu_pub));
    memset(&imu_debug_pub, 0, sizeof(imu_debug_pub));
    memset(&imu_msg, 0, sizeof(imu_msg));
    memset(&debug_msg, 0, sizeof(debug_msg));
    debug_msg.data = 0;
    
    // Time sync and DWT initialization
    epoch_offset_ns = 0;
    sync_counter = 0;
    dwt_last = 0;
    dwt_overflow_count = 0;
}

RosInterface::~RosInterface() {
    // Clean up all allocated resources
    (void)rcl_publisher_fini(&imu_pub, &node);
    (void)rcl_publisher_fini(&imu_debug_pub, &node);
    (void)rcl_client_fini(&set_mission_client, &node);
    (void)rclc_action_client_fini(&start_mission_client, &node);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

uint64_t RosInterface::dwt_micros_internal(void)
{
  uint32_t now = DWT->CYCCNT;
  if (now < dwt_last) dwt_overflow_count++;
  dwt_last = now;
  uint64_t total_cycles = (dwt_overflow_count << 32) | (uint64_t)now;
  return total_cycles / (SystemCoreClock / 1000000U);
}

void RosInterface::init() {
    // Enable DWT cycle counter for high-res timestamps (if not already enabled)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // Initialize micro-ROS support
    rclc_support_init(&support, 0, NULL, &allocator);

    // Initialize node
    rclc_node_init_default(&node, "supervisor_node", "", &support);

    // Initialize service client
    rclc_client_init_default(&set_mission_client,
                             &node,
                             ROSIDL_GET_SRV_TYPE_SUPPORT(ros2_interface, srv, SetMission),
                             "set_mission");

    // Initialize action client
    rclc_action_client_init_default(&start_mission_client,
                                    &node,
                                    ROSIDL_GET_ACTION_TYPE_SUPPORT(ros2_interface, StartMission),
                                    "start_mission");

    // Initialize executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);

    // Add service to executor
    rclc_executor_add_client(&executor, &set_mission_client, &res_set_mission, &RosInterface::set_mission_callback);

    // Add feedback and result callbacks to executor
    rclc_executor_add_action_client(&executor,
                                    &start_mission_client,
                                    1,
                                    &res_start_mission,
                                    &msg_feedback,
                                    &RosInterface::start_mission_goal_callback,
                                    &RosInterface::start_mission_feedback_callback,
                                    &RosInterface::start_mission_result_callback,
                                    &RosInterface::start_mission_cancel_callback,
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
    
    // Capture offset: agent_epoch - local_dwt at the sync moment
    int64_t sync_epoch_ns = rmw_uros_epoch_nanos();
    uint64_t sync_dwt_us = dwt_micros_internal();
    epoch_offset_ns = sync_epoch_ns - (int64_t)sync_dwt_us * 1000LL;
    sync_counter = 0;
}

void RosInterface::publishImuSample(const imu_sample_t *sample)
{
    if (!sample) return;
    
    // Timestamp: DWT capture moment + epoch offset from sync
    int64_t stamp_ns = epoch_offset_ns + (int64_t)sample->timestamp_us * 1000LL;
    imu_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

    // Linear acceleration: g -> m/s^2
    imu_msg.linear_acceleration.x = sample->imu.ax_g * G_TO_MS2;
    imu_msg.linear_acceleration.y = sample->imu.ay_g * G_TO_MS2;
    imu_msg.linear_acceleration.z = sample->imu.az_g * G_TO_MS2;

    // Angular velocity: dps -> rad/s
    imu_msg.angular_velocity.x = sample->imu.gx_dps * DPS_TO_RAD;
    imu_msg.angular_velocity.y = sample->imu.gy_dps * DPS_TO_RAD;
    imu_msg.angular_velocity.z = sample->imu.gz_dps * DPS_TO_RAD;

    (void)rcl_publish(&imu_pub, &imu_msg, NULL);

    // Periodic time re-sync and debug publish (~every 10s)
    if (++sync_counter >= TIME_SYNC_INTERVAL)
    {
        rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS);
        // Refresh epoch offset
        int64_t new_epoch_ns = rmw_uros_epoch_nanos();
        uint64_t new_dwt_us = dwt_micros_internal();
        epoch_offset_ns = new_epoch_ns - (int64_t)new_dwt_us * 1000LL;

        extern volatile int32_t imu_debug_status;
        debug_msg.data = imu_debug_status;
        (void)rcl_publish(&imu_debug_pub, &debug_msg, NULL);
        sync_counter = 0;
    }
}

void RosInterface::spin_some() {
    rclc_executor_spin_some(&executor, 5);
}

// ------------------------
// Callbacks
// ------------------------
void RosInterface::set_mission_callback(const void * ros_service_response) {
    // IDEA: ignore this for the moment and hope everything goes right as it will be sent again when the mission starts
    // or try to resend the mission (keep in mind the recursive aspect of it)
    const ros2_interface__srv__SetMission_Response * res = 
        (const ros2_interface__srv__SetMission_Response *)ros_service_response;
    if (res && res->accepted) {
        // Mission accepted - perhaps set a flag or log
    } else {
        // Mission rejected - handle error
    }
}

void RosInterface::start_mission_goal_callback(rclc_action_goal_handle_t * goal_handle, bool accepted, void * context) {
    RosInterface * self = static_cast<RosInterface*>(context);
    if (accepted) {
        // Goal was accepted
        self->current_goal_handle = goal_handle;
        g_mission_going_cmd.store(true);
    } else {
        // Goal was rejected
        // IDEA: set emergency to true (as given the ready signal the car would already be in driving state, 
        // or we could change how the flowchart is processed to dont even get to driving until the mission is accepted, so we would remain in ready, 
        // but this last option could not be under regulation)
        self->current_goal_handle = NULL;
        g_mission_going_cmd.store(false);
    }
}

void RosInterface::start_mission_feedback_callback(rclc_action_goal_handle_t * goal_handle, void * ros_feedback, void * context) {
    (void)goal_handle;
    (void)context;
    ros2_interface__action__StartMission_Feedback * fb =
        (ros2_interface__action__StartMission_Feedback *)ros_feedback;

    if (fb) {
        g_accel_cmd.store(fb->acceleration);
        g_steer_cmd.store(fb->steering);
        g_finished_cmd.store(fb->finished);
        g_emergency_cmd.store(fb->emergency);
    }
}

void RosInterface::start_mission_result_callback(rclc_action_goal_handle_t * goal_handle, void * ros_result, void * context) {
    (void)goal_handle;
    RosInterface * self = static_cast<RosInterface*>(context);
    ros2_interface__action__StartMission_GetResult_Response * res =
        (ros2_interface__action__StartMission_GetResult_Response *)ros_result;

    if (res) {
        g_finished_cmd.store(res->result.finished);
        g_emergency_cmd.store(res->result.emergency);
    }
    g_mission_going_cmd.store(false);
    self->current_goal_handle = NULL; // Reset handle on completion
}

void RosInterface::start_mission_cancel_callback(rclc_action_goal_handle_t * goal_handle, bool cancelled, void * context) {
    RosInterface * self = static_cast<RosInterface*>(context);
    if (cancelled) {
        // Handle cancellation, e.g., stop motors, log, etc.
        // IDEA: set emergency to true
        g_emergency_cmd.store(true);
        g_mission_going_cmd.store(false);
        self->current_goal_handle = NULL; // Reset handle on cancel
    } else {
        // Handle non-cancellation case if needed
    }
}

// ------------------------
// Access to interface
// ------------------------
void RosInterface::call_set_mission_service(int mission_id) {
    ros2_interface__srv__SetMission_Request__init(&req_set_mission);
    req_set_mission.mission_id = mission_id;

    int64_t sequence_number;
    (void)rcl_send_request(&set_mission_client, &req_set_mission, &sequence_number);
}

void RosInterface::send_start_mission_action_goal(int mission_id) {
    ros2_interface__action__StartMission_SendGoal_Request__init(&goal_start_mission);
    goal_start_mission.goal.mission_id = mission_id;
    // Generate a simple UUID (not cryptographically secure, but sufficient for embedded)
    for (int i = 0; i < 16; ++i) {
        goal_start_mission.goal_id.uuid[i] = (uint8_t)rand();
    }

    rclc_action_send_goal_request(&start_mission_client,
                                  &goal_start_mission,
                                  NULL);
}

void RosInterface::cancel_mission_action() {
    if (current_goal_handle != NULL) {
        rclc_action_send_cancel_request(current_goal_handle);
        g_mission_going_cmd.store(false);
        current_goal_handle = NULL; // Reset after cancel
    }
}
