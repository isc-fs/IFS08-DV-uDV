/**
 * @file ros_task.cpp
 * @brief ROS task implementation for FreeRTOS
 * @note Integrates micro-ROS communication and command handling
 */

#include <cstdint>
#include <cstring>

// FreeRTOS includes
extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "cmsis_os.h"
    #include "main.h"
}

// ROS-related includes
#include "ros_task.h"
#include "ros_task_commands.h"
#include "ros_interface.hpp"
#include "ros_globals.h"
#include "imu_service.h"

// External queue for IMU samples
extern osMessageQueueId_t imuQueueHandle;

/**
 * @brief C wrapper function that FreeRTOS calls
 * This function bridges between C and C++
 * Implements the ROS communication task
 */
extern "C" void StartRosTask(void *argument)
{
    (void)argument;  // Unused parameter

    // Create and initialize the ROS interface
    static RosInterface ros_if;
    ros_if.init();

    // Create the queue for AppTask to send commands
    g_ros_cmd_queue = xQueueCreate(4, sizeof(RosCommandMessage));

    if (g_ros_cmd_queue == NULL)
    {
        // Queue creation failed - enter error loop
        while (1)
        {
            osDelay(1000);
        }
    }

    RosCommandMessage cmd_msg;
    imu_sample_t imu_sample;

    // Task loop - runs indefinitely until the task is deleted
    while (1)
    {
        // Process IMU samples from IMU task (non-blocking)
        if (osMessageQueueGet(imuQueueHandle, &imu_sample, NULL, 0) == osOK)
        {
            ros_if.publishImuSample(&imu_sample);
        }
        
        // Process commands from AppTask (timeout 5ms)
        if (xQueueReceive(g_ros_cmd_queue, &cmd_msg, pdMS_TO_TICKS(5)) == pdPASS)
        {
            switch (cmd_msg.cmd)
            {
                case ROS_CMD_SET_MISSION:
                    ros_if.call_set_mission_service(cmd_msg.mission_id);
                    break;

                case ROS_CMD_START_MISSION:
                    ros_if.send_start_mission_action_goal(cmd_msg.mission_id);
                    break;

                case ROS_CMD_CANCEL_MISSION:
                    ros_if.cancel_mission_action();
                    break;

                default:
                    break;
            }
        }

        // Spin the ROS executor (process callbacks)
        ros_if.spin_some();
        osDelay(2);
    }
}
