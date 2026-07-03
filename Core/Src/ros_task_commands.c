/**
 * @file    ros_task_commands.c
 * @brief   No-op stubs for the mission-orchestration commands.
 *
 * See ros_task_commands.h. These keep app_task compiling until the
 * micro-ROS action layer (or a std_srvs/std_msgs equivalent) is wired in.
 */
#include "ros_task_commands.h"

bool send_set_mission_command(int mission_id)
{
    (void)mission_id;
    /* No real action layer yet: report "dispatched" so app_task stops
     * re-issuing, but nothing sets g_set_mission_ready (the run path
     * stays gated until the ROS layer is integrated). */
    return true;
}

bool send_start_mission_command(int mission_id)
{
    (void)mission_id;
    return true;
}

void send_cancel_mission_command(void)
{
    /* no-op */
}

void send_cancel_set_mission_command(void)
{
    /* no-op */
}
