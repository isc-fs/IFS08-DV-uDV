/**
 * @file    ros_task_commands.h
 * @brief   Mission-orchestration command interface used by app_task.
 *
 * On fix/17 these were micro-ROS action-client calls (SetMission /
 * RuntimeControl over the custom `dv_msgs` package). That action layer
 * (ros_interface.cpp + dv_msgs) is intentionally NOT integrated on this
 * branch — it does not compile and depends on a package that does not
 * exist in the tree. These are no-op STUBS that keep app_task's state
 * machine compiling and running.
 *
 * ⚠️  Until wired to the real ROS layer, nothing sets g_set_mission_ready
 *     / g_mission_going_cmd, so the autonomous path will not advance past
 *     AS Off (mission_selected stays false). The state machine, EBS init
 *     sequence, ASSI emission and CAN-driven inputs all still work.
 *
 * TODO(ros-layer): replace with real SetMission/RuntimeControl calls (or
 * std_srvs/std_msgs equivalents) that drive the ros_globals command
 * atomics. See docs / the comparison with fix/17's ros_interface.cpp.
 */
#ifndef ROS_TASK_COMMANDS_H
#define ROS_TASK_COMMANDS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Begin mission setup. Returns true if the command was dispatched. */
bool send_set_mission_command(int mission_id);

/** Begin the run once setup is confirmed. Returns true if dispatched. */
bool send_start_mission_command(int mission_id);

/** Cancel a running mission. */
void send_cancel_mission_command(void);

/** Cancel an in-progress mission setup. */
void send_cancel_set_mission_command(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS_TASK_COMMANDS_H */
