/**
 * @file    app_task.cpp
 * @brief   Application task implementation - Main state machine coordinator
 */

#include "app_task.h"

#include <cstdint>
#include <cstring>

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "cmsis_os.h"
    #include "hardware_io.h"
}

#include "state_manager.hpp"
#include "ebs_manager.hpp"
#include "ros_task_commands.h"
#include "can_interface.hpp"
#include "can_task.h"
#include "ros_globals.h"
#include "can_globals.h"

/**
 * @brief Reset ROS globals to default state
 */
static void reset_ros_globals(void)
{
    g_mission_going_cmd.store(false);
    g_accel_cmd.store(0.0f);
    g_steer_cmd.store(0.0f);
    g_emergency_cmd.store(false);
    g_finished_cmd.store(false);
}

/**
 * @brief Reset CAN globals to default state
 */
static void reset_can_globals(void)
{
    g_can_listen_go.store(false);
    g_can_mission_id.store(0);
    g_can_r2d.store(false);
    g_can_vehicle_standstill.store(true);
}

/**
 * @brief Reset all systems
 */
static void reset_all(void)
{
    // Cancel ongoing ROS mission if any
    if (g_mission_going_cmd.load())
    {
        send_cancel_mission_command();
    }

    // Reset global states
    reset_ros_globals();
    reset_can_globals();

    // Reset EBS and StateManager
    EbsManager::getInstance().reset();
    StateManager::getInstance().reset();

    // Clear task queues
    if (g_ros_cmd_queue != NULL)
    {
        xQueueReset(g_ros_cmd_queue);
    }
    if (g_can_cmd_queue != NULL)
    {
        xQueueReset(g_can_cmd_queue);
    }

    g_reset_cmd.store(false);
}

/**
 * @brief C wrapper function that FreeRTOS calls
 */
extern "C" void StartAppTask(void *argument)
{
    (void)argument;  // Unused parameter

    EbsManager& ebs = EbsManager::getInstance();
    StateManager& state_mgr = StateManager::getInstance();

    EBSInitState ebs_state = EBSInitState::Start;
    ASState as_state = ASState::OFF;
    ASState last_as_state = ASState::OFF;  // Track last state for ASSI updates
    uint32_t ready_start_time = 0;

    // Send initial OFF status via CAN queue
    CanInterface::getInstance().sendAssiStatus(StateManager::getAssiStatusCode(as_state));

    // Wait until CAN task creates its queue
    while (g_can_cmd_queue == NULL)
    {
        osDelay(10);
    }

    // Wait until ROS task creates its queue
    while (g_ros_cmd_queue == NULL)
    {
        osDelay(10);
    }

    // Main control loop
    while (1)
    {
        // Check for reset command (issued by external supervisor)
        if (g_reset_cmd.load())
        {
            reset_all();
        }

        // Update state machine with all current signals
        state_mgr.update();
        as_state = state_mgr.getState();
        bool asms_on = hardware_io_read_asms_on();

        // Send ASSI status if state changed
        if (as_state != last_as_state)
        {
            CanInterface::getInstance().sendAssiStatus(StateManager::getAssiStatusCode(as_state));
            last_as_state = as_state;
        }

        // Kick watchdog (required for safety; max loop time ~50ms before watchdog triggers)
        if (ebs_state != EBSInitState::WaitLow)
        {
            hardware_io_watchdog_kick();
        }

        // State machine dispatcher
        if (asms_on)
        {
            // Autonomous mode enabled

            // Reset state tracking when transitioning out of READY/DRIVING
            if (as_state != ASState::READY)
            {
                ready_start_time = 0;
                g_can_listen_go.store(false);
            }

            // Send zero control when not driving (safety)
            if (as_state != ASState::DRIVING)
            {
                CanInterface::getInstance().sendControl(0.0f, 0.0f);
            }

            // State-specific logic
            switch (as_state)
            {
                case ASState::OFF:
                    // Perform EBS initialization sequence steps
                    if (ebs_state != EBSInitState::Done && ebs_state != EBSInitState::Failed)
                    {
                        ebs_state = ebs.initSequenceStep();
                    }
                    break;

                case ASState::READY:
                    // Wait 5 seconds in READY state before signaling "go" to CAN
                    if (ready_start_time == 0)
                    {
                        ready_start_time = hardware_io_now_ms();
                    }
                    else if (hardware_io_now_ms() - ready_start_time > 5000)
                    {
                        g_can_listen_go.store(true);
                    }
                    break;

                case ASState::DRIVING:
                    // Mission should already be active, but ensure it starts
                    if (!g_mission_going_cmd.load())
                    {
                        send_start_mission_command(g_can_mission_id.load());
                    }
                    else
                    {
                        // Mission is running - check for emergency/finish conditions
                        if (g_emergency_cmd.load() || g_finished_cmd.load())
                        {
                            // Trigger emergency brake
                            ebs.activateEBS();

                            // Cancel mission if still active
                            if (g_mission_going_cmd.load())
                            {
                                send_cancel_mission_command();
                            }
                        }
                        else
                        {
                            // Send control commands continuously (ECU expects constant stream)
                            CanInterface::getInstance().sendControl(g_accel_cmd.load(), g_steer_cmd.load());
                        }
                    }
                    break;

                case ASState::EMERGENCY:
                    // EBS should already be active, but ensure it is
                    ebs.activateEBS();

                    // Cancel any active mission
                    if (g_mission_going_cmd.load())
                    {
                        send_cancel_mission_command();
                    }
                    break;

                case ASState::FINISHED:
                    // Mission complete - EBS already active, just ensure mission is cancelled
                    if (g_mission_going_cmd.load())
                    {
                        send_cancel_mission_command();
                    }
                    break;
            }
        }
        else
        {
            // Manual mode: verify safe conditions (empty pressure tanks)
            ebs.SafeManual();
        }

        // Small delay to prevent CPU hogging (but watchdog timeout < 50ms)
        osDelay(1);
    }
}
