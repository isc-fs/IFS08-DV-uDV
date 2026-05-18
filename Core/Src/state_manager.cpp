/**
 * @file    state_manager.cpp
 * @brief   Vehicle state machine implementation
 */

#include "state_manager.hpp"
#include <atomic>

extern "C" {
    #include "hardware_io.h"
}

// External globals from other modules
extern std::atomic<bool> g_can_r2d;
extern std::atomic<bool> g_can_vehicle_standstill;
extern std::atomic<int> g_can_mission_id;
extern std::atomic<bool> g_can_ts_active;
extern std::atomic<float> g_can_brake_pressure;
extern std::atomic<bool> g_finished_cmd;

StateManager::StateManager()
{
    // Initialize with default signals and OFF state
    state_ = ASState::OFF;
}

void StateManager::updateSignals()
{
    // Read from HardwareIO (digital inputs)
    signals_.asms_on = hardware_io_read_asms_on();
    signals_.ts_active = g_can_ts_active.load();
    signals_.sdc_res_open = hardware_io_read_sdc_res_open();

    // Read from EbsManager
    signals_.ebs_activated = ebs_.isActive();
    signals_.abs_checks_ok = ebs_.ASBChecksOK();
    signals_.brakes_engaged = ebs_.checkBrakeLinePressure();

    // Read from CAN globals (via atomics)
    signals_.r2d = g_can_r2d.load();
    signals_.vehicle_standstill = g_can_vehicle_standstill.load();
    int mission_id = g_can_mission_id.load();
    signals_.mission_selected = (mission_id > 0);

    // Read from ROS globals (via atomics)
    signals_.mission_finished = g_finished_cmd.load();
}

void StateManager::updateState()
{
    // State machine logic: priority is emergency > then normal flow
    if (signals_.ebs_activated)
    {
        // EBS is active - emergency state
        if (signals_.mission_finished && signals_.vehicle_standstill)
        {
            // Mission done and stopped -> transition to FINISHED or EMERGENCY based on SDC
            state_ = signals_.sdc_res_open ? ASState::EMERGENCY : ASState::FINISHED;
        }
        else
        {
            // Still in motion or mission not finished -> EMERGENCY state
            state_ = ASState::EMERGENCY;
        }
    }
    else
    {
        // EBS not active - normal operation states
        if (signals_.mission_selected && signals_.asms_on && 
            signals_.abs_checks_ok && signals_.ts_active)
        {
            // All preconditions met
            if (signals_.r2d)
            {
                // Ready-to-drive signal received -> DRIVING
                state_ = ASState::DRIVING;
            }
            else if (signals_.brakes_engaged)
            {
                // Brakes engaged, waiting for R2D -> READY
                state_ = ASState::READY;
            }
            else
            {
                // Not ready yet -> OFF
                state_ = ASState::OFF;
            }
        }
        else
        {
            // Preconditions not met -> OFF
            state_ = ASState::OFF;
        }
    }
}

void StateManager::update()
{
    updateSignals();
    updateState();
}

void StateManager::reset()
{
    state_ = ASState::OFF;
    updateSignals();
}

uint8_t StateManager::getAssiStatusCode(ASState state)
{
    // Convert AS state to ASSI (Autonomous System Status Indicator) CAN message code
    switch (state)
    {
        case ASState::OFF:
            return 0x00;
        case ASState::READY:
            return 0x02;
        case ASState::DRIVING:
            return 0x03;
        case ASState::EMERGENCY:
            return 0x01;
        case ASState::FINISHED:
            return 0x04;
        default:
            return 0x00;
    }
}
