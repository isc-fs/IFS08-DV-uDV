/**
 * @file    state_manager.cpp
 * @brief   Vehicle state machine implementation
 */

#include "state_manager.hpp"
#include "can_globals.h"
#include "ros_globals.h"
#include <atomic>

extern "C" {
    #include "hardware_io.h"
}

// Visible threshold for considering brakes engaged (CAN pressure, units as per CAN message)
// TODO: tune this value to the real system requirement
float g_brake_pressure_threshold = 1.0f;

StateManager::StateManager()
    : ebs_(EbsManager::getInstance())
{
    // Initialize with default signals and OFF state
    state_ = ASState::OFF;
}

void StateManager::updateSignals()
{
    // Read from HardwareIO (digital inputs) and CAN atomics
    signals_.asms_on = hardware_io_read_asms_on();
    signals_.ts_active = g_can_ts_active.load();
    signals_.sdc_res_open = g_can_sdc_res_open.load();

    // Read from EbsManager and hardware pins.
    // Do NOT trust the hardware readback while the EBS initialization is
    // actively exercising the actuators (these states drive the pins low):
    //   CheckActuator1, WaitInterActuatorCheck, CheckActuator2
    EBSInitState es = ebs_.getInitState();
    const bool in_actuator_check = (es == EBSInitState::CheckActuator1 ||
                                   es == EBSInitState::WaitInterActuatorCheck ||
                                   es == EBSInitState::CheckActuator2);
    signals_.ebs_activated = (!in_actuator_check && hardware_io_is_ebs_active());
    signals_.abs_checks_ok = ebs_.ASBChecksOK();
    signals_.brakes_engaged = (g_can_brake_pressure.load() >= g_brake_pressure_threshold);

    // Read from CAN globals (via atomics)
    signals_.r2d = g_can_r2d.load();
    signals_.vehicle_standstill = g_imu_vehicle_standstill.load();
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
