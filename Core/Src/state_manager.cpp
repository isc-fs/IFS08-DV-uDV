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
    /* TS active comes from the ECU on CAN 0x504 (VCU_ts_active), NOT from a
     * local pin.
     *
     * The old local read was `asms_on && hardware_io_read_tsms_on()` (TSMS on
     * A6). That measures the wrong thing: the TSMS is a SWITCH POSITION. It
     * says a human closed the master switch and 24 V is present on the shutdown
     * line — it says nothing about whether the tractive system is actually
     * energised. Precharge can still be in progress, and the AMS can open the
     * AIRs (cell fault, IMD, temperature) with the TSMS still physically closed.
     *
     * That gap is the dangerous direction: with the pin, an AMS-initiated TS
     * loss mid-run is INVISIBLE to us, so the state machine keeps believing the
     * TS is live and keeps commanding torque instead of tripping the
     * `!ts_on while READY/DRIVING -> EMERGENCY` rule in as_transition.hpp.
     *
     * 0x504 carries the ECU's own ok_precharge / HV-live view (chained from the
     * AMS's 0x020 ACU_ok_precharge, set iff its FSM is in Run|Charge), so it
     * means precharge complete / AIRs closed — the real thing. The historical
     * comment here said "nothing on the car transmits 0x504"; that was true once
     * and is now stale — the ECU sends it at 100 ms, ungated
     * (IFS08-CE-ECU control_task.cpp, vcu_ts_active.def). See uDV #180 / ECU isc-fs/IFS08-CE-ECU#127.
     *
     * ASMS is NOT folded in any more: it is its own signal and the AS
     * transition already consumes it separately (as_in.asms_on). */
    signals_.ts_active = can_ts_active_fresh(hardware_io_now_ms());
    // sdc_res_open comes via CAN 0x506 ("RES opened the SDC"). On-board derivation
    // from A1=RES_1_IN was considered but rejected (only 1 RES channel reaches the
    // MCU). See docs/STATE_MACHINE_INPUTS.md
    signals_.sdc_res_open = g_can_sdc_res_open.load();

    // Read from EbsManager and hardware pins.
    // Do NOT trust the hardware readback while the EBS initialization is
    // actively exercising the actuators (these states pulse the pins):
    //   CheckActuator1, WaitInterActuatorCheck, CheckActuator2
    EBSInitState es = ebs_.getInitState();
    const bool in_actuator_check = (es == EBSInitState::CheckActuator1 ||
                                   es == EBSInitState::WaitInterActuatorCheck ||
                                   es == EBSInitState::CheckActuator2);
    signals_.ebs_activated = (!in_actuator_check && hardware_io_is_ebs_active());
    /* Raw ASB stored-energy read: BOTH tanks above 3 bar (A5/A4). Read ONCE per
     * tick and reused for abs_checks_ok below — checkStoragePressures() does two
     * 640.5-cycle ADC conversions, and calling ASBChecksOK() separately would
     * double that for no new information. Raw on purpose: app_task debounces it
     * before it reaches the AS transition (a single noisy sample must not latch
     * Emergency). */
    signals_.asb_pressure_ok = ebs_.checkStoragePressures();
    /* Identical to the previous ebs_.ASBChecksOK(), inlined to reuse the read
     * above (ASBChecksOK == init Done && checkStoragePressures). */
    signals_.abs_checks_ok = (es == EBSInitState::Done) && signals_.asb_pressure_ok;
    // brakes_engaged comes via CAN 0x505 (VCU_brake_over_limit): the ECU's
    // binary verdict that its brake sensor reads above the hard-braking
    // limit (BrakeDvHardRaw, ECU-owned — no uDV-side threshold to tune).
    // See docs/STATE_MACHINE_INPUTS.md
    signals_.brakes_engaged = g_can_brake_over_limit.load();

    // TODO(inputs): R2D != GO. GO comes from the RES (CAN / hardwired A2=GO_RES);
    // R2D is given *to* the ECU. The OFF->DRIVING gate on r2d needs rework — the
    // drive trigger should be GO, with R2D as an output. Left as-is for now.
    // See docs/STATE_MACHINE_INPUTS.md
    signals_.r2d = g_can_r2d.load();
    signals_.vehicle_standstill = g_imu_vehicle_standstill.load();
    int mission_id = g_can_mission_id.load();
    /* >= 0: mission 0 is a valid 0-based AMI index; -1 = none received */
    signals_.mission_selected = (mission_id >= 0 && g_set_mission_ready.load());

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
