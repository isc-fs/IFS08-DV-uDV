/**
 * @file    ebs_manager.cpp
 * @brief   Emergency Brake System manager implementation
 */

#include "ebs_manager.hpp"
#include <atomic>

extern std::atomic<float> g_can_brake_pressure;

EbsManager::EbsManager()
{
    // Ensure all members are properly initialized
    init_state_ = EBSInitState::Start;
    start_time_ = 0;
}

EBSInitState EbsManager::initSequenceStep()
{
    switch (init_state_)
    {
        case EBSInitState::Start:
            if (hardware_io_read_sdc_is_ready())
            {
                init_state_ = EBSInitState::WaitLow;
                start_time_ = hardware_io_now_ms();
            }
            break;

        case EBSInitState::WaitLow:
            if (!hardware_io_read_sdc_is_ready())
            {
                init_state_ = EBSInitState::CheckPressure;
            }
            else if (hardware_io_now_ms() - start_time_ > GENERAL_TIMEOUT_MS)
            {
                init_state_ = EBSInitState::Failed;
            }
            break;

        case EBSInitState::CheckPressure:
            if (checkStoragePressures())
            {
                init_state_ = EBSInitState::WaitTS;
                hardware_io_set_as_close_sdc(true);
                start_time_ = hardware_io_now_ms();
            }
            else
            {
                init_state_ = EBSInitState::Failed;
            }
            break;

        case EBSInitState::WaitTS:
            // TS is sensed locally: TSMS (A6) AND ASMS (A3) HIGH (was CAN 0x504).
            if (hardware_io_read_asms_on() && hardware_io_read_tsms_on())
            {
                init_state_ = EBSInitState::CheckActuator1;
                hardware_io_enable_ebs_actuator_1(true);
            }
            break;

        case EBSInitState::CheckActuator1:
            if (checkBrakeLinePressure())
            {
                init_state_ = EBSInitState::WaitInterActuatorCheck;
                hardware_io_enable_ebs_actuator_1(false);
                start_time_ = hardware_io_now_ms();
            }
            break;

        case EBSInitState::WaitInterActuatorCheck:
            if (hardware_io_now_ms() - start_time_ > INTER_ACTUATOR_WAIT_MS)
            {
                init_state_ = EBSInitState::CheckActuator2;
                hardware_io_enable_ebs_actuator_2(true);
            }
            break;

        case EBSInitState::CheckActuator2:
            if (checkBrakeLinePressure())
            {
                init_state_ = EBSInitState::Done;
                hardware_io_enable_ebs_actuator_2(false);
            }
            break;

        case EBSInitState::Failed:
            // Terminal state - no action
            break;
        case EBSInitState::Done:
            // Terminal state - no action
            break;
    }

    return init_state_;
}

bool EbsManager::checkStoragePressures()
{
    float a1_p = hardware_io_read_actuator1_storage_pressure();
    float a2_p = hardware_io_read_actuator2_storage_pressure();

    return (a1_p > ACTUATOR_STORAGE_THRESHOLD) &&
           (a2_p > ACTUATOR_STORAGE_THRESHOLD);
}

bool EbsManager::checkBrakeLinePressure()
{
    float b_p = g_can_brake_pressure.load();
    return (b_p > BRAKE_PRESSURE_THRESHOLD);
}

bool EbsManager::ASBChecksOK()
{
    return (init_state_ == EBSInitState::Done) && checkStoragePressures();
}

bool EbsManager::SafeManual()
{
    float a1_p = hardware_io_read_actuator1_storage_pressure();
    float a2_p = hardware_io_read_actuator2_storage_pressure();

    return (a1_p < EMPTY_ACTUATOR_STORAGE_THRESHOLD) &&
           (a2_p < EMPTY_ACTUATOR_STORAGE_THRESHOLD);
}

void EbsManager::activateEBS()
{
    hardware_io_enable_ebs_actuator_1(false);
    hardware_io_enable_ebs_actuator_2(false);
}

void EbsManager::deactivateEBS()
{
    hardware_io_enable_ebs_actuator_1(true);
    hardware_io_enable_ebs_actuator_2(true);
}

void EbsManager::reset()
{
    init_state_ = EBSInitState::Start;
    deactivateEBS();
}
