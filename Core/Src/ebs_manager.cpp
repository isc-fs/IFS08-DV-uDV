/**
 * @file    ebs_manager.cpp
 * @brief   Emergency Brake System manager implementation
 */

#include "ebs_manager.hpp"
#include "bench_stubs.h"
#include "can_globals.h"   /* can_ts_active_fresh — 0x504 TS view (#180) */
#include <atomic>

extern std::atomic<bool> g_can_brake_over_limit;

EbsManager::EbsManager()
{
    // Ensure all members are properly initialized
    init_state_ = EBSInitState::Start;
    start_time_ = 0;
}

EBSInitState EbsManager::initSequenceStep()
{
    /* Bench stub (bench_stubs.h toggle, 0 on dev — this branch folds away):
     * no pneumatics/SDC on the bench, so the real self-check below can never
     * reach Done — report Done at once so the AS state machine can arm. See
     * bench_stubs.h (incl. the skipped SDC-close caveat). */
    if (BENCH_STUB_EBS_INIT)
    {
        init_state_ = EBSInitState::Done;
        return init_state_;
    }

    switch (init_state_)
    {
        case EBSInitState::Start:
            /* The former SDC-ready gate read hardware_io_read_sdc_is_ready() (A1),
             * but A1 physically carries RES_1_IN — a dead/mislabeled signal, NOT
             * an AS-SDC-ready feedback (docs/STATE_MACHINE_INPUTS.md "Cleanup
             * noted"). There is no valid SDC-ready input (D4 is output-only), so
             * the gate is REMOVED: it can never pass on the real car and blocked
             * the whole self-check unless a bench stub bypassed it. Advance to the
             * real first check. Kept as a distinct state so the 0x7A9 / pit-diag
             * init-state numbering is unchanged. */
            init_state_ = EBSInitState::WaitLow;
            break;

        case EBSInitState::WaitLow:
            /* Formerly required that same dead read to fall LOW within 5 s or it
             * Failed — impossible for a steady signal. Gate removed; go straight
             * to the tank-pressure check (which is what actually closes the SDC). */
            init_state_ = EBSInitState::CheckPressure;
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

        // EBS actuator self-test (FS-Rules T15.2). Polarity: LOW = fire
        // (build brake pressure), HIGH = release. Each actuator is fired
        // in turn, its brake-line pressure verified, then released; after
        // Done both are released (HIGH) ready for normal operation.
        case EBSInitState::WaitTS:
            /* TS comes from the ECU on CAN 0x504, not the TSMS pin — same
             * source and same reasoning as StateManager::updateSignals() (see
             * the comment there; #180). ASMS stays an explicit, separate
             * condition rather than being folded into "TS active".
             * On a bench with no ECU this needs BENCH_STUB_TS=1, else the
             * self-check parks here forever. */
            if (hardware_io_read_asms_on() &&
                can_ts_active_fresh(hardware_io_now_ms()))
            {
                init_state_ = EBSInitState::CheckActuator1;
                hardware_io_enable_ebs_actuator_1(false);  // fire A1 (LOW)
            }
            break;

        case EBSInitState::CheckActuator1:
            if (checkBrakeLinePressure())
            {
                init_state_ = EBSInitState::WaitInterActuatorCheck;
                hardware_io_enable_ebs_actuator_1(true);   // release A1 (HIGH)
                start_time_ = hardware_io_now_ms();
            }
            break;

        case EBSInitState::WaitInterActuatorCheck:
            if (hardware_io_now_ms() - start_time_ > INTER_ACTUATOR_WAIT_MS)
            {
                init_state_ = EBSInitState::CheckActuator2;
                hardware_io_enable_ebs_actuator_2(false);  // fire A2 (LOW)
            }
            break;

        case EBSInitState::CheckActuator2:
            if (checkBrakeLinePressure())
            {
                init_state_ = EBSInitState::Done;
                hardware_io_enable_ebs_actuator_2(true);    // release A2 (HIGH)
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
    /* BENCH_STUB_EBS_SENSORS: fake the pneumatic tank pressure sensors (A5/A4)
     * so the self-check passes with no air. The valves still actuate for real;
     * this only bypasses the reading. Folds away on a car build. */
    if (BENCH_STUB_EBS_SENSORS)
        return true;

    float a1_p = hardware_io_read_actuator1_storage_pressure();
    float a2_p = hardware_io_read_actuator2_storage_pressure();

    return (a1_p > ACTUATOR_STORAGE_THRESHOLD) &&
           (a2_p > ACTUATOR_STORAGE_THRESHOLD);
}

bool EbsManager::checkBrakeLinePressure()
{
    /* BENCH_STUB_EBS_SENSORS: fake the hydraulic brake-line verdict so the
     * actuator self-test passes with no air (the actuator still fires for real
     * just above/below this check). Folds away on a car build. */
    if (BENCH_STUB_EBS_SENSORS)
        return true;

    /* The ECU sends the verdict, not the value (0x505 VCU_brake_over_limit:
     * brake_raw > config::BrakeDvHardRaw, ECU-owned threshold). The old
     * float32-bar reading + local threshold never matched what the ECU
     * actually transmits. */
    return g_can_brake_over_limit.load();
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

// EBS polarity (confirmed): LOW = fire the brake, HIGH = release. LOW is
// the power-on/reset level, so the brake is fail-safe on reset/power loss.
void EbsManager::activateEBS()
{
    hardware_io_enable_ebs_actuator_1(false);   // fire (LOW)
    hardware_io_enable_ebs_actuator_2(false);   // fire (LOW)
}

void EbsManager::deactivateEBS()
{
    hardware_io_enable_ebs_actuator_1(true);    // release (HIGH)
    hardware_io_enable_ebs_actuator_2(true);    // release (HIGH)
}

void EbsManager::reset()
{
    init_state_ = EBSInitState::Start;
    deactivateEBS();
}
