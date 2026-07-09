/**
 * @file    ebs_manager.hpp
 * @brief   Emergency Brake System manager - C++ singleton
 */

#ifndef INC_EBS_MANAGER_HPP_
#define INC_EBS_MANAGER_HPP_

#include <cstdint>
#include <stdbool.h>

extern "C" {
    #include "hardware_io.h"
}

enum class EBSInitState {
    Start,
    WaitLow,
    CheckPressure,
    WaitTS,
    CheckActuator1,
    WaitInterActuatorCheck,
    CheckActuator2,
    Failed,
    Done
};

class EbsManager {
public:
    // Singleton access
    static EbsManager& getInstance() {
        static EbsManager instance;
        return instance;
    }

    // Non-blocking initialization sequence (RTOS-friendly)
    EBSInitState initSequenceStep();

    // Pressure checks
    bool checkStoragePressures();
    bool checkBrakeLinePressure();
    bool ASBChecksOK();
    bool SafeManual();

    // Brake actuators only (D1/D2) — SDC (D4) left untouched.
    void engageBrakes();     // fire    (D1/D2 LOW)
    void disengageBrakes();  // release (D1/D2 HIGH)

    // Full AS brake state = brakes AND the AS's SDC contribution together.
    //   activateEBS()   = engageBrakes()    + open  SDC (D4 LOW)  — the rules
    //                     safe state (FS-Rules T15.3.5/T11.9.5: brakes fired
    //                     AND SDC open).
    //   deactivateEBS() = disengageBrakes() + close SDC (D4 HIGH) — the AS
    //                     stands down: brakes released and the AS closes its
    //                     own SDC part (the loop may still be opened elsewhere).
    // Anyone needing the SDC alone calls hardware_io_set_as_close_sdc() directly.
    void activateEBS();
    void deactivateEBS();
    void reset();

    // Status queries
    EBSInitState getInitState() const { return init_state_; }

    // Delete copy constructor and assignment operator
    EbsManager(const EbsManager&) = delete;
    EbsManager& operator=(const EbsManager&) = delete;

private:
    EbsManager();

    // Pressure thresholds (bar). Brake-line has no local threshold — the
    // ECU sends the 0x505 over-limit verdict against its own BrakeDvHardRaw.
    static constexpr float ACTUATOR_STORAGE_THRESHOLD = 1.0f;
    static constexpr float EMPTY_ACTUATOR_STORAGE_THRESHOLD = 0.1f;

    // Timing constants (ms)
    static constexpr uint32_t GENERAL_TIMEOUT_MS = 5000;
    static constexpr uint32_t INTER_ACTUATOR_WAIT_MS = 5000;

    // State variables
    EBSInitState init_state_ = EBSInitState::Start;
    uint32_t start_time_ = 0;
};

#endif /* INC_EBS_MANAGER_HPP_ */
