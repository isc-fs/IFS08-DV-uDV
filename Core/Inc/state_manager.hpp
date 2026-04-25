/**
 * @file    state_manager.hpp
 * @brief   Vehicle state machine manager - C++ singleton
 */

#ifndef INC_STATE_MANAGER_HPP_
#define INC_STATE_MANAGER_HPP_

#include "ebs_manager.hpp"

enum class ASState {
    OFF,        // Autonomous system off
    READY,      // Ready to drive (brakes engaged)
    DRIVING,    // Mission in progress
    EMERGENCY,  // Emergency brake activated
    FINISHED    // Mission complete
};

struct StateManagerSignals {
    bool asms_on = false;           // ASMS on signal (from HardwareIO)
    bool ts_active = false;         // Tractive System active (from HardwareIO)
    bool sdc_res_open = false;      // SDC resistor open (from HardwareIO)
    bool ebs_activated = false;     // EBS activated (from EbsManager)
    bool abs_checks_ok = false;     // All brake system checks passed (from EbsManager)
    bool brakes_engaged = false;    // Brakes actively engaged (from EbsManager)
    bool mission_selected = false;  // Mission selected (from CanInterface via can_globals)
    bool r2d = false;               // Ready to Drive signal (from CanInterface via can_globals)
    bool vehicle_standstill = true; // Vehicle velocity near zero (from IMU via can_globals)
    bool mission_finished = false;  // Mission finished (from RosInterface via ros_globals)
};

class StateManager {
public:
    // Singleton access
    static StateManager& getInstance() {
        static StateManager instance;
        return instance;
    }

    // Update state machine
    void update();

    // Query current state and signals
    const StateManagerSignals& getSignals() const { return signals_; }
    ASState getState() const { return state_; }

    // Reset to initial state
    void reset();

    // Delete copy constructor and assignment operator
    StateManager(const StateManager&) = delete;
    StateManager& operator=(const StateManager&) = delete;

private:
    StateManager();

    // Internal methods
    void updateSignals();
    void updateState();

    // State variables
    StateManagerSignals signals_;
    ASState state_ = ASState::OFF;
    EbsManager& ebs_ = EbsManager::getInstance();
};

#endif /* INC_STATE_MANAGER_HPP_ */
