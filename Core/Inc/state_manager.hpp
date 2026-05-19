/**
 * @file    state_manager.hpp
 * @brief   Vehicle state machine manager - C++ singleton
 */

#ifndef INC_STATE_MANAGER_HPP_
#define INC_STATE_MANAGER_HPP_

#include "ebs_manager.hpp"
#include <cstdint>

// Visible control for brake-engaged pressure threshold (CAN-provided).
// TODO: adjust this threshold to the correct value for your vehicle.
extern float g_brake_pressure_threshold;

enum class ASState {
    OFF,        // Autonomous system off
    READY,      // Ready to drive (brakes engaged)
    DRIVING,    // Mission in progress
    EMERGENCY,  // Emergency brake activated
    FINISHED    // Mission complete
};

struct StateManagerSignals {
    bool asms_on = false;           // ASMS on signal (hardware io)
    bool ts_active = false;         // Tractive System active (can)
    bool sdc_res_open = false;      // SDC resistor open (can)
    bool ebs_activated = false;     // EBS activated (hardware io)
    bool abs_checks_ok = false;     // All brake system checks passed (hardware io)
    bool brakes_engaged = false;    // Brakes actively engaged (can)
    bool mission_selected = false;  // Mission selected (can)
    bool r2d = false;               // Ready to Drive signal (can)
    bool vehicle_standstill = true; // Vehicle velocity near zero (can)
    bool mission_finished = false;  // Mission finished (ros)
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

    ASState getState() const { return state_; }

    // Reset to initial state
    void reset();

    /**
     * @brief Convert ASState to ASSI status code for CAN message
     * @param state AS state to convert
     * @return ASSI status code (0x00=OFF, 0x02=READY, 0x03=DRIVING, 0x01=EMERGENCY, 0x04=FINISHED)
     */
    static uint8_t getAssiStatusCode(ASState state);

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
    EbsManager& ebs_;
};

#endif /* INC_STATE_MANAGER_HPP_ */
