/**
 * @file    as_actuation.hpp
 * @brief   Pure AS-state safe-actuation decision (host-unit-testable).
 *
 * Extracted from app_task's main loop, the same way as_transition.hpp holds
 * the pure transition decision: given the current AS state and the ASMS master
 * switch, decide the two safety-relevant actuator intents the loop applies each
 * tick — whether the EBS is released or fired, and whether the AS SDC is opened.
 * NO hardware / RTOS dependencies, so the safety rules can be exhaustively
 * unit-tested off-target (tests/host/test_as_actuation.cpp).
 *
 * Scope: this is the STEADY per-state rule. It deliberately does NOT model the
 * OFF-state EBS init self-test (T15.2), which drives the actuators through its
 * own sequence while the init FSM is running; app_task runs that while ebs_init
 * is not yet Done/Failed and applies this steady rule otherwise (incl. OFF once
 * the init has completed). It also does not cover the watchdog safe-state
 * (safety_monitor.c), which fires the EBS + opens the SDC independently.
 */
#ifndef AS_ACTUATION_HPP
#define AS_ACTUATION_HPP

#include "state_manager.hpp"   /* ASState */

/** The two safe-actuation intents app_task applies each tick. */
struct AsActuation {
    bool ebs_release; /**< true  -> deactivateEBS() (brakes released, D1/D2 HIGH)
                           false -> activateEBS()   (brakes fired,    D1/D2 LOW) */
    bool sdc_open;    /**< true  -> open the AS SDC (hardware_io_set_as_close_sdc(false),
                           D4 LOW): Tractive System de-energised (TSAL green) in the
                           terminal safe states. false leaves the SDC as-is. */
};

/**
 * @brief Decide the safe-actuation intents from the AS state + ASMS switch.
 *        Pure; identical to the rule applied in app_task's loop.
 *
 * Rules (FS-Rules req #4 / T15.3):
 *  - EBS is RELEASED only when the car is meant to move without the autonomous
 *    brake holding it: autonomous DRIVING, or manual handling (ASMS off). It is
 *    FIRED (engaged) in every other AS state — the car is immobilised whenever
 *    it is neither autonomously driving nor under manual control.
 *  - The AS SDC is OPENED in the terminal safe states FINISHED and EMERGENCY,
 *    which de-energises the Tractive System (TSAL returns to green) and fires
 *    the EBS via the fail-safe path. (Only reachable with ASMS on.)
 */
inline AsActuation as_actuation(ASState state, bool asms_on)
{
    AsActuation a{};
    a.ebs_release = (!asms_on) || (state == ASState::DRIVING);
    a.sdc_open    = asms_on &&
                    (state == ASState::FINISHED || state == ASState::EMERGENCY);
    return a;
}

#endif /* AS_ACTUATION_HPP */
