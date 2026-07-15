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
 *
 * @param dv_stopping  the pipeline commanded the end-of-mission stop
 *   (/dv/status == DV_STATUS_STOPPING, issue #176) and it applies to the run.
 *   The caller folds in freshness AND mission_needs_pipeline, the same way
 *   as_transition.hpp's dv_* inputs already fold in freshness — so a stale
 *   link or a standalone mission reads false here.
 *
 *   It fires the EBS *while staying in DRIVING with the SDC closed*, which is
 *   the one combination the AS state table has no row for. That is intentional
 *   and is NOT a new electrical capability: D1/D2 (EBS actuators) and D4 (AS
 *   SDC) are independent GPIOs, and "brakes fired + SDC closed" is already the
 *   steady state of AS READY and of post-init AS OFF. It exists only so the car
 *   can reach the standstill that AS FINISHED requires — not as a service brake
 *   (the EBS is binary; there is nothing to modulate).
 *
 *   ORDERING NOTE: ASMS-off still wins. A human pushing the car with the ASMS
 *   off must get released brakes no matter what the pipeline is saying, so
 *   `!asms_on` stays first in the OR and dv_stopping can never override it.
 *
 *   A dv_stopping that goes stale mid-stop does NOT silently release the
 *   brakes: as_next_state's dv_lost_driving rule trips EMERGENCY on the same
 *   tick (it runs before this in app_task's loop), and EMERGENCY fires the EBS
 *   anyway. So the brakes stay on through the failure.
 */
inline AsActuation as_actuation(ASState state, bool asms_on, bool dv_stopping)
{
    AsActuation a{};
    a.ebs_release = (!asms_on) ||
                    (state == ASState::DRIVING && !dv_stopping);
    a.sdc_open    = asms_on &&
                    (state == ASState::FINISHED || state == ASState::EMERGENCY);
    return a;
}

#endif /* AS_ACTUATION_HPP */
