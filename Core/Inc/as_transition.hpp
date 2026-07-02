/**
 * @file    as_transition.hpp
 * @brief   Pure AS state-machine transition decision (host-unit-testable).
 *
 * Extracted verbatim from app_task's main loop so the safety-relevant AS
 * transition logic can be exhaustively unit-tested off-target. NO hardware /
 * RTOS / ROS dependencies — the caller reads the signals each tick and passes
 * them in; this function only decides the next AS state. Header-only `inline`
 * so it adds no translation unit and no build-manifest entry.
 *
 * NOTE: this branch (fix/19-uart-assi) does NOT run the DV pipeline handshake
 * — there is no /dv/status gate and no Finished path. The AS state is driven
 * purely by the local signals (ASMS, TS, RES). The pipeline-aware variant
 * (with the dv_* inputs, the READY->DRIVING /dv/status gate and the Finished
 * transition) lives on the pipeline-interface branch and supersedes this on
 * merge.
 */
#ifndef AS_TRANSITION_HPP
#define AS_TRANSITION_HPP

#include "state_manager.hpp"   /* ASState */

/** Signal snapshot the AS transition decision consumes (one tick). */
struct AsInputs {
    bool asms_on;    /**< ASMS master switch on (hardware io)  */
    bool ts_on;      /**< Tractive System active              */
    bool res_estop;  /**< RES emergency-stop asserted         */
    bool res_go;     /**< RES "go" asserted                   */
    bool res_ok;     /**< RES link healthy (received; ok OR go, no e-stop) */
};

/**
 * @brief Decide the next AS state from the previous state + this tick's
 *        signals. Pure; identical to the chain in app_task's loop.
 *
 * Fail-safe ordering: ASMS-off and a latched Emergency win first; then any
 * emergency trigger (RES e-stop, or TS loss while armed); then the Driving /
 * Ready progression. Returns `prev` unchanged when nothing matches, so the
 * state only moves on an explicit trigger.
 *
 * DRIVING is sticky: the RES "go" is a momentary trigger (R2D press), not a
 * level, so once Driving the run continues until an explicit exit (RES
 * e-stop, TS loss, or ASMS off). Without this, a held "go" oscillates
 * READY<->DRIVING every tick (res_ok being true for "go" too), and a released
 * "go" would drop Driving back to Ready mid-run.
 */
inline ASState as_next_state(ASState prev, const AsInputs& in)
{
    if (!in.asms_on)
        return ASState::OFF;                       /* ASMS off always wins   */
    if (prev == ASState::EMERGENCY)
        return ASState::EMERGENCY;                 /* latches until ASMS off */
    if (in.res_estop
        || (!in.ts_on && (prev == ASState::DRIVING || prev == ASState::READY)))
        return ASState::EMERGENCY;
    if (prev == ASState::DRIVING)
        return ASState::DRIVING;                   /* sticky — see above     */
    if (in.res_go && prev == ASState::READY)
        return ASState::DRIVING;
    if (in.res_ok && in.ts_on)
        return ASState::READY;
    return prev;                                    /* no change              */
}

#endif /* AS_TRANSITION_HPP */
