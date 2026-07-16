/**
 * @file    as_transition.hpp
 * @brief   Pure AS state-machine transition decision (host-unit-testable).
 *
 * Extracted verbatim from app_task's main loop so the safety-relevant AS
 * transition logic can be exhaustively unit-tested off-target, the same way
 * the DV pipeline keeps its decision core in the pure, unit-tested
 * `mission_control/reconcile.py`. NO hardware / RTOS / ROS dependencies —
 * the caller reads the signals each tick and passes them in; this function
 * only decides the next AS state. Header-only `inline` so it adds no
 * translation unit and no build-manifest entry.
 *
 * The uDV owns this state machine; the DV pipeline only reacts to the byte
 * it publishes. The `dv_*` inputs summarise the level-triggered read of
 * /dv/status (see dv_interface.h): each already folds in freshness, so e.g.
 * `dv_ready` is false when /dv/status is stale. `dv_fresh` alone lets this
 * function derive the "pipeline heartbeat lost while driving" rule.
 */
#ifndef AS_TRANSITION_HPP
#define AS_TRANSITION_HPP

#include "state_manager.hpp"   /* ASState */

/** Signal snapshot the AS transition decision consumes (one tick). */
struct AsInputs {
    bool asms_on;       /**< ASMS master switch on (hardware io)            */
    bool ts_on;         /**< Tractive System active                        */
    bool res_estop;     /**< RES emergency-stop asserted                   */
    bool res_go;        /**< RES "go" asserted                             */
    bool res_ok;        /**< RES received, no e-stop / go                  */
    bool steer_emergency; /**< steering reported ESTADO_MOTOR_EMERGENCIA (grave) */
    bool ebs_init_done; /**< EBS init sequence reached EBSInitState::Done   */
    bool asb_pressure_ok; /**< BOTH ASB air tanks above ACTUATOR_STORAGE_THRESHOLD
                               (3 bar) — i.e. the EBS still has the stored energy to
                               actually stop the car. False when EITHER sensor (A5
                               actuator-1 / A4 actuator-2) reads below it.
                               Debounced by the caller (see app_task): a single noisy
                               ADC sample must not latch Emergency and kill a run. */
    bool ready_dwell_elapsed; /**< the car has been in AS READY for the mandated
                                 minimum dwell (>= 5 s, FS-Rules) — a GO received
                                 before it elapses must be refused. Sourced in
                                 app_task from the READY-entry timestamp; gates
                                 READY->DRIVING alongside res_go. */
    bool dv_fresh;      /**< /dv/status seen within DV_STATUS_STALE_MS      */
    bool dv_ready;      /**< fresh && /dv/status == DV_STATUS_READY         */
    bool dv_finished;   /**< fresh && /dv/status == DV_STATUS_FINISHED      */
    bool dv_emergency;  /**< fresh && /dv/status == EMERGENCY or FAILED     */
    bool mission_needs_pipeline; /**< selected mission is driven by the DV pipeline
                                    (accel / skidpad / autocross / trackdrive). When
                                    false the mission is a standalone open-loop one
                                    (inspection, EBS test): no /dv/status gate on GO,
                                    and a stale pipeline never trips it. */
    bool mission_complete; /**< a standalone mission self-reported done (e.g. the
                                inspection sweep timer elapsed) -> DRIVING to FINISHED */
    bool vehicle_standstill; /**< the car is stopped (|rpm| < threshold, ECU 0x506).
                                Lets a genuine end-of-mission FINISH win over the ASB
                                low-pressure trip: the braking that ended the mission
                                may itself have drawn the tanks below 3 bar, and a
                                completed mission that stopped the car should report
                                Finished, not Emergency. */
    bool mission_valid;    /**< the selected AMI code maps to a real mission
                                (mission_for_code != nullptr). GO is refused when
                                false, so an unknown / SHUTDOWN code can never enter
                                DRIVING and release the brakes with no mission body.
                                (Set true for every defined mission, so the normal
                                arming path is unchanged.) */
};

/**
 * @brief Decide the next AS state from the previous state + this tick's
 *        signals. Pure; identical to the chain in app_task's loop.
 *
 * Fail-safe ordering: ASMS-off and a latched Emergency/Finished win first;
 * then any emergency trigger (RES e-stop, a steering grave-fault, TS loss
 * while armed, a pipeline-raised emergency, or a lost pipeline heartbeat
 * mid-run); then the normal Finished / Driving / Ready progression. Returns
 * `prev` unchanged when nothing matches (e.g. steady Driving with the
 * pipeline Running), so the state only moves on an explicit trigger.
 */
inline ASState as_next_state(ASState prev, const AsInputs& in)
{
    /* Pipeline heartbeat lost while driving (link / DVPC dead) — only for a
     * mission that actually depends on the pipeline. A standalone open-loop
     * mission (inspection / EBS test) has no heartbeat to lose. */
    const bool dv_lost_driving =
        (prev == ASState::DRIVING) && in.mission_needs_pipeline && !in.dv_fresh;

    /* A genuine end-of-mission finish, AT STANDSTILL: the mission is over
     * (standalone self-finish or pipeline FINISHED) and the car is stopped.
     * At that point FINISHED wins over the ASB low-pressure trip below — the
     * braking that ended the mission may itself have drawn the tanks under 3
     * bar, and a completed mission that stopped the car should report Finished,
     * not Emergency. This only changes the REPORTED terminal state, never the
     * actuation: as_actuation fires the EBS + opens the SDC in BOTH FINISHED
     * and EMERGENCY, and the car is already stopped. It yields ONLY the ASB
     * trip — a real emergency (e-stop, steering grave-fault, TS loss, pipeline
     * emergency / lost heartbeat) still forces Emergency even here. */
    const bool finishing_at_standstill =
        (prev == ASState::DRIVING) && in.vehicle_standstill &&
        (in.mission_complete ||
         (in.mission_needs_pipeline && in.dv_finished));

    if (!in.asms_on)
        return ASState::OFF;                       /* ASMS off always wins  */
    if (prev == ASState::EMERGENCY)
        return ASState::EMERGENCY;                 /* latches until ASMS off */
    if (prev == ASState::FINISHED)
        return ASState::FINISHED;                  /* latches until ASMS off */
    if (in.res_estop
        || in.steer_emergency  /* steering grave-fault: unconditional, like e-stop */
        || (!in.ts_on && (prev == ASState::DRIVING || prev == ASState::READY))
        /* ASB stored energy gone while ARMED: either air tank below 3 bar means
         * the EBS may no longer be able to stop the car, so the car must not
         * keep driving on a brake that might not fire. Same shape and scope as
         * the ts_on rule — deliberately ONLY while READY/DRIVING:
         *
         *   - In OFF the tanks are legitimately empty/filling at power-on, and
         *     Emergency LATCHES until ASMS-off. Tripping there would mean a car
         *     that boots with flat tanks could never arm even after filling them.
         *     OFF is instead covered by the READY gate below (no pressure, no arm)
         *     and by the EBS init self-check (CheckPressure -> Failed).
         *   - EMERGENCY/FINISHED already returned above (they latch), and both
         *     hold the EBS fired anyway.
         *
         * Note this is the FIRST live consumer of tank pressure: ASBChecksOK()
         * gated only AS Ready, and abs_checks_ok was never an AsInputs field, so
         * until now a tank could drain mid-run with nothing noticing. */
        || (!in.asb_pressure_ok && !finishing_at_standstill
                                && (prev == ASState::DRIVING || prev == ASState::READY))
        || (in.dv_emergency && in.mission_needs_pipeline
                            && (prev == ASState::DRIVING || prev == ASState::READY))
        || dv_lost_driving)
        return ASState::EMERGENCY;
    /* DRIVING -> FINISHED: a pipeline mission ends when mission_control reports
     * FINISHED; a standalone mission ends when its own open-loop logic raises
     * mission_complete (e.g. the inspection sweep timer elapsed). */
    if (prev == ASState::DRIVING
        && (in.mission_complete
            || (in.mission_needs_pipeline && in.dv_finished)))
        return ASState::FINISHED;
    if (prev == ASState::DRIVING)
        return ASState::DRIVING;                   /* DRIVING is sticky: GO is a
                                                      trigger, not a level — only
                                                      the exits above end a run
                                                      (estop / TS loss / DV emerg /
                                                      DV lost / DV finished /
                                                      mission_complete / ASMS off).
                                                      Without this, a held GO
                                                      oscillates READY<->DRIVING and
                                                      a released GO tears the
                                                      mission down mid-drive. */
    if (in.res_go && prev == ASState::READY && in.mission_valid
        && (in.dv_ready || !in.mission_needs_pipeline)
        && in.ready_dwell_elapsed)
        return ASState::DRIVING;                   /* pipeline mission: go honoured
                                                      only if DV READY; standalone:
                                                      go alone (no pipeline). Both
                                                      require a valid mission: an
                                                      unknown / SHUTDOWN code never
                                                      drives (would release brakes
                                                      with no mission body). The
                                                      ready_dwell_elapsed gate holds
                                                      GO until the mandated >=5 s AS
                                                      READY dwell has passed (FS-Rules)
                                                      — a GO before then is refused. */
    /* READY is gated on the EBS init sequence being complete (dev 9395936):
     * app_task only advances the init FSM while in OFF, so arming before
     * Done would strand it (ASBChecksOK never true). A Failed init keeps
     * the car in OFF. */
    /* asb_pressure_ok also gates ARMING, not just the mid-run trip. Without it
     * a car whose tanks drained AFTER a passing init self-check could still
     * reach READY: ebs_init_done is a one-shot latch on the init FSM, and the
     * live pressure read (ASBChecksOK) reached only StateManager::updateState(),
     * which is dead for AS decisions. The gate is what makes "no stored energy,
     * no arming" true in the state machine rather than just in the init. */
    if (in.res_ok && in.ts_on && in.ebs_init_done && in.asb_pressure_ok)
        return ASState::READY;
    return prev;                                    /* no change */
}

#endif /* AS_TRANSITION_HPP */
