/**
 * @file    mission_pipeline.cpp
 * @brief   Pipeline-driven mission body shared by ACCEL / SKIDPAD / AUTOCROSS /
 *          TRACKDRIVE (AMI codes 1-4).
 *
 * These four missions differ only in the AMI menu entry; the uDV-side actuation
 * is identical — relay the DV pipeline's latest normalised /ctrl/cmd:
 *   - throttle -> ECU torque on 0x507 (send_accel), streamed and paced by
 *     app_task to the ECU's 20 ms cycle; zeroed on a stale link so a dropped
 *     link never latches the last throttle.
 *   - steering -> absolute 0x020 angle (send_steer, applied as
 *     norm * STEER_FULL_LOCK_DEG in app_task). Sent ONLY while /ctrl/cmd is
 *     fresh: a 0x020 zero on a stale link would snap the wheel to center
 *     mid-corner. (PR #90 retired the consumer-less normalised 0x508 frame.)
 *
 * Completion and safe-state are the FSM's job, not the mission's:
 *   - TRACKDRIVE (and the others) finish when mission_control reports
 *     dv/status FINISHED -> as_next_state converts DRIVING to FINISHED via the
 *     (needs_pipeline && dv_finished) rule. is_complete is nullptr here so the
 *     mission never double-signals completion.
 *   - A stale /dv/status or a pipeline EMERGENCY byte trips EMERGENCY in
 *     as_next_state before on_tick runs; zeroing on cmd staleness is the second
 *     layer.
 *
 * TODO(#71, on-car): STEER_FULL_LOCK_DEG (65) + steering ratio + sign are
 * commissioning items; 0x507 accel is confirmed against the ECU .def.
 */
#include "mission.h"

namespace {

MissionCommand pipeline_on_tick(const MissionCtx* ctx)
{
    MissionCommand cmd = {};

    /* Torque (0x507) streams every tick — app_task paces it to the ECU's 20 ms
     * cycle. Zero it on a stale /ctrl/cmd so a dropped link never latches the
     * last throttle. */
    cmd.send_accel = true;
    cmd.accel_norm = ctx->ctrl_cmd_fresh ? ctx->ctrl_accel : 0.0f;

    /* Steering (0x020, applied as norm * STEER_FULL_LOCK_DEG in app_task) is
     * sent ONLY while /ctrl/cmd is fresh. On a stale link we send NO steering:
     * a 0x020 zero would snap the wheel to center mid-corner. The controller
     * holds its last target and the >400 ms staleness rule trips EMERGENCY via
     * the AS transition anyway. */
    if (ctx->ctrl_cmd_fresh)
    {
        cmd.send_steer = true;
        cmd.steer_norm = ctx->ctrl_steer;
    }
    return cmd;
}

} // namespace

/* Shared by codes 1-4. needs_pipeline=true: GO is gated on a fresh dv/status
 * READY and the lost-heartbeat rule is armed. is_complete/on_enter/on_exit are
 * nullptr — the pipeline owns the run lifecycle end-to-end.
 * `extern` forces external linkage (namespace-scope `const` is internal by
 * default in C++), so mission_registry.cpp's extern declaration binds here. */
extern const Mission mission_pipeline = {
    "pipeline",          /* name           */
    true,                /* needs_pipeline */
    nullptr,             /* on_enter       */
    pipeline_on_tick,    /* on_tick        */
    nullptr,             /* is_complete    */
    nullptr,             /* on_exit        */
    true,                /* requests_r2d — pipeline drives, needs DV mode */
};
