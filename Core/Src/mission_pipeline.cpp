/**
 * @file    mission_pipeline.cpp
 * @brief   Pipeline-driven mission body shared by ACCEL / SKIDPAD / AUTOCROSS /
 *          TRACKDRIVE (AMI codes 1-4).
 *
 * These four missions differ only in the AMI menu entry; the uDV-side actuation
 * is identical — relay the DV pipeline's latest normalised /ctrl/cmd to the
 * inverter / steering ECU every tick. The inverter expects a continuous stream,
 * so we emit every tick from the latched value, and ZERO both channels when
 * /ctrl/cmd goes stale so a dropped link can never latch the last
 * throttle/steering. (Verbatim intent of the PR #80 inline pipeline branch.)
 *
 * Completion and safe-state are the FSM's job, not the mission's:
 *   - TRACKDRIVE (and the others) finish when mission_control reports
 *     dv/status FINISHED -> as_next_state converts DRIVING to FINISHED via the
 *     (needs_pipeline && dv_finished) rule. is_complete is nullptr here so the
 *     mission never double-signals completion.
 *   - A stale /dv/status or a pipeline EMERGENCY byte trips EMERGENCY in
 *     as_next_state before on_tick runs, so a dropped heartbeat is handled
 *     upstream; zeroing on cmd staleness here is the second layer.
 *
 * Accel (0x507) is CONFIRMED against the ECU .def (int32 LE percent — the
 * conversion lives in Can::sendAccel; app_task paces this TX to the ECU's 20 ms
 * torque cycle). TODO(G3, on-car): 0x508 steer has no ECU consumer on the
 * current map (steering is commanded via 0x020) — confirm who reads it or retire
 * it. The values here are the normalised [-1, 1] contract, clamped on receive.
 */
#include "mission.h"

namespace {

MissionCommand pipeline_on_tick(const MissionCtx* ctx)
{
    MissionCommand cmd = {};
    cmd.send_drive = true;   /* stream every tick (the ECU expects it) */

    if (ctx->ctrl_cmd_fresh)
    {
        cmd.accel_norm = ctx->ctrl_accel;
        cmd.steer_norm = ctx->ctrl_steer;
    }
    else
    {
        cmd.accel_norm = 0.0f;   /* stale link -> zero, never latch the last cmd */
        cmd.steer_norm = 0.0f;
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
};
