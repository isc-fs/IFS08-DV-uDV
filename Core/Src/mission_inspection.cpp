/**
 * @file    mission_inspection.cpp
 * @brief   Standalone INSPECTION mission (AMI code 6) — open-loop steering +
 *          drivetrain (torque) test. Requests DV R2D and, once the ECU confirms
 *          (0x511), commands a steady 15% torque — so it exercises the full
 *          uDV->ECU->inverter chain (wheels-up/bench: real torque flows).
 *
 * Proves the steering chain end-to-end with no pipeline: sweep the steering
 * ±INSPECTION_STEER_AMP_DEG at INSPECTION_STEER_FREQ_HZ, one 0x020 angle
 * command per INSPECTION_STEER_DT_MS, then self-finish after
 * INSPECTION_DURATION_MS (raising is_complete -> DRIVING to FINISHED via
 * as_next_state). The steering motor is started (0x010) by app_task on the GO
 * edge; the ±90° amplitude intentionally exceeds the steering firmware's ±60°
 * clamp and saturates — matching the PR #80 behaviour this lifts verbatim.
 *
 * Pure module: no CAN/RTOS/HAL includes. on_tick returns the actuation; the
 * only state is the rate-limit stamp, reset in on_enter and so trivially
 * host-testable (see tests/host/test_missions.cpp).
 *
 * NOTE: the steering board blocks ~1 s + up to ~4 s homing (Arranque) after
 * motor start, so the actuator ignores roughly the first ~5 s of the sweep.
 * That is expected for a 30 s demo — do not "fix" it here.
 */
#include "mission.h"

#include <cmath>

namespace {

constexpr float    TWO_PI                   = 6.283185307f;
constexpr float    INSPECTION_STEER_AMP_DEG = 90.0f;   /* sweep amplitude (deg) */
constexpr float    INSPECTION_STEER_FREQ_HZ = 0.3f;    /* sweep frequency (Hz)  */
constexpr uint32_t INSPECTION_STEER_DT_MS   = 200u;    /* one command per 200 ms */
constexpr uint32_t INSPECTION_DURATION_MS   = 30000u;  /* 30 s, then FINISHED   */
constexpr float    INSPECTION_TORQUE_NORM   = 0.15f;   /* 15% torque once DV R2D confirmed */

/* Open-loop sweep angle at a given mission-elapsed time. Phase derives from the
 * elapsed time so each run starts clean at 0°. */
float inspection_sweep_deg(uint32_t elapsed_ms)
{
    float t = (float)elapsed_ms / 1000.0f;
    return INSPECTION_STEER_AMP_DEG * std::sin(TWO_PI * INSPECTION_STEER_FREQ_HZ * t);
}

/* Rate-limit state for the 200 ms command cadence. s_first_emit forces the
 * FIRST tick of each run to command unconditionally (on_enter re-arms it), so
 * the opening steering command never depends on the absolute value of now_ms.
 * (Relying on "now_ms >> DT" would drop the first command if a run ever starts
 * with a small tick count or just after the ~49.7-day tick-counter wrap.)
 * Single mission instance on the target; both are reset every run via on_enter. */
bool     s_first_emit   = true;
uint32_t s_last_emit_ms = 0u;

void inspection_on_enter(const MissionCtx* ctx)
{
    (void)ctx;
    s_first_emit   = true;   /* first DRIVING tick of the run emits immediately */
    s_last_emit_ms = 0u;
}

MissionCommand inspection_on_tick(const MissionCtx* ctx)
{
    MissionCommand cmd = {};   /* default: emit nothing this tick */

    /* Command on the first tick of the run, then once per INSPECTION_STEER_DT_MS.
     * Unsigned tick subtraction is wrap-safe. */
    if (s_first_emit ||
        (uint32_t)(ctx->now_ms - s_last_emit_ms) >= INSPECTION_STEER_DT_MS)
    {
        cmd.send_steer_angle = true;
        cmd.steer_angle_deg  = inspection_sweep_deg(ctx->mission_elapsed_ms);
        s_last_emit_ms       = ctx->now_ms;
        s_first_emit         = false;
    }

    /* Torque test: once the ECU confirms DV R2D (0x511), command a steady 15%.
     * Before confirm this stays 0, and app_task keeps the 0x507 stream alive at
     * 0 — so torque only flows after DV mode is actually latched. app_task paces
     * the 0x507 TX to the ECU's 20 ms cycle. */
    if (ctx->r2d_confirmed)
    {
        cmd.send_accel = true;
        cmd.accel_norm = INSPECTION_TORQUE_NORM;
    }
    return cmd;
}

bool inspection_is_complete(const MissionCtx* ctx)
{
    return ctx->mission_elapsed_ms >= INSPECTION_DURATION_MS;
}

} // namespace

/* Standalone: no pipeline gate, self-finishes on its own timer. on_exit is
 * nullptr — app_task stops the steering motor on the ->FINISHED edge.
 * `extern` forces external linkage (a namespace-scope `const` is internal by
 * default in C++), so mission_registry.cpp's extern declaration binds here. */
extern const Mission mission_inspection = {
    "inspection",            /* name           */
    false,                   /* needs_pipeline */
    inspection_on_enter,     /* on_enter       */
    inspection_on_tick,      /* on_tick        */
    inspection_is_complete,  /* is_complete    */
    nullptr,                 /* on_exit        */
    true,                    /* requests_r2d — exercise the ECU DV R2D handshake */
};
