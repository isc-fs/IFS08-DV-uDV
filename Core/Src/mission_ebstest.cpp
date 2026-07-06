/**
 * @file    mission_ebstest.cpp
 * @brief   Standalone EBS-TEST mission (AMI code 5) — currently a hold-straight
 *          stub.
 *
 * The FS-Rules EBS test drives the car to a set speed, commands the EBS trigger
 * and verifies the deceleration. That closed-loop behaviour is an on-car TODO;
 * for now this holds the wheels straight (0° every tick) so the mission is
 * selectable and the plumbing is exercised, matching the PR #80 stub.
 *
 * Standalone (no pipeline). It does NOT self-finish — is_complete is nullptr —
 * so the run ends only on an external exit (RES e-stop, ASMS off, ...). When the
 * real test lands, add on_enter (spin the drive up), on_tick (speed profile +
 * EBS trigger) and is_complete (deceleration verified).
 */
#include "mission.h"

namespace {

MissionCommand ebstest_on_tick(const MissionCtx* ctx)
{
    (void)ctx;
    MissionCommand cmd = {};
    cmd.send_steer_angle = true;
    cmd.steer_angle_deg  = 0.0f;   /* hold straight */
    /* TODO(on-car): drive to the rules test speed, trigger the EBS, verify the
     * deceleration; then give this mission a real is_complete. */
    return cmd;
}

} // namespace

/* `extern` forces external linkage (namespace-scope `const` is internal by
 * default in C++), so mission_registry.cpp's extern declaration binds here. */
extern const Mission mission_ebstest = {
    "ebs_test",          /* name           */
    false,               /* needs_pipeline */
    nullptr,             /* on_enter       */
    ebstest_on_tick,     /* on_tick        */
    nullptr,             /* is_complete    */
    nullptr,             /* on_exit        */
};
