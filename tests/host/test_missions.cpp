/**
 * @file test_missions.cpp
 * @brief Host unit tests for the mission dispatch layer (Core/Inc/mission.h +
 *        Core/Src/mission_registry.cpp + mission_inspection/pipeline/ebstest.cpp).
 *
 * Missions are PURE (like as_transition.hpp): a body consumes a MissionCtx and
 * returns a MissionCommand, touching no CAN/RTOS/HAL. So this suite links the
 * real mission sources directly — no stubs — and exhaustively checks:
 *   1. the registry mapping (every AMI code 0..9 + out-of-range),
 *   2. the INSPECTION control law (rate-limit cadence, sweep math, self-finish),
 *   3. the shared PIPELINE relay (pass-through when fresh, zero when stale),
 *   4. the EBS-test stub, MANUAL no-op,
 *   5. cross-cutting invariants (classification, single actuation channel).
 *
 * Build + run:  make test_missions
 */
#include <cstdio>
#include <cmath>
#include <cstring>

#include "mission.h"

/* Mission singletons under test (defined in their own TUs). */
extern const Mission mission_inspection;
extern const Mission mission_pipeline;
extern const Mission mission_ebstest;

/* ---- INSPECTION contract constants (MUST match mission_inspection.cpp) ---- */
static const float    INSP_AMP_DEG  = 90.0f;
static const float    INSP_FREQ_HZ  = 0.3f;
static const uint32_t INSP_DT_MS    = 200u;
static const uint32_t INSP_DUR_MS   = 30000u;
static const float    TWO_PI        = 6.283185307f;

static int g_checks = 0;
static int g_fail   = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        ++g_checks;                                                            \
        if (!(cond)) { ++g_fail;                                               \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); }      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        ++g_checks;                                                            \
        float _a = (float)(a), _b = (float)(b), _t = (float)(tol);            \
        if (std::fabs(_a - _b) > _t) { ++g_fail;                               \
            std::printf("FAIL %s:%d  |%f - %f| > %f\n", __FILE__, __LINE__,    \
                        _a, _b, _t); }                                         \
    } while (0)

static MissionCtx ctx_make(uint32_t now_ms, uint32_t elapsed_ms,
                           bool fresh, float accel, float steer)
{
    MissionCtx c;
    c.now_ms             = now_ms;
    c.mission_elapsed_ms = elapsed_ms;
    c.ctrl_cmd_fresh     = fresh;
    c.ctrl_accel         = accel;
    c.ctrl_steer         = steer;
    return c;
}

/* Named-local wrappers — the vtable takes a MissionCtx*, and you can't take the
 * address of a temporary, so build a local ctx and pass it. */
static MissionCommand tick(const Mission* m, uint32_t now, uint32_t el,
                           bool fresh, float accel, float steer)
{
    MissionCtx c = ctx_make(now, el, fresh, accel, steer);
    return m->on_tick(&c);
}

static void enter(const Mission* m, uint32_t now)
{
    MissionCtx c = ctx_make(now, 0, false, 0, 0);
    m->on_enter(&c);
}

static bool is_done(const Mission* m, uint32_t elapsed)
{
    MissionCtx c = ctx_make(0, elapsed, false, 0, 0);
    return m->is_complete(&c);
}

static float expected_sweep_deg(uint32_t elapsed_ms)
{
    float t = (float)elapsed_ms / 1000.0f;
    return INSP_AMP_DEG * std::sin(TWO_PI * INSP_FREQ_HZ * t);
}

/* ==== 1. registry mapping =============================================== */
static void test_registry_mapping(void)
{
    // MANUAL (0): real, standalone, no actuation body.
    const Mission* m0 = mission_for_code(MISSION_MANUAL);
    CHECK(m0 != nullptr);
    CHECK(m0 != nullptr && !m0->needs_pipeline);
    CHECK(m0 != nullptr && std::strcmp(m0->name, "manual") == 0);
    CHECK(m0 != nullptr && m0->on_tick == nullptr);   // genuine no-op

    // Pipeline missions 1..4 all resolve to the SAME shared body.
    const Mission* accel     = mission_for_code(MISSION_ACCEL);
    const Mission* skidpad   = mission_for_code(MISSION_SKIDPAD);
    const Mission* autocross = mission_for_code(MISSION_AUTOCROSS);
    const Mission* track     = mission_for_code(MISSION_TRACKDRIVE);
    CHECK(accel == &mission_pipeline);
    CHECK(skidpad == &mission_pipeline);
    CHECK(autocross == &mission_pipeline);
    CHECK(track == &mission_pipeline);
    CHECK(track != nullptr && track->needs_pipeline);
    CHECK(track != nullptr && track->is_complete == nullptr);  // finishes via dv/status
    CHECK(track != nullptr && track->on_tick != nullptr);

    // EBS test (5): standalone stub.
    const Mission* ebs = mission_for_code(MISSION_EBS_TEST);
    CHECK(ebs == &mission_ebstest);
    CHECK(ebs != nullptr && !ebs->needs_pipeline);
    CHECK(ebs != nullptr && std::strcmp(ebs->name, "ebs_test") == 0);

    // Inspection (6): standalone, full lifecycle except on_exit.
    const Mission* insp = mission_for_code(MISSION_INSPECTION);
    CHECK(insp == &mission_inspection);
    CHECK(insp != nullptr && !insp->needs_pipeline);
    CHECK(insp != nullptr && insp->on_enter != nullptr);
    CHECK(insp != nullptr && insp->on_tick != nullptr);
    CHECK(insp != nullptr && insp->is_complete != nullptr);
    CHECK(insp != nullptr && insp->on_exit == nullptr);

    // SHUTDOWN (7) + aux (8, 9): no uDV mission.
    CHECK(mission_for_code(MISSION_SHUTDOWN) == nullptr);
    CHECK(mission_for_code(8) == nullptr);
    CHECK(mission_for_code(9) == nullptr);

    // Out of range (negative, >=10, far) -> nullptr, never a crash.
    CHECK(mission_for_code(-1) == nullptr);
    CHECK(mission_for_code(-42) == nullptr);
    CHECK(mission_for_code(10) == nullptr);
    CHECK(mission_for_code(255) == nullptr);
    CHECK(mission_for_code(1000000) == nullptr);
}

/* ==== 2. inspection control law ========================================= */
static void test_inspection_cadence(void)
{
    enter(&mission_inspection, 0);   // reset rate-limit stamp to 0

    // First DRIVING tick emits immediately (now >> DT).
    MissionCommand a = tick(&mission_inspection, 1000, 0, false, 0, 0);
    CHECK(a.send_steer_angle);
    CHECK(!a.send_accel);            // inspection never uses the drive channels
    CHECK(!a.send_steer);

    // +100 ms (< DT): no new command.
    MissionCommand b = tick(&mission_inspection, 1100, 100, false, 0, 0);
    CHECK(!b.send_steer_angle);

    // +200 ms from the last emit: commands again.
    MissionCommand cc = tick(&mission_inspection, 1200, 200, false, 0, 0);
    CHECK(cc.send_steer_angle);

    // Just under DT from that emit: no command.
    MissionCommand d = tick(&mission_inspection, 1399, 399, false, 0, 0);
    CHECK(!d.send_steer_angle);

    // Exactly DT: command.
    MissionCommand e = tick(&mission_inspection, 1400, 400, false, 0, 0);
    CHECK(e.send_steer_angle);
}

static void test_inspection_first_emit_guarantee(void)
{
    // The FIRST tick of every run MUST command, regardless of now_ms — even when
    // now_ms < DT (a GO shortly after boot, or after the ~49.7-day tick-counter
    // wrap). This is the s_first_emit guarantee; it must not rely on now_ms being
    // large. Masked in normal operation by the 5 s READY dwell, so a plain
    // "first tick emits" check at now=1000+ can't see a regression here.
    //
    // Every tick below uses now_ms < DT so that ONLY the re-armed first-emit
    // (not the now_ms magnitude) can produce a command — which makes this test
    // actually exercise on_enter's reset: a no-op on_enter leaves the tick gated.

    // (a) Boot-time GO at a tiny now: the first tick still commands.
    enter(&mission_inspection, 0);
    CHECK(tick(&mission_inspection, 10, 0, false, 0, 0).send_steer_angle);   // now < DT

    // (b) GO at exactly now == 0.
    enter(&mission_inspection, 0);
    CHECK(tick(&mission_inspection, 0, 0, false, 0, 0).send_steer_angle);

    // (c) Re-arm across runs. Prime a LOW emit stamp, confirm steady-state
    // gating still suppresses a sub-DT tick (proves the stamp is live), then a
    // fresh on_enter must re-arm first-emit so the next sub-DT tick commands
    // anyway. A no-op on_enter leaves it gated -> this CHECK fails.
    enter(&mission_inspection, 0);
    CHECK(tick(&mission_inspection, 100, 0, false, 0, 0).send_steer_angle);   // emit, stamp=100
    CHECK(!tick(&mission_inspection, 150, 50, false, 0, 0).send_steer_angle); // +50 < DT: gated
    enter(&mission_inspection, 150);                                          // re-arm
    CHECK(tick(&mission_inspection, 160, 60, false, 0, 0).send_steer_angle);  // +60 < DT: emits
}

static void test_inspection_sweep_math(void)
{
    // Force every tick to emit by advancing now_ms by >= DT each call; vary the
    // mission-elapsed time to sample the sweep across a full period.
    enter(&mission_inspection, 0);

    const uint32_t elapsed_pts[] = {0, 833, 1667, 2500, 3333, 5000, 10000};
    uint32_t now = 100000;
    for (unsigned i = 0; i < sizeof(elapsed_pts)/sizeof(elapsed_pts[0]); ++i) {
        now += INSP_DT_MS + 5;   // guarantee emission
        MissionCommand cmd = tick(&mission_inspection, now, elapsed_pts[i], false, 0, 0);
        CHECK(cmd.send_steer_angle);
        CHECK_NEAR(cmd.steer_angle_deg, expected_sweep_deg(elapsed_pts[i]), 0.5f);
    }

    // Start of sweep is exactly 0 deg.
    now += INSP_DT_MS + 5;
    MissionCommand z = tick(&mission_inspection, now, 0, false, 0, 0);
    CHECK_NEAR(z.steer_angle_deg, 0.0f, 0.001f);

    // Quarter period (t = 1/(4*freq) ~ 0.833 s) is the +amplitude peak.
    uint32_t quarter = (uint32_t)(1000.0f / (4.0f * INSP_FREQ_HZ));  // ~833 ms
    now += INSP_DT_MS + 5;
    MissionCommand pk = tick(&mission_inspection, now, quarter, false, 0, 0);
    CHECK_NEAR(pk.steer_angle_deg, INSP_AMP_DEG, 0.5f);

    // Amplitude bound: |sweep| never exceeds the configured amplitude.
    for (uint32_t el = 0; el <= INSP_DUR_MS; el += 250) {
        now += INSP_DT_MS + 5;
        MissionCommand cmd = tick(&mission_inspection, now, el, false, 0, 0);
        CHECK(std::fabs(cmd.steer_angle_deg) <= INSP_AMP_DEG + 0.01f);
    }
}

static void test_inspection_self_finish(void)
{
    // is_complete flips exactly at INSP_DUR_MS.
    CHECK(!is_done(&mission_inspection, INSP_DUR_MS - 1));
    CHECK(is_done(&mission_inspection, INSP_DUR_MS));
    CHECK(is_done(&mission_inspection, INSP_DUR_MS + 5000));
    // Fresh run: not complete at the start.
    CHECK(!is_done(&mission_inspection, 0));
}

static void test_inspection_full_run(void)
{
    // Drive the mission the way app_task will: on_enter on the GO edge, then
    // one on_tick + is_complete per ~1 ms DRIVING tick, stopping the tick the
    // mission self-finishes (app_task leaves DRIVING the next tick). Asserts the
    // whole-run contract: ~5 Hz command cadence and self-finish exactly at 30 s.
    enter(&mission_inspection, 0);
    const uint32_t base = 1000000u;   // arbitrary large kernel tick base
    int      emits = 0;
    bool     completed = false;
    uint32_t complete_at = 0;
    float    last_angle = 0.0f;

    for (uint32_t e = 0; e <= 31000u; ++e) {
        uint32_t now = base + e;
        MissionCommand cmd = tick(&mission_inspection, now, e, false, 0, 0);
        if (cmd.send_steer_angle) {
            ++emits;
            last_angle = cmd.steer_angle_deg;
            CHECK(std::fabs(cmd.steer_angle_deg) <= INSP_AMP_DEG + 0.01f);
        }
        if (is_done(&mission_inspection, e)) { completed = true; complete_at = e; break; }
    }

    // One command per 200 ms across the 30 s run (0, 200, ..., 30000) = 151.
    CHECK(emits >= 150 && emits <= 152);
    // Self-finish fires exactly at the configured duration, not before/after.
    CHECK(completed);
    CHECK(complete_at == INSP_DUR_MS);
    // The final commanded angle equals the sweep at 30 s (continuity sanity).
    CHECK_NEAR(last_angle, expected_sweep_deg(INSP_DUR_MS), 0.5f);
}

/* ==== 3. pipeline relay ================================================== */
static void test_pipeline_passthrough(void)
{
    // Fresh /ctrl/cmd -> torque (send_accel) + normalised steering (send_steer);
    // the norm->deg conversion happens in app_task, so the mission passes the
    // normalised steer through untouched. It never uses the direct-angle channel.
    MissionCommand a = tick(&mission_pipeline, 1000, 500, true, 0.5f, -0.3f);
    CHECK(a.send_accel);
    CHECK(a.send_steer);
    CHECK(!a.send_steer_angle);          // pipeline uses send_steer, not the sweep channel
    CHECK_NEAR(a.accel_norm, 0.5f, 1e-6f);
    CHECK_NEAR(a.steer_norm, -0.3f, 1e-6f);

    // Full range passes through unclamped by the mission (clamp is downstream).
    MissionCommand hi = tick(&mission_pipeline, 1000, 500, true, 1.0f, 1.0f);
    CHECK_NEAR(hi.accel_norm, 1.0f, 1e-6f);
    CHECK_NEAR(hi.steer_norm, 1.0f, 1e-6f);
    MissionCommand lo = tick(&mission_pipeline, 1000, 500, true, -1.0f, -1.0f);
    CHECK_NEAR(lo.accel_norm, -1.0f, 1e-6f);
    CHECK_NEAR(lo.steer_norm, -1.0f, 1e-6f);
}

static void test_pipeline_zeros_when_stale(void)
{
    // Stale /ctrl/cmd -> torque still streams but ZEROED (send_accel, accel 0),
    // and NO steering (send_steer false) — a 0x020 zero would center the wheel.
    MissionCommand s = tick(&mission_pipeline, 1000, 500, false, 0.9f, -0.7f);
    CHECK(s.send_accel);
    CHECK_NEAR(s.accel_norm, 0.0f, 1e-6f);
    CHECK(!s.send_steer);
    CHECK(!s.send_steer_angle);
}

/* ==== 4. ebs-test stub + manual no-op =================================== */
static void test_ebstest_holds_straight(void)
{
    // Regardless of ctx, the stub commands 0 deg (hold straight), no drive.
    MissionCommand a = tick(&mission_ebstest, 1000, 100, true, 0.5f, 0.5f);
    CHECK(a.send_steer_angle);
    CHECK_NEAR(a.steer_angle_deg, 0.0f, 1e-6f);
    CHECK(!a.send_accel);
    CHECK(!a.send_steer);
    MissionCommand b = tick(&mission_ebstest, 9999, 20000, false, -1.0f, 1.0f);
    CHECK(b.send_steer_angle);
    CHECK_NEAR(b.steer_angle_deg, 0.0f, 1e-6f);
}

static void test_manual_is_noop(void)
{
    const Mission* m = mission_for_code(MISSION_MANUAL);
    CHECK(m != nullptr);
    // A no-op mission carries no actuation and never self-finishes.
    CHECK(m != nullptr && m->on_tick == nullptr);
    CHECK(m != nullptr && m->is_complete == nullptr);
    CHECK(m != nullptr && m->on_enter == nullptr);
    CHECK(m != nullptr && m->on_exit == nullptr);
    CHECK(m != nullptr && !m->needs_pipeline);
}

/* ==== 5. cross-cutting invariants ======================================= */
static void test_classification_invariants(void)
{
    // Exactly the four pipeline missions (codes 1..4) need the pipeline; every
    // other defined mission is standalone.
    for (int code = 0; code <= 9; ++code) {
        const Mission* m = mission_for_code(code);
        bool should_need_pipeline = (code >= MISSION_ACCEL && code <= MISSION_TRACKDRIVE);
        if (m == nullptr) {
            // Only 7/8/9 (and out of range) are unmapped.
            CHECK(code >= MISSION_SHUTDOWN);
        } else {
            CHECK(m->needs_pipeline == should_need_pipeline);
            CHECK(m->name != nullptr && m->name[0] != '\0');
        }
    }
}

static void test_single_actuation_channel(void)
{
    // A mission drives EITHER the direct-angle channel (send_steer_angle,
    // inspection/ebs) OR the pipeline channels (send_accel/send_steer), never
    // both — they are two different actuation styles on the same wheel/torque.
    // Sweep a range of contexts across every mission that has an on_tick body.
    const Mission* bodies[] = {&mission_inspection, &mission_pipeline, &mission_ebstest};
    for (unsigned b = 0; b < sizeof(bodies)/sizeof(bodies[0]); ++b) {
        const Mission* m = bodies[b];
        if (m->on_enter) { enter(m, 0); }
        uint32_t now = 5000;
        for (uint32_t el = 0; el <= 31000; el += 500) {
            now += INSP_DT_MS + 1;   // keep inspection emitting
            for (int fresh = 0; fresh < 2; ++fresh) {
                MissionCommand cmd = tick(m, now, el, fresh != 0, 0.4f, -0.4f);
                CHECK(!(cmd.send_steer_angle && (cmd.send_accel || cmd.send_steer)));
            }
        }
    }
}

int main(void)
{
    test_registry_mapping();
    test_inspection_cadence();
    test_inspection_first_emit_guarantee();
    test_inspection_sweep_math();
    test_inspection_self_finish();
    test_inspection_full_run();
    test_pipeline_passthrough();
    test_pipeline_zeros_when_stale();
    test_ebstest_holds_straight();
    test_manual_is_noop();
    test_classification_invariants();
    test_single_actuation_channel();

    std::printf("\nmissions: %d checks, %d failures\n", g_checks, g_fail);
    if (g_fail == 0) {
        std::printf("All tests green.\n");
        return 0;
    }
    return 1;
}
