/**
 * @file test_as_transition.cpp
 * @brief Host unit tests for the pure AS transition decision
 *        (Core/Inc/as_transition.hpp) added for the stock-typed DV-pipeline
 *        interface.
 *
 * This is the safety-relevant logic that changed when the uDV moved to the
 * topic-based pipeline contract: the /dv/status handshake gate on
 * Ready->Driving, the FINISHED / EMERGENCY reactions, and the "pipeline
 * heartbeat lost while driving" watchdog. The IWDG task-stall watchdog is
 * unchanged and covered by test_safety_eval; this suite guards the AS-state
 * logic that feeds it.
 *
 * Two layers:
 *   1. Named cases — readable truth-table spot checks.
 *   2. An exhaustive sweep over the *realizable* input space asserting the
 *      fail-safe invariants (ASMS-off wins, Driving only via the full gate,
 *      Emergency/Finished latch, e-stop and lost-heartbeat force Emergency).
 *
 * Build + run:  make test_as_transition
 */
#include <cstdio>
#include <initializer_list>

#include "as_transition.hpp"   // AsInputs, as_next_state, ASState

static int g_checks = 0;
static int g_failures = 0;

static const char *name(ASState s)
{
    switch (s) {
        case ASState::OFF:       return "OFF";
        case ASState::READY:     return "READY";
        case ASState::DRIVING:   return "DRIVING";
        case ASState::EMERGENCY: return "EMERGENCY";
        case ASState::FINISHED:  return "FINISHED";
    }
    return "?";
}

#define CHECK_EQ(prev, in, expect)                                            \
    do {                                                                      \
        ++g_checks;                                                           \
        ASState got = as_next_state((prev), (in));                            \
        if (got != (expect)) {                                                \
            ++g_failures;                                                     \
            std::printf("FAIL %s:%d  prev=%s -> got %s, want %s\n",           \
                        __FILE__, __LINE__, name(prev), name(got),            \
                        name(expect));                                        \
        }                                                                     \
    } while (0)

#define CHECK_TRUE(cond)                                                      \
    do {                                                                      \
        ++g_checks;                                                           \
        if (!(cond)) {                                                        \
            ++g_failures;                                                     \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
        }                                                                     \
    } while (0)

// ---- input builders (only realizable combinations) ------------------
// RES is a single status: exactly one of estop/go/ok (or none of them).
enum ResKind { RES_NONE, RES_OK, RES_GO, RES_ESTOP };
// /dv/status folded to what the AS logic consumes.
enum DvKind  { DV_STALE, DV_FRESH_OTHER, DV_FRESH_READY, DV_FRESH_FINISHED, DV_FRESH_EMERG };

static AsInputs make(bool asms, bool ts, ResKind res, DvKind dv)
{
    AsInputs in{};
    in.asms_on      = asms;
    in.ts_on        = ts;
    in.res_estop    = (res == RES_ESTOP);
    in.res_go       = (res == RES_GO);
    /* Mirrors the producer in app_task.cpp: "go" also means the RES link is
     * healthy (res_ok true for status 0 AND status 2 — the fix/19 port). */
    in.res_ok       = (res == RES_OK) || (res == RES_GO);
    in.dv_fresh     = (dv != DV_STALE);
    in.dv_ready     = (dv == DV_FRESH_READY);
    in.dv_finished  = (dv == DV_FRESH_FINISHED);
    in.dv_emergency = (dv == DV_FRESH_EMERG);
    return in;
}

// ---- 1. named truth-table cases -------------------------------------
static void test_named_cases(void)
{
    // ASMS off always -> OFF, from every prev.
    for (ASState p : {ASState::OFF, ASState::READY, ASState::DRIVING,
                      ASState::EMERGENCY, ASState::FINISHED})
        CHECK_EQ(p, make(false, true, RES_OK, DV_FRESH_READY), ASState::OFF);

    // Normal arm: ASMS + TS + RES ok, from OFF -> READY.
    CHECK_EQ(ASState::OFF, make(true, true, RES_OK, DV_STALE), ASState::READY);

    // Ready but pipeline NOT ready yet: go is IGNORED (the core new gate).
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_OTHER), ASState::READY);
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_STALE),       ASState::READY);
    // Ready + go + pipeline READY -> DRIVING.
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_READY), ASState::DRIVING);

    // Driving, pipeline still Running (fresh, not ready/finished/emerg),
    // RES still go -> stay DRIVING (no READY<->DRIVING oscillation with a
    // held GO, even though go now implies res_ok).
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_FRESH_OTHER), ASState::DRIVING);
    // Driving, GO RELEASED (RES back to plain ok) -> STAY DRIVING. GO is a
    // trigger, not a level: releasing it must not tear the mission down.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_OK, DV_FRESH_OTHER), ASState::DRIVING);
    // GO held while still OFF (pressed before arming): with go implying
    // res_ok (fix/19 port), the car can still reach READY...
    CHECK_EQ(ASState::OFF, make(true, true, RES_GO, DV_STALE), ASState::READY);
    // ...and the very next tick READY+go+DV_READY takes it to DRIVING.
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_READY), ASState::DRIVING);
    // Driving, pipeline reports FINISHED -> FINISHED.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_FRESH_FINISHED), ASState::FINISHED);
    // Driving, pipeline reports EMERGENCY -> EMERGENCY.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_FRESH_EMERG), ASState::EMERGENCY);
    // Driving, pipeline heartbeat LOST (stale) -> EMERGENCY (the watchdog).
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_STALE), ASState::EMERGENCY);

    // RES e-stop -> EMERGENCY (even from OFF/READY).
    CHECK_EQ(ASState::OFF,   make(true, true, RES_ESTOP, DV_FRESH_READY), ASState::EMERGENCY);
    CHECK_EQ(ASState::READY, make(true, true, RES_ESTOP, DV_FRESH_READY), ASState::EMERGENCY);

    // TS lost while armed -> EMERGENCY; TS lost while OFF is not (yet) emergency.
    CHECK_EQ(ASState::READY,   make(true, false, RES_OK, DV_FRESH_READY), ASState::EMERGENCY);
    CHECK_EQ(ASState::DRIVING, make(true, false, RES_GO, DV_FRESH_OTHER), ASState::EMERGENCY);
    CHECK_EQ(ASState::OFF,     make(true, false, RES_OK, DV_STALE),       ASState::OFF);

    // Emergency + Finished latch until ASMS off.
    CHECK_EQ(ASState::EMERGENCY, make(true, true, RES_OK, DV_FRESH_READY), ASState::EMERGENCY);
    CHECK_EQ(ASState::FINISHED,  make(true, true, RES_OK, DV_FRESH_READY), ASState::FINISHED);

    // Pipeline-only emergency while merely OFF (not armed) must NOT fabricate
    // an emergency out of nowhere — it only bites while READY/DRIVING.
    CHECK_EQ(ASState::OFF, make(true, true, RES_OK, DV_FRESH_EMERG), ASState::READY);
}

// ---- 2. exhaustive invariant sweep ----------------------------------
static void test_invariants(void)
{
    const ASState prevs[] = {ASState::OFF, ASState::READY, ASState::DRIVING,
                             ASState::EMERGENCY, ASState::FINISHED};
    const ResKind ress[]  = {RES_NONE, RES_OK, RES_GO, RES_ESTOP};
    const DvKind  dvs[]   = {DV_STALE, DV_FRESH_OTHER, DV_FRESH_READY,
                             DV_FRESH_FINISHED, DV_FRESH_EMERG};

    for (ASState prev : prevs)
      for (int asms = 0; asms < 2; ++asms)
        for (int ts = 0; ts < 2; ++ts)
          for (ResKind res : ress)
            for (DvKind dv : dvs) {
                AsInputs in = make(asms != 0, ts != 0, res, dv);
                ASState out = as_next_state(prev, in);

                // INV1 fail-safe: ASMS off => OFF, no exceptions.
                if (!in.asms_on) { CHECK_TRUE(out == ASState::OFF); continue; }

                // INV2 Driving gate: the ONLY way to *enter* Driving (from a
                // non-Driving state) is prev==READY && RES go && a fresh DV
                // READY. (Staying in Driving with no trigger is separate.)
                if (out == ASState::DRIVING && prev != ASState::DRIVING) {
                    CHECK_TRUE(prev == ASState::READY && in.res_go && in.dv_ready);
                }

                // INV3/INV4 latches: Emergency/Finished never spontaneously
                // leave (while ASMS stays on).
                if (prev == ASState::EMERGENCY) CHECK_TRUE(out == ASState::EMERGENCY);
                if (prev == ASState::FINISHED)  CHECK_TRUE(out == ASState::FINISHED);

                // INV5 e-stop: an armed, non-latched e-stop forces Emergency.
                if (prev != ASState::EMERGENCY && prev != ASState::FINISHED &&
                    in.res_estop) {
                    CHECK_TRUE(out == ASState::EMERGENCY);
                }

                // INV6 lost-heartbeat watchdog: a stale /dv/status while
                // Driving forces Emergency.
                if (prev == ASState::DRIVING && !in.dv_fresh) {
                    CHECK_TRUE(out == ASState::EMERGENCY);
                }

                // INV7 DRIVING persistence: while Driving with a healthy world
                // (no e-stop, TS on, DV fresh and not finished/emergency), the
                // run continues REGARDLESS of the RES go level — a held or
                // released GO never bounces the state.
                if (prev == ASState::DRIVING && !in.res_estop && in.ts_on &&
                    in.dv_fresh && !in.dv_finished && !in.dv_emergency) {
                    CHECK_TRUE(out == ASState::DRIVING);
                }
            }
}

int main(void)
{
    test_named_cases();
    test_invariants();

    std::printf("\nas_transition: %d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) {
        std::printf("All tests green.\n");
        return 0;
    }
    return 1;
}
