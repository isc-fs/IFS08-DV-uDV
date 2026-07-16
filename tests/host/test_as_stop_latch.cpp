/**
 * @file test_as_stop_latch.cpp
 * @brief Host unit tests for the pure DV_STOPPING debounce+latch
 *        (Core/Inc/as_stop_latch.hpp).
 *
 * Two properties the end-of-mission stop (#176) must have and this suite pins:
 *   - DEBOUNCE: the stop arms only after DV_STOPPING_MIN_STREAK consecutive
 *     STOPPING messages, so no single spurious byte 7 can brake at speed. The
 *     streak is counted at the message boundary (ros_set_dv_status); here we
 *     model that counter directly and feed the derived `arm` into the latch.
 *   - LATCH (never released): once armed during a run the brakes stay on until
 *     the run leaves DRIVING — a 7->3 revert or a 7<->3 chatter cannot lift
 *     them. The car must not move again; it only waits for AS Finished.
 *
 * Build + run:  make test_as_stop_latch
 */
#include <cstdio>
#include "as_stop_latch.hpp"   // stop_latch_next, DV_STOPPING_MIN_STREAK

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                           \
    do {                                                                      \
        ++g_checks;                                                           \
        if (!(cond)) {                                                        \
            ++g_failures;                                                     \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
        }                                                                     \
    } while (0)

// Model of the message-boundary streak counter in ros_set_dv_status: +1 per
// STOPPING message, reset to 0 by any other byte. `arm` mirrors app_task's
// dv_stop_arm: fresh + pipeline + streak >= threshold (freshness/pipeline held
// true here; those gates are exercised in the app_task-level cases upstream).
struct Stream {
    unsigned streak = 0;
    void msg_stopping() { if (streak < 255u) ++streak; }
    void msg_other()    { streak = 0; }
    bool arm() const    { return streak >= DV_STOPPING_MIN_STREAK; }
};

// ---- 1. pure latch truth table --------------------------------------
static void test_latch_pure(void)
{
    // Not driving -> always cleared, regardless of prev/arm.
    CHECK(stop_latch_next(false, false, false) == false);
    CHECK(stop_latch_next(true,  false, false) == false);   // clears on leaving Driving
    CHECK(stop_latch_next(true,  false, true)  == false);
    CHECK(stop_latch_next(false, false, true)  == false);

    // Driving: sets on a fresh arm, holds once set.
    CHECK(stop_latch_next(false, true, false) == false);    // idle
    CHECK(stop_latch_next(false, true, true)  == true);     // rising edge arms
    CHECK(stop_latch_next(true,  true, false) == true);     // STICKY: arm gone, still latched
    CHECK(stop_latch_next(true,  true, true)  == true);     // idempotent while armed
}

// INV: within a run the latch is monotonic (never true->false); it only clears
// by leaving Driving. Exhaustive over (prev, arm) for in_driving=true.
static void test_latch_monotonic(void)
{
    for (int prev = 0; prev < 2; ++prev)
      for (int arm = 0; arm < 2; ++arm) {
        bool next = stop_latch_next(prev != 0, true, arm != 0);
        if (prev) CHECK(next == true);                 // once latched, stays latched
        // leaving Driving always clears, whatever prev/arm:
        CHECK(stop_latch_next(prev != 0, false, arm != 0) == false);
      }
}

// ---- 2. DEBOUNCE: N-1 spurious bytes never arm ----------------------
static void test_debounce(void)
{
    // A single spurious STOPPING, surrounded by RUNNING, never arms.
    Stream s;
    s.msg_other();      /* RUNNING */          CHECK(!s.arm());
    s.msg_stopping();   /* one stray 7 */      CHECK(!s.arm());
    s.msg_other();      /* back to RUNNING */  CHECK(!s.arm());

    // Exactly threshold-1 consecutive STOPPING still does NOT arm.
    Stream s2;
    for (unsigned i = 0; i < DV_STOPPING_MIN_STREAK - 1u; ++i) {
        s2.msg_stopping();
        CHECK(!s2.arm());                       // still below threshold
    }
    // The threshold-th consecutive message arms.
    s2.msg_stopping();
    CHECK(s2.arm());

    // A gap resets the count: threshold-1, then a RUNNING, then more, must
    // re-earn the full streak.
    Stream s3;
    for (unsigned i = 0; i < DV_STOPPING_MIN_STREAK - 1u; ++i) s3.msg_stopping();
    s3.msg_other();                             // gap!
    CHECK(!s3.arm());
    for (unsigned i = 0; i < DV_STOPPING_MIN_STREAK - 1u; ++i) {
        s3.msg_stopping();
        CHECK(!s3.arm());                       // must count from scratch
    }
    s3.msg_stopping();
    CHECK(s3.arm());
}

// ---- 3. scenario replays (stream + latch together) ------------------
// Drive the latch tick-by-tick the way app_task does, feeding s.arm().
static void test_scenarios(void)
{
    const unsigned N = DV_STOPPING_MIN_STREAK;

    // Scenario A — normal stop: N STOPPING while Driving -> latched, then a
    // 7->3 revert MUST NOT release it. This is fault 2 from the PR.
    {
        Stream s; bool latched = false;
        for (unsigned i = 0; i < N; ++i) { s.msg_stopping();
            latched = stop_latch_next(latched, /*driving=*/true, s.arm()); }
        CHECK(latched == true);                 // armed after N messages
        // pipeline reverts to RUNNING (byte 3) repeatedly:
        for (int i = 0; i < 5; ++i) { s.msg_other();
            latched = stop_latch_next(latched, true, s.arm()); }
        CHECK(latched == true);                 // STILL latched — brakes stay on
    }

    // Scenario B — chatter 7<->3<->7 while Driving: must not un-latch once set,
    // and must not release between edges after it is set.
    {
        Stream s; bool latched = false;
        // build up to armed
        for (unsigned i = 0; i < N; ++i) { s.msg_stopping();
            latched = stop_latch_next(latched, true, s.arm()); }
        CHECK(latched);
        const int seq[] = {0,1,0,1,1,0,1,0,0,1};   // 1=STOPPING, 0=other
        for (int b : seq) {
            if (b) s.msg_stopping(); else s.msg_other();
            latched = stop_latch_next(latched, true, s.arm());
            CHECK(latched == true);             // never drops mid-run
        }
    }

    // Scenario C — single spurious 7 mid-run NEVER latches (debounce holds end
    // to end): fault 1 from the PR.
    {
        Stream s; bool latched = false;
        const int seq[] = {0,0,1,0,0,0,1,0,1,0}; // isolated 7s, never N in a row
        for (int b : seq) {
            if (b) s.msg_stopping(); else s.msg_other();
            latched = stop_latch_next(latched, true, s.arm());
            CHECK(latched == false);            // brakes never touched
        }
    }

    // Scenario D — leaving Driving clears the latch; a NEW run must re-earn its
    // own streak (per-run debounce, matching app_task's streak reset on unbind).
    {
        Stream s; bool latched = false;
        for (unsigned i = 0; i < N; ++i) { s.msg_stopping();
            latched = stop_latch_next(latched, true, s.arm()); }
        CHECK(latched);
        // run ends -> FINISHED (not driving). app_task also zeroes the streak:
        latched = stop_latch_next(latched, /*driving=*/false, s.arm());
        CHECK(latched == false);
        s.msg_other();  // streak reset (models g_dv_stopping_streak.store(0))
        // new run begins; one stray 7 must not immediately re-arm:
        s.msg_stopping();
        latched = stop_latch_next(latched, /*driving=*/true, s.arm());
        CHECK(latched == false);
    }
}

int main(void)
{
    test_latch_pure();
    test_latch_monotonic();
    test_debounce();
    test_scenarios();
    std::printf("\nas_stop_latch: %d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("All tests green.\n");
    return g_failures == 0 ? 0 : 1;
}
