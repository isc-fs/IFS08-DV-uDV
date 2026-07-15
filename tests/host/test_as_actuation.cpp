/**
 * @file test_as_actuation.cpp
 * @brief Host unit tests for the pure AS safe-actuation decision
 *        (Core/Inc/as_actuation.hpp): which AS state / ASMS combination
 *        releases vs fires the EBS, and which opens the AS SDC.
 *
 * Safety-relevant (FS-Rules req #4 / T15.3): the car must be immobilised
 * whenever it is neither autonomously driving nor under manual control, and
 * the terminal safe states must de-energise the TS (open the SDC). This suite
 * guards that rule independently of the implementation, so a future change
 * that (say) forgets to release in manual, or fires while DRIVING, or drops
 * the SDC-open in FINISHED, fails here.
 *
 * Build + run:  make test_as_actuation
 */
#include <cstdio>
#include "as_actuation.hpp"   // AsActuation, as_actuation, ASState

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

#define CHECK(cond)                                                           \
    do {                                                                      \
        ++g_checks;                                                           \
        if (!(cond)) {                                                        \
            ++g_failures;                                                     \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
        }                                                                     \
    } while (0)

static const ASState kStates[] = {
    ASState::OFF, ASState::READY, ASState::DRIVING,
    ASState::EMERGENCY, ASState::FINISHED,
};

// ---- 1. named truth-table cases (readable intent) -------------------
static void test_named(void)
{
    // ASMS on, no stop commanded: EBS engaged in every state EXCEPT DRIVING.
    CHECK(as_actuation(ASState::OFF,       true, false).ebs_release == false);
    CHECK(as_actuation(ASState::READY,     true, false).ebs_release == false);
    CHECK(as_actuation(ASState::DRIVING,   true, false).ebs_release == true);   // the only autonomous release
    CHECK(as_actuation(ASState::EMERGENCY, true, false).ebs_release == false);
    CHECK(as_actuation(ASState::FINISHED,  true, false).ebs_release == false);

    // ASMS off (manual handling): EBS RELEASED regardless of state.
    for (ASState s : kStates)
        CHECK(as_actuation(s, false, false).ebs_release == true);

    // SDC opened ONLY in the terminal safe states (FINISHED / EMERGENCY), ASMS on.
    CHECK(as_actuation(ASState::EMERGENCY, true, false).sdc_open == true);
    CHECK(as_actuation(ASState::FINISHED,  true, false).sdc_open == true);
    CHECK(as_actuation(ASState::OFF,       true, false).sdc_open == false);
    CHECK(as_actuation(ASState::READY,     true, false).sdc_open == false);
    CHECK(as_actuation(ASState::DRIVING,   true, false).sdc_open == false);
    // Manual (ASMS off) never opens the SDC from this rule.
    for (ASState s : kStates)
        CHECK(as_actuation(s, false, false).sdc_open == false);
}

// ---- 2. the end-of-mission stop (DV_STATUS_STOPPING, issue #176) -----
static void test_dv_stopping(void)
{
    // THE point of the feature: DRIVING + stop commanded fires the EBS while
    // leaving the SDC CLOSED and (by the caller) the AS state in DRIVING. This
    // is the one combination the FS AS state table has no row for.
    AsActuation stop = as_actuation(ASState::DRIVING, true, true);
    CHECK(stop.ebs_release == false);   // brakes ON
    CHECK(stop.sdc_open    == false);   // ...but the SDC stays CLOSED

    // It flips ONLY the DRIVING release. Every other ASMS-on state already
    // fires the EBS, so the stop byte is a no-op there.
    CHECK(as_actuation(ASState::OFF,       true, true).ebs_release == false);
    CHECK(as_actuation(ASState::READY,     true, true).ebs_release == false);
    CHECK(as_actuation(ASState::EMERGENCY, true, true).ebs_release == false);
    CHECK(as_actuation(ASState::FINISHED,  true, true).ebs_release == false);

    // ...and it never opens the SDC by itself: only FINISHED/EMERGENCY do.
    CHECK(as_actuation(ASState::EMERGENCY, true, true).sdc_open == true);
    CHECK(as_actuation(ASState::FINISHED,  true, true).sdc_open == true);
    CHECK(as_actuation(ASState::OFF,       true, true).sdc_open == false);
    CHECK(as_actuation(ASState::READY,     true, true).sdc_open == false);

    // SAFETY: ASMS-off (manual handling) WINS over the stop byte. A human
    // pushing the car must get released brakes even if a live pipeline is
    // still asserting STOPPING — otherwise a stale 7 could lock the wheels
    // while the car is being recovered by hand.
    for (ASState s : kStates)
        CHECK(as_actuation(s, false, true).ebs_release == true);
}

// ---- 3. exhaustive sweep asserting the invariants -------------------
static void test_invariants(void)
{
    for (ASState s : kStates)
      for (int asms = 0; asms < 2; ++asms)
        for (int stop = 0; stop < 2; ++stop) {
          AsActuation a = as_actuation(s, asms != 0, stop != 0);

          // INV1: EBS released iff (ASMS off) OR (DRIVING and no stop commanded).
          bool expect_release = (asms == 0) ||
                                (s == ASState::DRIVING && stop == 0);
          CHECK(a.ebs_release == expect_release);

          // INV2: the ONLY state that releases while ASMS is on is DRIVING.
          if (asms != 0 && a.ebs_release) CHECK(s == ASState::DRIVING);

          // INV3: SDC opened iff ASMS on AND a terminal safe state. The stop
          // byte must NEVER influence the SDC — that is what keeps it a stop
          // rather than an emergency.
          bool expect_sdc = (asms != 0) &&
                            (s == ASState::FINISHED || s == ASState::EMERGENCY);
          CHECK(a.sdc_open == expect_sdc);

          // INV4: opening the SDC never coincides with releasing the EBS — a
          // de-energised safe state must keep the brakes fired.
          if (a.sdc_open) CHECK(a.ebs_release == false);

          // INV5: the stop byte can only ever ADD braking, never remove it.
          // Compare against the same case with no stop commanded.
          AsActuation base = as_actuation(s, asms != 0, false);
          if (!base.ebs_release) CHECK(a.ebs_release == false);
        }
}

int main(void)
{
    test_named();
    test_dv_stopping();
    test_invariants();
    std::printf("\nas_actuation: %d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("All tests green.\n");
    (void)name; // used in potential future diagnostics
    return g_failures == 0 ? 0 : 1;
}
