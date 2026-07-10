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
    // ASMS on: EBS engaged in every state EXCEPT DRIVING.
    CHECK(as_actuation(ASState::OFF,       true).ebs_release == false);
    CHECK(as_actuation(ASState::READY,     true).ebs_release == false);
    CHECK(as_actuation(ASState::DRIVING,   true).ebs_release == true);   // the only autonomous release
    CHECK(as_actuation(ASState::EMERGENCY, true).ebs_release == false);
    CHECK(as_actuation(ASState::FINISHED,  true).ebs_release == false);

    // ASMS off (manual handling): EBS RELEASED regardless of state.
    for (ASState s : kStates)
        CHECK(as_actuation(s, false).ebs_release == true);

    // SDC opened ONLY in the terminal safe states (FINISHED / EMERGENCY), ASMS on.
    CHECK(as_actuation(ASState::EMERGENCY, true).sdc_open == true);
    CHECK(as_actuation(ASState::FINISHED,  true).sdc_open == true);
    CHECK(as_actuation(ASState::OFF,       true).sdc_open == false);
    CHECK(as_actuation(ASState::READY,     true).sdc_open == false);
    CHECK(as_actuation(ASState::DRIVING,   true).sdc_open == false);
    // Manual (ASMS off) never opens the SDC from this rule.
    for (ASState s : kStates)
        CHECK(as_actuation(s, false).sdc_open == false);
}

// ---- 2. exhaustive sweep asserting the invariants -------------------
static void test_invariants(void)
{
    for (ASState s : kStates)
      for (int asms = 0; asms < 2; ++asms) {
        AsActuation a = as_actuation(s, asms != 0);

        // INV1: EBS released iff (ASMS off) OR (autonomous DRIVING). Fired otherwise.
        bool expect_release = (asms == 0) || (s == ASState::DRIVING);
        CHECK(a.ebs_release == expect_release);

        // INV2: the ONLY state that releases while ASMS is on is DRIVING.
        if (asms != 0 && a.ebs_release) CHECK(s == ASState::DRIVING);

        // INV3: SDC opened iff ASMS on AND a terminal safe state.
        bool expect_sdc = (asms != 0) &&
                          (s == ASState::FINISHED || s == ASState::EMERGENCY);
        CHECK(a.sdc_open == expect_sdc);

        // INV4: opening the SDC never coincides with releasing the EBS — a
        // de-energised safe state must keep the brakes fired.
        if (a.sdc_open) CHECK(a.ebs_release == false);
      }
}

int main(void)
{
    test_named();
    test_invariants();
    std::printf("\nas_actuation: %d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("All tests green.\n");
    (void)name; // used in potential future diagnostics
    return g_failures == 0 ? 0 : 1;
}
