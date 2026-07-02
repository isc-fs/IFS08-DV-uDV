/**
 * @file test_as_transition.cpp
 * @brief Host unit tests for the pure AS transition decision
 *        (Core/Inc/as_transition.hpp) on the fix/19-uart-assi branch.
 *
 * Guards the safety-relevant AS-state logic extracted from app_task's loop:
 * the ASMS-off / Emergency precedence, the RES e-stop and TS-loss emergency
 * triggers, and — the bug this suite was written for — DRIVING being STICKY
 * so a momentary RES "go" can't oscillate READY<->DRIVING or tear a run down
 * when released. (This branch has no /dv/status handshake; that variant is
 * tested on the pipeline-interface branch.)
 *
 * Build + run:  make test_as_transition
 */
#include <cstdio>

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

// RES is a single status: exactly one of estop/go/ok (or none received).
enum ResKind { RES_NONE, RES_OK, RES_GO, RES_ESTOP };

// Mirror the producer in app_task.cpp: res_ok is true for a received "ok"
// AND for "go" (go implies a healthy link), false for e-stop / none.
static AsInputs make(bool asms, bool ts, ResKind res)
{
    AsInputs in{};
    in.asms_on   = asms;
    in.ts_on     = ts;
    in.res_estop = (res == RES_ESTOP);
    in.res_go    = (res == RES_GO);
    in.res_ok    = (res == RES_OK) || (res == RES_GO);
    return in;
}

static void named_cases()
{
    // ASMS off always wins, from any state.
    CHECK_EQ(ASState::OFF,       make(false, true,  RES_OK),  ASState::OFF);
    CHECK_EQ(ASState::DRIVING,   make(false, true,  RES_GO),  ASState::OFF);
    CHECK_EQ(ASState::EMERGENCY, make(false, true,  RES_OK),  ASState::OFF);

    // Arming: ASMS + TS + RES ok -> READY.
    CHECK_EQ(ASState::OFF,   make(true, true,  RES_OK), ASState::READY);
    // No TS yet -> stay OFF (can't arm).
    CHECK_EQ(ASState::OFF,   make(true, false, RES_OK), ASState::OFF);

    // READY + RES go -> DRIVING.
    CHECK_EQ(ASState::READY, make(true, true,  RES_GO), ASState::DRIVING);

    // --- the oscillation bug this suite exists for ---
    // DRIVING + go still held -> stay DRIVING (no READY<->DRIVING bounce).
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO), ASState::DRIVING);
    // DRIVING + go RELEASED (RES back to plain ok) -> stay DRIVING. go is a
    // momentary trigger, not a level; releasing it must not end the run.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_OK), ASState::DRIVING);

    // GO held BEFORE arming (pressed early): go implies res_ok, so the car
    // can still reach READY, then DRIVING on the next tick.
    CHECK_EQ(ASState::OFF,   make(true, true, RES_GO), ASState::READY);
    CHECK_EQ(ASState::READY, make(true, true, RES_GO), ASState::DRIVING);

    // Emergency triggers.
    CHECK_EQ(ASState::READY,   make(true, true,  RES_ESTOP), ASState::EMERGENCY);
    CHECK_EQ(ASState::DRIVING, make(true, true,  RES_ESTOP), ASState::EMERGENCY);
    CHECK_EQ(ASState::DRIVING, make(true, false, RES_OK),    ASState::EMERGENCY); // TS lost
    CHECK_EQ(ASState::READY,   make(true, false, RES_OK),    ASState::EMERGENCY); // TS lost

    // Emergency latches until ASMS off (its only exit).
    CHECK_EQ(ASState::EMERGENCY, make(true, true,  RES_OK),  ASState::EMERGENCY);
    CHECK_EQ(ASState::EMERGENCY, make(true, true,  RES_GO),  ASState::EMERGENCY);
    CHECK_EQ(ASState::EMERGENCY, make(false, true, RES_OK),  ASState::OFF);
}

static void exhaustive_invariants()
{
    const ASState prevs[] = { ASState::OFF, ASState::READY,
                              ASState::DRIVING, ASState::EMERGENCY };
    const ResKind ress[]  = { RES_NONE, RES_OK, RES_GO, RES_ESTOP };

    for (ASState prev : prevs)
      for (int asms = 0; asms < 2; ++asms)
        for (int ts = 0; ts < 2; ++ts)
          for (ResKind res : ress)
          {
              AsInputs in = make(asms, ts, res);
              ASState out = as_next_state(prev, in);

              // INV1: ASMS off -> OFF, unconditionally.
              if (!in.asms_on) { CHECK_TRUE(out == ASState::OFF); continue; }

              // INV2: Emergency latches while ASMS stays on.
              if (prev == ASState::EMERGENCY) {
                  CHECK_TRUE(out == ASState::EMERGENCY);
                  continue;
              }

              // INV3: RES e-stop (ASMS on, not already latched) -> Emergency.
              if (in.res_estop) { CHECK_TRUE(out == ASState::EMERGENCY); continue; }

              // INV4: TS lost while armed -> Emergency.
              if (!in.ts_on &&
                  (prev == ASState::DRIVING || prev == ASState::READY)) {
                  CHECK_TRUE(out == ASState::EMERGENCY);
                  continue;
              }

              // INV5: DRIVING is sticky — a healthy world keeps the run going
              // regardless of the RES "go" level (no oscillation, no teardown).
              if (prev == ASState::DRIVING && in.ts_on) {
                  CHECK_TRUE(out == ASState::DRIVING);
                  continue;
              }

              // INV6: Driving is only ENTERED from READY with an asserted go.
              if (out == ASState::DRIVING && prev != ASState::DRIVING) {
                  CHECK_TRUE(prev == ASState::READY && in.res_go);
              }
          }
}

int main()
{
    named_cases();
    exhaustive_invariants();

    std::printf("as_transition: %d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("All tests green.\n");
    return g_failures == 0 ? 0 : 1;
}
