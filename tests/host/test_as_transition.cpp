/**
 * @file test_as_transition.cpp
 * @brief Host unit tests for the pure AS transition decision
 *        (Core/Inc/as_transition.hpp) added for the stock-typed DV-pipeline
 *        interface.
 *
 * This is the safety-relevant logic that changed when the uDV moved to the
 * topic-based pipeline contract: the /dv/status handshake gate on
 * Ready->Driving, the FINISHED / EMERGENCY reactions, the "pipeline
 * heartbeat lost while driving" watchdog, and the EBS-init gate on arming
 * (dev 9395936: no READY until the init sequence reports Done). The IWDG
 * task-stall watchdog is unchanged and covered by test_safety_eval; this
 * suite guards the AS-state logic that feeds it.
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

// ebs_done defaults to true so the pre-gate cases read unchanged; the
// EBS-init gate cases (dev 9395936) pass false explicitly. steer_emerg
// defaults to false (no steering fault) so the pre-existing cases read
// unchanged; the steering-fault cases pass true explicitly.
// needs_pipeline defaults to TRUE so every pre-existing case keeps its
// pipeline-gated semantics; the standalone-mission cases pass false.
// mission_complete defaults to false; the self-finish cases pass true.
// mission_valid defaults to TRUE (the selected AMI code maps to a real
// mission) so every pre-existing case is unchanged; the unknown/SHUTDOWN
// cases pass false to assert GO is refused.
// ready_dwell defaults to TRUE so every pre-existing GO case still reaches
// DRIVING; the AS-Ready dwell-gate cases pass false to assert GO is held
// until the mandated >=5 s READY dwell has elapsed (FS-Rules).
static AsInputs make(bool asms, bool ts, ResKind res, DvKind dv,
                     bool ebs_done = true, bool steer_emerg = false,
                     bool needs_pipeline = true, bool mission_complete = false,
                     bool mission_valid = true, bool ready_dwell = true,
                     bool asb_p_ok = true, bool standstill = false)
{
    AsInputs in{};
    in.asms_on         = asms;
    in.ts_on           = ts;
    in.res_estop       = (res == RES_ESTOP);
    in.res_go          = (res == RES_GO);
    /* Mirrors the producer in app_task.cpp: "go" also means the RES link is
     * healthy (res_ok true for status 0 AND status 2 — the fix/19 port). */
    in.res_ok          = (res == RES_OK) || (res == RES_GO);
    in.steer_emergency = steer_emerg;
    in.ebs_init_done   = ebs_done;
    in.dv_fresh        = (dv != DV_STALE);
    in.dv_ready        = (dv == DV_FRESH_READY);
    in.dv_finished     = (dv == DV_FRESH_FINISHED);
    in.dv_emergency    = (dv == DV_FRESH_EMERG);
    in.mission_needs_pipeline = needs_pipeline;
    in.mission_complete       = mission_complete;
    in.mission_valid          = mission_valid;
    in.ready_dwell_elapsed    = ready_dwell;
    in.asb_pressure_ok        = asb_p_ok;
    in.vehicle_standstill     = standstill;
    return in;
}

// ---- 1. named truth-table cases -------------------------------------
static void test_named_cases(void)
{
    // ASMS off always -> OFF, from every prev.
    for (ASState p : {ASState::OFF, ASState::READY, ASState::DRIVING,
                      ASState::EMERGENCY, ASState::FINISHED})
        CHECK_EQ(p, make(false, true, RES_OK, DV_FRESH_READY), ASState::OFF);

    // Normal arm: ASMS + TS + RES ok (EBS init done), from OFF -> READY.
    CHECK_EQ(ASState::OFF, make(true, true, RES_OK, DV_STALE), ASState::READY);

    // --- EBS-init gate (dev 9395936): no READY until init reports Done ---
    // (app_task only steps the EBS init FSM while OFF; arming early would
    // strand it and ASBChecksOK could never pass. Failed init = stay OFF.)
    CHECK_EQ(ASState::OFF, make(true, true, RES_OK, DV_STALE, false), ASState::OFF);
    CHECK_EQ(ASState::OFF, make(true, true, RES_GO, DV_FRESH_READY, false), ASState::OFF);
    // E-stop precedence is NOT weakened by the gate.
    CHECK_EQ(ASState::OFF, make(true, true, RES_ESTOP, DV_STALE, false), ASState::EMERGENCY);

    // Ready but pipeline NOT ready yet: go is IGNORED (the core new gate).
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_OTHER), ASState::READY);
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_STALE),       ASState::READY);
    // Ready + go + pipeline READY -> DRIVING.
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_READY), ASState::DRIVING);

    // --- AS-Ready dwell gate (FS-Rules >=5 s in READY before GO) ---
    // 10th make() arg is ready_dwell. A GO before the dwell elapses is REFUSED:
    // the car holds in READY even with an otherwise-complete GO gate.
    // (A) pipeline mission, DV READY, GO, but dwell NOT elapsed -> stay READY.
    CHECK_EQ(ASState::READY,
             make(true, true, RES_GO, DV_FRESH_READY, true, false, true, false, true, false),
             ASState::READY);
    // (B) standalone mission, GO, but dwell NOT elapsed -> stay READY.
    CHECK_EQ(ASState::READY,
             make(true, true, RES_GO, DV_STALE, true, false, false, false, true, false),
             ASState::READY);
    // (C) same standalone GO once the dwell HAS elapsed -> DRIVING (control).
    CHECK_EQ(ASState::READY,
             make(true, true, RES_GO, DV_STALE, true, false, false, false, true, true),
             ASState::DRIVING);
    // (D) the dwell gate must NOT weaken the fail-safe order: an e-stop from
    // READY with the dwell not elapsed still forces EMERGENCY.
    CHECK_EQ(ASState::READY,
             make(true, true, RES_ESTOP, DV_STALE, true, false, true, false, true, false),
             ASState::EMERGENCY);
    // (E) the dwell gates ENTRY only, not persistence: already DRIVING with a
    // healthy world stays DRIVING regardless of the dwell flag.
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_GO, DV_FRESH_OTHER, true, false, true, false, true, false),
             ASState::DRIVING);

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

    // Steering grave-fault (ESTADO_MOTOR_EMERGENCIA) -> EMERGENCY, unconditional
    // like an RES e-stop (a dead steering motor means the car can't steer).
    // steer_emerg is the 6th make() arg (after ebs_done).
    CHECK_EQ(ASState::OFF,     make(true, true,  RES_OK, DV_STALE,       true, true), ASState::EMERGENCY);
    CHECK_EQ(ASState::READY,   make(true, true,  RES_OK, DV_FRESH_READY, true, true), ASState::EMERGENCY);
    CHECK_EQ(ASState::DRIVING, make(true, true,  RES_GO, DV_FRESH_OTHER, true, true), ASState::EMERGENCY);
    // ...but ASMS-off still wins over a steering fault (fail-safe order).
    CHECK_EQ(ASState::READY,   make(false, true, RES_OK, DV_STALE,       true, true), ASState::OFF);

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

    // --- Standalone missions (inspection / EBS test): needs_pipeline=false ---
    // The 7th make() arg is needs_pipeline; the 8th is mission_complete.
    // GO from READY is honoured WITHOUT any DV READY (no pipeline to wait on).
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_STALE,      true, false, false),
             ASState::DRIVING);
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_OTHER, true, false, false),
             ASState::DRIVING);
    // A stale /dv/status while driving standalone must NOT trip Emergency
    // (there is no pipeline heartbeat) — the run continues.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_STALE, true, false, false),
             ASState::DRIVING);
    // A pipeline EMERGENCY byte must be IGNORED by a standalone mission.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_FRESH_EMERG, true, false, false),
             ASState::DRIVING);
    // Standalone self-finish: mission_complete -> FINISHED.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_STALE, true, false, false, true),
             ASState::FINISHED);

    // mission_complete also ends a PIPELINE mission from DRIVING (defensive:
    // whoever raises it wins over "keep driving").
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_FRESH_OTHER, true, false, true, true),
             ASState::FINISHED);
    // ...but an e-stop still beats a self-finish (fail-safe order).
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_ESTOP, DV_STALE, true, false, false, true),
             ASState::EMERGENCY);

    // The safety triggers that are mission-independent still fire for a
    // standalone mission: RES e-stop, steering grave-fault, TS loss.
    CHECK_EQ(ASState::DRIVING, make(true, true,  RES_ESTOP, DV_STALE, true, false, false),
             ASState::EMERGENCY);
    CHECK_EQ(ASState::DRIVING, make(true, true,  RES_GO,    DV_STALE, true, true,  false),
             ASState::EMERGENCY);
    CHECK_EQ(ASState::DRIVING, make(true, false, RES_GO,    DV_STALE, true, false, false),
             ASState::EMERGENCY);

    // --- Invalid mission (unknown / SHUTDOWN code -> mission_for_code nullptr) ---
    // The 9th make() arg is mission_valid. GO is REFUSED when the selected code
    // maps to no mission, so an unknown/SHUTDOWN code can never enter DRIVING
    // and release the brakes with no mission body. READY holds.
    // (A) would-be standalone GO (needs_pipeline=false) is refused when invalid.
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_STALE, true, false, false, false, false),
             ASState::READY);
    // (B) would-be pipeline GO (needs_pipeline=true, DV READY) is refused too.
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_FRESH_READY, true, false, true, false, false),
             ASState::READY);
    // (C) an invalid mission can still ARM to READY (a safe braked state) — the
    // gate is on GO (Driving), not on arming.
    CHECK_EQ(ASState::OFF, make(true, true, RES_OK, DV_STALE, true, false, false, false, false),
             ASState::READY);
    // (D) a valid mission is unaffected: same inputs but mission_valid=true drive.
    CHECK_EQ(ASState::READY, make(true, true, RES_GO, DV_STALE, true, false, false, false, true),
             ASState::DRIVING);
    // (E) safety exits still win over an invalid mission: an e-stop from READY
    // with an invalid mission still forces EMERGENCY.
    CHECK_EQ(ASState::READY, make(true, true, RES_ESTOP, DV_STALE, true, false, false, false, false),
             ASState::EMERGENCY);
    // (F) an invalid mission already DRIVING (can't happen via GO, but defensive)
    // stays DRIVING with a healthy world — the gate is on ENTRY, not persistence.
    CHECK_EQ(ASState::DRIVING, make(true, true, RES_GO, DV_STALE, true, false, false, false, false),
             ASState::DRIVING);
}

// ---- 2. exhaustive invariant sweep ----------------------------------
// ---- ASB stored-energy trip (named cases) ---------------------------
// The air tanks are the EBS's stored energy. Below 3 bar on EITHER sensor the
// brake may not be able to stop the car, so an armed car must abandon the run.
// Trailing make() arg is asb_pressure_ok.
static void test_asb_pressure(void)
{
    // DRIVING with the tanks drained -> Emergency. This could not be caught at
    // all before: tank pressure reached only ASBChecksOK(), which gated AS Ready
    // and nothing else, so a mid-run drain went unnoticed.
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  true, false, true, true, false),
             ASState::EMERGENCY);

    // READY with the tanks drained -> Emergency (armed, so the run is over).
    CHECK_EQ(ASState::READY,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  true, false, true, true, false),
             ASState::EMERGENCY);

    // OFF with flat tanks -> stays OFF, NOT Emergency. This is the boot case:
    // tanks fill after power-on and Emergency LATCHES until ASMS-off, so
    // tripping here would mean the car could never arm even once filled.
    CHECK_EQ(ASState::OFF,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  true, false, true, true, false),
             ASState::OFF);

    // ...and flat tanks block ARMING: OFF -> READY refused. Without this a
    // drain occurring after the init self-check passed could still arm, since
    // ebs_init_done is a one-shot latch on the init FSM.
    CHECK_EQ(ASState::OFF,
             make(true, true, RES_OK, DV_FRESH_READY, true, false,
                  true, false, true, true, false),
             ASState::OFF);

    // Same inputs but healthy tanks DO arm — proves the pressure term is what
    // refused above, not some other gate.
    CHECK_EQ(ASState::OFF,
             make(true, true, RES_OK, DV_FRESH_READY, true, false,
                  true, false, true, true, true),
             ASState::READY);

    // A healthy armed car is unaffected: DRIVING stays DRIVING.
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  true, false, true, true, true),
             ASState::DRIVING);

    // ASMS-off still wins over a low-pressure trip (manual handling).
    CHECK_EQ(ASState::DRIVING,
             make(false, true, RES_OK, DV_FRESH_OTHER, true, false,
                  true, false, true, true, false),
             ASState::OFF);

    // --- FINISH wins at standstill with a completed mission, even low tanks ---
    // 12th make() arg is vehicle_standstill.
    // Standalone mission complete + STOPPED + tanks LOW -> FINISHED (not
    // Emergency): the stop that ended the mission may have drained the tanks,
    // and the car is already halted. This is the case that motivated the whole
    // finishing_at_standstill carve-out.
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  /*needs_pipeline=*/false, /*mission_complete=*/true,
                  true, true, /*asb_ok=*/false, /*standstill=*/true),
             ASState::FINISHED);

    // Pipeline mission, fresh DV FINISHED + STOPPED + tanks LOW -> FINISHED.
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_OK, DV_FRESH_FINISHED, true, false,
                  /*needs_pipeline=*/true, /*mission_complete=*/false,
                  true, true, /*asb_ok=*/false, /*standstill=*/true),
             ASState::FINISHED);

    // Same finish but STILL MOVING (not standstill) + tanks LOW -> Emergency:
    // a car that hasn't stopped with no brake energy is a real emergency, and
    // the finish does NOT get to override that.
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  /*needs_pipeline=*/false, /*mission_complete=*/true,
                  true, true, /*asb_ok=*/false, /*standstill=*/false),
             ASState::EMERGENCY);

    // A HARD emergency still wins even at a finished standstill: e-stop during
    // the finish -> Emergency, not FINISHED. (Tanks healthy here to isolate it.)
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_ESTOP, DV_FRESH_OTHER, true, false,
                  /*needs_pipeline=*/false, /*mission_complete=*/true,
                  true, true, /*asb_ok=*/true, /*standstill=*/true),
             ASState::EMERGENCY);

    // Control: mission complete + STOPPED + tanks HEALTHY -> FINISHED (proves
    // the low-tank cases above finish for the right reason, not a fluke).
    CHECK_EQ(ASState::DRIVING,
             make(true, true, RES_OK, DV_FRESH_OTHER, true, false,
                  /*needs_pipeline=*/false, /*mission_complete=*/true,
                  true, true, /*asb_ok=*/true, /*standstill=*/true),
             ASState::FINISHED);
}

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
          for (int ebs = 0; ebs < 2; ++ebs)
           for (int se = 0; se < 2; ++se)
            for (int np = 0; np < 2; ++np)
             for (int mc = 0; mc < 2; ++mc)
              for (int mv = 0; mv < 2; ++mv)
              for (int dw = 0; dw < 2; ++dw)
              for (int ap = 0; ap < 2; ++ap)
              for (int ss = 0; ss < 2; ++ss)
              for (ResKind res : ress)
              for (DvKind dv : dvs) {
                AsInputs in = make(asms != 0, ts != 0, res, dv, ebs != 0, se != 0,
                                   np != 0, mc != 0, mv != 0, dw != 0, ap != 0,
                                   ss != 0);
                ASState out = as_next_state(prev, in);

                // INV1 fail-safe: ASMS off => OFF, no exceptions.
                if (!in.asms_on) { CHECK_TRUE(out == ASState::OFF); continue; }

                // INV2 Driving gate: the ONLY way to *enter* Driving (from a
                // non-Driving state) is prev==READY && RES go && a VALID mission
                // && the mission's start condition — a fresh DV READY for a
                // pipeline mission, or nothing extra for a standalone one.
                if (out == ASState::DRIVING && prev != ASState::DRIVING) {
                    CHECK_TRUE(prev == ASState::READY && in.res_go &&
                               in.mission_valid &&
                               (in.dv_ready || !in.mission_needs_pipeline) &&
                               in.ready_dwell_elapsed);
                }

                // INV2b mission-valid gate: an invalid mission (unknown /
                // SHUTDOWN code) can NEVER enter Driving from a non-Driving
                // state, regardless of every other input.
                if (prev != ASState::DRIVING && !in.mission_valid) {
                    CHECK_TRUE(out != ASState::DRIVING);
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

                // INV5b ASB stored energy: an armed car whose air tanks are
                // below 3 bar MUST go to Emergency — the EBS may no longer be
                // able to stop it, so it must not keep driving. Armed only:
                // OFF legitimately has flat tanks while filling, and Emergency
                // latches, so tripping there would strand the car forever.
                // EXCEPTION: a genuine finish at standstill (mission over AND
                // stopped) reports FINISHED instead — the braking that ended
                // the mission may have drained the tanks, and the car is
                // already halted. That exception is asserted by INV5d below.
                const bool finishing_at_standstill =
                    (prev == ASState::DRIVING) && in.vehicle_standstill &&
                    (in.mission_complete ||
                     (in.mission_needs_pipeline && in.dv_finished));
                if (prev != ASState::EMERGENCY && prev != ASState::FINISHED &&
                    !in.asb_pressure_ok && !finishing_at_standstill &&
                    (prev == ASState::DRIVING || prev == ASState::READY)) {
                    CHECK_TRUE(out == ASState::EMERGENCY);
                }

                // INV5d finish wins at standstill: a completed mission that has
                // stopped the car reports FINISHED even with the tanks below 3
                // bar — but ONLY when no HARD emergency (e-stop, steering fault,
                // TS loss, pipeline emerg / lost heartbeat) is also present.
                // Those still win over a finish.
                if (finishing_at_standstill && !in.res_estop &&
                    !in.steer_emergency && in.ts_on &&
                    !(in.mission_needs_pipeline && (!in.dv_fresh || in.dv_emergency))) {
                    CHECK_TRUE(out == ASState::FINISHED);
                }

                // INV5c no stored energy, no arming: flat tanks can never reach
                // READY, whatever else is true. ebs_init_done is a one-shot
                // latch, so without this a post-init drain could still arm.
                if (!in.asb_pressure_ok && prev != ASState::READY) {
                    CHECK_TRUE(out != ASState::READY);
                }

                // INV6 lost-heartbeat watchdog: a stale /dv/status while
                // Driving forces Emergency — but ONLY for a pipeline mission.
                // A standalone mission has no heartbeat to lose.
                if (prev == ASState::DRIVING && in.mission_needs_pipeline &&
                    !in.dv_fresh) {
                    CHECK_TRUE(out == ASState::EMERGENCY);
                }

                // INV6b standalone isolation: a standalone mission driving with
                // a stale/emergency pipeline byte but otherwise healthy and not
                // self-finished stays DRIVING — the pipeline can't touch it.
                // "Healthy" now includes ASB stored energy: drained tanks trip
                // Emergency regardless of who owns the mission.
                if (prev == ASState::DRIVING && !in.mission_needs_pipeline &&
                    !in.res_estop && !in.steer_emergency && in.ts_on &&
                    in.asb_pressure_ok && !in.mission_complete) {
                    CHECK_TRUE(out == ASState::DRIVING);
                }

                // INV7 DRIVING persistence: while Driving with a healthy world
                // (no e-stop, no steering fault, TS on, not self-finished, and
                // for a pipeline mission a fresh DV that is not finished/emerg),
                // the run continues REGARDLESS of the RES go level.
                if (prev == ASState::DRIVING && !in.res_estop && !in.steer_emergency &&
                    in.ts_on && in.asb_pressure_ok && !in.mission_complete &&
                    (!in.mission_needs_pipeline ||
                     (in.dv_fresh && !in.dv_finished && !in.dv_emergency))) {
                    CHECK_TRUE(out == ASState::DRIVING);
                }

                // INV8 EBS-init gate (dev 9395936): READY is only ENTERED
                // with the EBS init sequence Done.
                if (out == ASState::READY && prev != ASState::READY) {
                    CHECK_TRUE(in.ebs_init_done);
                }

                // INV9 steering grave-fault: a non-latched steer_emergency
                // forces Emergency, unconditional like an e-stop.
                if (prev != ASState::EMERGENCY && prev != ASState::FINISHED &&
                    in.steer_emergency) {
                    CHECK_TRUE(out == ASState::EMERGENCY);
                }

                // INV10 self-finish: from DRIVING, mission_complete ends the
                // run in FINISHED unless a higher-priority safe-state exit
                // (ASMS off / e-stop / steering fault / TS loss / pipeline
                // emerg or lost) fires first. ASB low pressure is ALSO such an
                // exit — EXCEPT at standstill, where the finish wins (INV5d).
                // So FINISHED is guaranteed when the tanks are healthy OR the
                // car is stopped.
                if (prev == ASState::DRIVING && in.mission_complete &&
                    !in.res_estop && !in.steer_emergency && in.ts_on &&
                    (in.asb_pressure_ok || in.vehicle_standstill) &&
                    !(in.mission_needs_pipeline &&
                      (!in.dv_fresh || in.dv_emergency))) {
                    CHECK_TRUE(out == ASState::FINISHED);
                }
            }
}

int main(void)
{
    test_named_cases();
    test_asb_pressure();
    test_invariants();

    std::printf("\nas_transition: %d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) {
        std::printf("All tests green.\n");
        return 0;
    }
    return 1;
}
