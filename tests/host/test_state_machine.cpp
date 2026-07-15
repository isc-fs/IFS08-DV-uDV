/**
 * @file test_state_machine.cpp
 * @brief Host unit tests for the autonomy state machine + EBS manager.
 *
 * Exercises the REAL state_manager.cpp / ebs_manager.cpp logic (linked,
 * not re-implemented) against a controllable hardware_io stub + the real
 * can_globals / ros_globals atomics. Verifies:
 *   - getAssiStatusCode mapping (FS-Rules T14.9 / T14.8 codes)
 *   - the EBS init sequence (happy path + failure/timeout branches + checks)
 *   - every AS state transition (OFF/READY/DRIVING/EMERGENCY/FINISHED)
 *     incl. the EBS-readback polarity the integration fixed.
 *
 * No HAL / FreeRTOS scheduler — hardware_io is stubbed here, and the
 * stub headers in stubs/ provide the opaque RTOS types can_globals.h
 * pulls in. Build + run:  make test_state_machine
 */
#include <cstdio>

#include "state_manager.hpp"
#include "ebs_manager.hpp"
#include "bench_stubs.h"   /* stub-aware — see the EBS-init note below */
#include "can_globals.h"
#include "ros_globals.h"

/* This suite is BENCH-STUB-AWARE. The bench stubs (bench_stubs.h) may be
 * enabled on dev for pre-production bench/car testing. When BENCH_STUB_EBS_INIT
 * is on, EbsManager short-circuits the init self-check straight to Done, so the
 * step-by-step init / failure-path cases below are compiled out (they cannot
 * test a bypassed sequence) and a stub-path check runs in their place.
 * Everything else — the ASSI codes, the EBS check primitives, and every AS
 * state transition — is exercised either way. BENCH_STUB_DVPC gates only
 * app_task (not linked here), so it has no effect on this suite. */

// ---------------------------------------------------------------------
// Controllable hardware_io stub (replaces the HAL-backed hardware_io.c).
// Models the EBS actuator pins so is_ebs_active() reflects the confirmed
// LOW=fire convention the real driver uses: s_ebs_pinN holds the raw pin
// level (enable(true) -> HIGH -> released; enable(false) -> LOW -> fired),
// and the brake is "active" when either pin is LOW.
// ---------------------------------------------------------------------
namespace {
bool     s_asms_on        = false;
bool     s_tsms_on        = false;
bool     s_sdc_ready      = false;
float    s_pressure1      = 0.0f;
float    s_pressure2      = 0.0f;
bool     s_ebs_pin1       = false;   // D1 raw level (LOW=fire, HIGH=release)
bool     s_ebs_pin2       = false;   // D2 raw level (LOW=fire, HIGH=release)
bool     s_sdc_closed     = false;   // D4 (HIGH = SDC closed)
uint32_t s_now_ms         = 0;
}  // namespace

extern "C" {
void  hardware_io_set_as_close_sdc(bool on)            { s_sdc_closed = on; }
void  hardware_io_enable_ebs_actuator_1(bool enable)   { s_ebs_pin1 = enable; }
void  hardware_io_enable_ebs_actuator_2(bool enable)   { s_ebs_pin2 = enable; }
bool  hardware_io_is_ebs_active(void)                  { return !s_ebs_pin1 || !s_ebs_pin2; }
bool  hardware_io_read_sdc_is_ready(void)              { return s_sdc_ready; }
bool  hardware_io_read_asms_on(void)                   { return s_asms_on; }
bool  hardware_io_read_tsms_on(void)                   { return s_tsms_on; }
bool  hardware_io_read_sdc_res_open(void)              { return false; }
float hardware_io_read_actuator1_storage_pressure(void){ return s_pressure1; }
float hardware_io_read_actuator2_storage_pressure(void){ return s_pressure2; }
uint32_t hardware_io_now_ms(void)                      { return s_now_ms; }
void  hardware_io_watchdog_kick(void)                  { /* no-op */ }
}

// ---------------------------------------------------------------------
// Tiny test harness
// ---------------------------------------------------------------------
static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        ++g_checks;                                                       \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                 \
    } while (0)

/* Reset both singletons, the stub hardware, and all input globals to a
 * clean OFF/Start baseline before each test. */
static void reset_world(void)
{
    s_asms_on = false; s_tsms_on = false; s_sdc_ready = false;
    s_pressure1 = 0.0f; s_pressure2 = 0.0f;
    s_ebs_pin1 = false; s_ebs_pin2 = false;
    /* Start the mock clock at a NON-ZERO tick. hardware_io_now_ms() is HAL_GetTick,
     * which has been running since HAL_Init long before FDCAN is up, so a real
     * 0x504 can never be stamped at tick 0 — and 0 is the "never received"
     * sentinel in can_ts_active_fresh() (same idiom as g_res_last_rx_tick). At 0
     * the mock would make a freshly-published frame read as never-seen. */
    s_sdc_closed = false; s_now_ms = 1000;

    g_can_ts_active.store(false);
    g_can_ts_active_stamp_ms.store(0);   /* 0 = no 0x504 ever seen -> TS off */
    g_can_sdc_res_open.store(false);
    g_can_brake_over_limit.store(false);
    g_can_mission_id.store(-1);   /* -1 = none; 0 is a valid 0-based mission */
    g_can_r2d.store(false);
    g_imu_vehicle_standstill.store(true);
    g_set_mission_ready.store(false);
    g_finished_cmd.store(false);

    EbsManager::getInstance().reset();
    StateManager::getInstance().reset();
}

/* Model the ECU's 100 ms 0x504 VCU_ts_active cyclic: publish a frame stamped at
 * the CURRENT mock time. TS is sourced from CAN, not the TSMS pin (#180), so a
 * test that advances s_now_ms past TS_ACTIVE_STALE_MS must re-publish exactly as
 * the real ECU would. A frame that stops arriving is MEANT to read as TS-off —
 * that fail-safe is the point of the change, and test_sm_ts_stale_is_off covers it. */
static void ts_can_publish(bool active)
{
    g_can_ts_active.store(active);
    g_can_ts_active_stamp_ms.store(s_now_ms);
}

/* Step the EBS init FSM all the way to Done with nominal hardware. */
static void drive_ebs_to_done(EbsManager& ebs)
{
    ebs.initSequenceStep();                        // Start -> WaitLow (unconditional)
    ebs.initSequenceStep();                        // WaitLow -> CheckPressure
    s_pressure1 = 5.0f; s_pressure2 = 5.0f;        // both tanks > 3 bar
    ebs.initSequenceStep();                        // CheckPressure -> WaitTS (closes SDC)
    s_asms_on = true; ts_can_publish(true);        // ASMS(A3) + fresh ECU 0x504
    ebs.initSequenceStep();                        // WaitTS -> CheckActuator1 (A1 on)
    g_can_brake_over_limit.store(true);
    ebs.initSequenceStep();                        // CheckActuator1 -> WaitInter (A1 off)
    s_now_ms += 6000;
    ebs.initSequenceStep();                        // WaitInter -> CheckActuator2 (A2 on)
    ebs.initSequenceStep();                        // CheckActuator2 -> Done (A2 off)
}

/* Helper: set the "all preconditions met" inputs for READY/DRIVING. */
static void set_drive_preconditions(void)
{
    s_asms_on = true;
    ts_can_publish(true);                          // fresh ECU 0x504 -> TS live
    g_can_mission_id.store(5);
    g_set_mission_ready.store(true);
}

// ---------------------------------------------------------------------
// getAssiStatusCode (FS-Rules T14.9 / T14.8 byte codes)
// ---------------------------------------------------------------------
static void test_assi_codes(void)
{
    CHECK(StateManager::getAssiStatusCode(ASState::OFF)       == 0x00);
    CHECK(StateManager::getAssiStatusCode(ASState::EMERGENCY) == 0x01);
    CHECK(StateManager::getAssiStatusCode(ASState::READY)     == 0x02);
    CHECK(StateManager::getAssiStatusCode(ASState::DRIVING)   == 0x03);
    CHECK(StateManager::getAssiStatusCode(ASState::FINISHED)  == 0x04);
}

// ---------------------------------------------------------------------
// EBS init sequence
//
// The real step-by-step sequence + failure paths are only meaningful with the
// stub OFF (BENCH_STUB_EBS_INIT=0). With the stub ON, initSequenceStep() jumps
// straight to Done, so these are compiled out and test_ebs_stub_short_circuits()
// runs instead.
// ---------------------------------------------------------------------
#if !BENCH_STUB_EBS_INIT
static void test_ebs_happy_path(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    CHECK(ebs.getInitState() == EBSInitState::Start);

    /* Start/WaitLow now advance unconditionally (the dead RES_1_IN "SDC-ready"
     * gate was removed); CheckPressure is the first real gate. */
    CHECK(ebs.initSequenceStep() == EBSInitState::WaitLow);
    CHECK(ebs.initSequenceStep() == EBSInitState::CheckPressure);
    s_pressure1 = 5.0f; s_pressure2 = 5.0f;      // both tanks > 3 bar
    CHECK(ebs.initSequenceStep() == EBSInitState::WaitTS);
    CHECK(s_sdc_closed == true);                 // SDC closed on entering WaitTS
    s_asms_on = true; ts_can_publish(true);      // ASMS(A3) + fresh ECU 0x504 (#180)
    CHECK(ebs.initSequenceStep() == EBSInitState::CheckActuator1);
    CHECK(s_ebs_pin1 == false);                  // A1 fired (LOW)
    g_can_brake_over_limit.store(true);
    CHECK(ebs.initSequenceStep() == EBSInitState::WaitInterActuatorCheck);
    CHECK(s_ebs_pin1 == true);                   // A1 released (HIGH)
    s_now_ms += 6000;
    CHECK(ebs.initSequenceStep() == EBSInitState::CheckActuator2);
    CHECK(s_ebs_pin2 == false);                  // A2 fired (LOW)
    CHECK(ebs.initSequenceStep() == EBSInitState::Done);
    CHECK(s_ebs_pin2 == true);                   // A2 released (HIGH)
    CHECK(ebs.ASBChecksOK() == true);
}

static void test_ebs_pressure_failure(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    ebs.initSequenceStep();                               // Start -> WaitLow
    ebs.initSequenceStep();                               // WaitLow -> CheckPressure
    s_pressure1 = 0.5f; s_pressure2 = 0.5f;               // below the 3 bar threshold
    CHECK(ebs.initSequenceStep() == EBSInitState::Failed);
}
#else  /* BENCH_STUB_EBS_INIT */
/* Stub path: the first step jumps straight to Done so the car can arm without
 * the EBS pneumatics rig. ASBChecksOK still gates on tank pressure, so an empty
 * system does not falsely pass. (The real sequence is covered when stub=0.) */
static void test_ebs_stub_short_circuits(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    CHECK(ebs.getInitState() == EBSInitState::Start);
    CHECK(ebs.initSequenceStep() == EBSInitState::Done);
    s_pressure1 = 0.5f; s_pressure2 = 0.5f;   // tanks below threshold
    CHECK(ebs.ASBChecksOK() == false);        // still gated on pressure
    s_pressure1 = 5.0f; s_pressure2 = 5.0f;   // tanks OK (> 3 bar)
    CHECK(ebs.ASBChecksOK() == true);
}
#endif /* BENCH_STUB_EBS_INIT */

static void test_ebs_checks(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    /* checkStoragePressures: strictly > 3.0 on BOTH tanks. */
    s_pressure1 = 3.0f; s_pressure2 = 4.0f; CHECK(ebs.checkStoragePressures() == false);
    s_pressure1 = 3.1f; s_pressure2 = 3.1f; CHECK(ebs.checkStoragePressures() == true);
    s_pressure1 = 4.0f; s_pressure2 = 2.9f; CHECK(ebs.checkStoragePressures() == false);
    /* checkBrakeLinePressure: mirrors the ECU 0x505 over-limit verdict. */
    g_can_brake_over_limit.store(false); CHECK(ebs.checkBrakeLinePressure() == false);
    g_can_brake_over_limit.store(true);  CHECK(ebs.checkBrakeLinePressure() == true);
    /* SafeManual: both tanks < 0.1 (empty). */
    s_pressure1 = 0.05f; s_pressure2 = 0.05f; CHECK(ebs.SafeManual() == true);
    s_pressure1 = 0.2f;  s_pressure2 = 0.05f; CHECK(ebs.SafeManual() == false);
    /* activateEBS drives both actuators LOW (LOW = fire, fail-safe);
     * deactivateEBS drives them HIGH (release); is_ebs_active reads LOW. */
    ebs.deactivateEBS(); CHECK(hardware_io_is_ebs_active() == false);
    ebs.activateEBS();   CHECK(hardware_io_is_ebs_active() == true);
}

// ---------------------------------------------------------------------
// AS state machine transitions
// ---------------------------------------------------------------------
static void test_sm_starts_off(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    sm.update();
    CHECK(sm.getState() == ASState::OFF);
}

static void test_sm_mission_zero_selected(void)
{
    /* Mission 0 is a valid 0-based AMI index (feat/18 wire contract) — it
     * must count as "selected". Regression for the sentinel that used to
     * be 0 and swallowed it. */
    reset_world();
    StateManager& sm = StateManager::getInstance();
    g_can_mission_id.store(0);
    g_set_mission_ready.store(true);
    sm.update();
    CHECK(sm.getSignals().mission_selected == true);

    /* And the reset sentinel (-1) must NOT count as selected. */
    g_can_mission_id.store(-1);
    sm.update();
    CHECK(sm.getSignals().mission_selected == false);
}

static void test_sm_preconditions_without_ebs_done_is_off(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    set_drive_preconditions();
    g_can_brake_over_limit.store(true);
    /* EBS not driven to Done -> abs_checks_ok false -> still OFF. */
    sm.update();
    CHECK(sm.getState() == ASState::OFF);
}

static void test_sm_ready(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    drive_ebs_to_done(EbsManager::getInstance());
    set_drive_preconditions();
    g_can_brake_over_limit.store(true);   // brakes engaged
    g_can_r2d.store(false);             // no R2D yet
    sm.update();
    CHECK(sm.getState() == ASState::READY);
}

static void test_sm_driving(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    drive_ebs_to_done(EbsManager::getInstance());
    set_drive_preconditions();
    g_can_brake_over_limit.store(true);
    g_can_r2d.store(true);              // R2D -> DRIVING
    sm.update();
    CHECK(sm.getState() == ASState::DRIVING);
}

/* ---- TS now comes from the ECU on CAN 0x504, not the TSMS pin (#180) -----
 * The whole point of the change is that silence must read as "TS off". These
 * pin the fail-safe so nobody re-latches a stale TS by accident. */

/* A fresh 0x504 saying "TS down" -> not ready, even with everything else met. */
static void test_sm_ts_can_false_is_off(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    drive_ebs_to_done(EbsManager::getInstance());
    set_drive_preconditions();
    g_can_brake_over_limit.store(true);
    ts_can_publish(false);                // ECU: precharge not complete / AIRs open
    sm.update();
    CHECK(sm.getSignals().ts_active == false);
    CHECK(sm.getState() == ASState::OFF);
}

/* THE regression this change exists for: the ECU goes quiet mid-run. The old
 * pin-based read could not see this at all (the TSMS stays physically closed
 * while the AMS opens the AIRs), so the car kept believing the TS was live.
 * A stale 0x504 must now read as TS-off — which trips Emergency from
 * READY/DRIVING in as_transition. */
static void test_sm_ts_stale_is_off(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    drive_ebs_to_done(EbsManager::getInstance());
    set_drive_preconditions();
    g_can_brake_over_limit.store(true);
    sm.update();
    CHECK(sm.getSignals().ts_active == true);    // fresh: TS live

    /* ECU stops transmitting. The atomic still holds `true` — only the age
     * tells us nobody is home. */
    s_now_ms += TS_ACTIVE_STALE_MS;              // exactly at the window -> stale
    sm.update();
    CHECK(g_can_ts_active.load() == true);       // the latched value is UNCHANGED...
    CHECK(sm.getSignals().ts_active == false);   // ...but we correctly read TS off
    CHECK(sm.getState() == ASState::OFF);
}

/* Never received at all (no ECU on the bus) -> TS off, not "assume live". */
static void test_sm_ts_never_received_is_off(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    sm.update();
    CHECK(sm.getSignals().ts_active == false);
}

/* Just inside the window still counts as fresh (no off-by-one that would make
 * a healthy 100 ms cyclic flicker). */
static void test_sm_ts_just_fresh_is_on(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    ts_can_publish(true);
    s_now_ms += (TS_ACTIVE_STALE_MS - 1u);
    sm.update();
    CHECK(sm.getSignals().ts_active == true);
}

static void test_sm_preconditions_but_no_brakes_no_r2d_is_off(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    drive_ebs_to_done(EbsManager::getInstance());
    set_drive_preconditions();
    g_can_brake_over_limit.store(false);  // brakes NOT engaged
    g_can_r2d.store(false);            // and no R2D
    sm.update();
    CHECK(sm.getState() == ASState::OFF);
}

static void test_sm_emergency_on_ebs_active(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    EbsManager& ebs = EbsManager::getInstance();
    drive_ebs_to_done(ebs);
    ebs.activateEBS();                 // EBS fired, pins LOW
    g_finished_cmd.store(false);       // mission not finished
    sm.update();
    CHECK(sm.getState() == ASState::EMERGENCY);
}

static void test_sm_emergency_when_finished_but_moving(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    EbsManager& ebs = EbsManager::getInstance();
    drive_ebs_to_done(ebs);
    ebs.activateEBS();
    g_finished_cmd.store(true);
    g_imu_vehicle_standstill.store(false);   // still moving
    sm.update();
    CHECK(sm.getState() == ASState::EMERGENCY);
}

static void test_sm_finished(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    EbsManager& ebs = EbsManager::getInstance();
    drive_ebs_to_done(ebs);
    ebs.activateEBS();
    g_finished_cmd.store(true);
    g_imu_vehicle_standstill.store(true);
    g_can_sdc_res_open.store(false);         // SDC not open at RES
    sm.update();
    CHECK(sm.getState() == ASState::FINISHED);
}

static void test_sm_finished_but_sdc_open_is_emergency(void)
{
    reset_world();
    StateManager& sm = StateManager::getInstance();
    EbsManager& ebs = EbsManager::getInstance();
    drive_ebs_to_done(ebs);
    ebs.activateEBS();
    g_finished_cmd.store(true);
    g_imu_vehicle_standstill.store(true);
    g_can_sdc_res_open.store(true);          // SDC open -> EMERGENCY, not FINISHED
    sm.update();
    CHECK(sm.getState() == ASState::EMERGENCY);
}

int main(void)
{
    test_assi_codes();

#if !BENCH_STUB_EBS_INIT
    test_ebs_happy_path();
    test_ebs_pressure_failure();
#else
    test_ebs_stub_short_circuits();
#endif
    test_ebs_checks();

    test_sm_starts_off();
    test_sm_mission_zero_selected();
    test_sm_preconditions_without_ebs_done_is_off();
    test_sm_ready();
    test_sm_driving();
    test_sm_ts_can_false_is_off();
    test_sm_ts_stale_is_off();
    test_sm_ts_never_received_is_off();
    test_sm_ts_just_fresh_is_on();
    test_sm_preconditions_but_no_brakes_no_r2d_is_off();
    test_sm_emergency_on_ebs_active();
    test_sm_emergency_when_finished_but_moving();
    test_sm_finished();
    test_sm_finished_but_sdc_open_is_emergency();

    std::printf("\nstate_machine: %d checks, %d failures\n",
                g_checks, g_failures);
    if (g_failures == 0) {
        std::printf("All tests green.\n");
        return 0;
    }
    return 1;
}
