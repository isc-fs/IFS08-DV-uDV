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
#include "bench_stubs.h"   /* toggles must be 0 here — suite tests REAL logic */
#include "can_globals.h"
#include "ros_globals.h"

/* Fail the build if someone commits enabled bench stubs: this suite (and the
 * car) must always see the real EBS init + pipeline gating. The bench flips
 * the toggles only on a throwaway bench/ branch. */
static_assert(BENCH_STUB_EBS_INIT == 0,
              "BENCH_STUB_EBS_INIT must be 0 on dev (bench_stubs.h)");
static_assert(BENCH_STUB_DVPC == 0,
              "BENCH_STUB_DVPC must be 0 on dev (bench_stubs.h)");

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
    s_sdc_closed = false; s_now_ms = 0;

    g_can_ts_active.store(false);
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

/* Step the EBS init FSM all the way to Done with nominal hardware. */
static void drive_ebs_to_done(EbsManager& ebs)
{
    s_sdc_ready = true;  ebs.initSequenceStep();   // Start -> WaitLow
    s_sdc_ready = false; ebs.initSequenceStep();   // WaitLow -> CheckPressure
    s_pressure1 = 2.0f; s_pressure2 = 2.0f;
    ebs.initSequenceStep();                        // CheckPressure -> WaitTS (closes SDC)
    s_asms_on = true; s_tsms_on = true;            // TS = ASMS(A3) && TSMS(A6)
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
    s_tsms_on = true;                              // TS = ASMS(A3) && TSMS(A6)
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
// ---------------------------------------------------------------------
static void test_ebs_happy_path(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    CHECK(ebs.getInitState() == EBSInitState::Start);

    s_sdc_ready = true;  CHECK(ebs.initSequenceStep() == EBSInitState::WaitLow);
    s_sdc_ready = false; CHECK(ebs.initSequenceStep() == EBSInitState::CheckPressure);
    s_pressure1 = 2.0f; s_pressure2 = 2.0f;
    CHECK(ebs.initSequenceStep() == EBSInitState::WaitTS);
    CHECK(s_sdc_closed == true);                 // SDC closed on entering WaitTS
    s_asms_on = true; s_tsms_on = true;          // TS = ASMS(A3) && TSMS(A6)
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

static void test_ebs_waitlow_timeout(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    s_sdc_ready = true; ebs.initSequenceStep();          // -> WaitLow
    /* SDC never drops; advance past the 5 s timeout. */
    s_now_ms += 5001;
    CHECK(ebs.initSequenceStep() == EBSInitState::Failed);
    CHECK(ebs.ASBChecksOK() == false);
}

static void test_ebs_pressure_failure(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    s_sdc_ready = true;  ebs.initSequenceStep();          // -> WaitLow
    s_sdc_ready = false; ebs.initSequenceStep();          // -> CheckPressure
    s_pressure1 = 0.5f; s_pressure2 = 0.5f;               // below threshold
    CHECK(ebs.initSequenceStep() == EBSInitState::Failed);
}

static void test_ebs_checks(void)
{
    reset_world();
    EbsManager& ebs = EbsManager::getInstance();
    /* checkStoragePressures: strictly > 1.0 on BOTH tanks. */
    s_pressure1 = 1.0f; s_pressure2 = 2.0f; CHECK(ebs.checkStoragePressures() == false);
    s_pressure1 = 1.1f; s_pressure2 = 1.1f; CHECK(ebs.checkStoragePressures() == true);
    s_pressure1 = 2.0f; s_pressure2 = 0.9f; CHECK(ebs.checkStoragePressures() == false);
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

    test_ebs_happy_path();
    test_ebs_waitlow_timeout();
    test_ebs_pressure_failure();
    test_ebs_checks();

    test_sm_starts_off();
    test_sm_mission_zero_selected();
    test_sm_preconditions_without_ebs_done_is_off();
    test_sm_ready();
    test_sm_driving();
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
