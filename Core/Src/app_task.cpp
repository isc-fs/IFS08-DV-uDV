/**
 * @file    app_task.cpp
 * @brief   Application task implementation - Main state machine coordinator
 */

#include "app_task.h"

#include <cstdint>
#include <cstring>

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "cmsis_os.h"
    #include "hardware_io.h"
    #include "safety_monitor.h"  /* IWDG safety supervisor: heartbeat/arm */
    #include "assi_task.h"       /* ASSI mode API (UART/Arduino LED bridge) */
    #include "bench_stubs.h"     /* bench stub toggles (all 0 on dev) */

    /* /debug queue (freertos.c) — used to announce active bench stubs. */
    extern osMessageQueueId_t debugQueueHandle;
}
#include <cstdio>              /* snprintf for the stub announcement */

#include "state_manager.hpp"
#include "as_transition.hpp"   /* pure AS transition decision (host-tested) */
#include "as_actuation.hpp"    /* pure safe-state actuation decision (host-tested) */
#include "ebs_manager.hpp"
#include "ros_task_commands.h"
#include "can_interface.hpp"
#include "can_task.h"
#include "ros_globals.h"
#include "can_globals.h"
#include "mission.h"           /* per-mission dispatch (Mission vtable, registry) */

extern "C" {
    #include "dv_interface.h"   /* dv/status + ctrl/cmd contract bytes + timings */
}

/* ------------------------------------------------------------------------
 * Mission dispatch
 *
 * The AMI board sends the 0-based mission menu index on CAN 0x503 byte[0]
 * (see IFS08-DV_AMI Core/Src/ami.c `missions[]`). app_task resolves the code
 * to a `const Mission*` via mission_for_code() and drives it through the
 * Mission vtable (mission.h). Each mission's control law lives in its own TU
 * (mission_inspection.cpp / mission_pipeline.cpp / ...). Pipeline missions
 * (accel/skidpad/autocross/trackdrive) are driven by the DV pipeline over
 * /ctrl/cmd; standalone ones (inspection, EBS test) run open-loop.
 *
 * A mission body is PURE: it consumes a MissionCtx snapshot and returns a
 * MissionCommand; this loop builds the snapshot (below) and applies the
 * command via Can::, so no mission touches the CAN driver directly.
 * ------------------------------------------------------------------------ */
static MissionCtx app_build_mission_ctx(uint32_t now_ms, uint32_t elapsed_ms)
{
    MissionCtx ctx;
    ctx.now_ms             = now_ms;
    ctx.mission_elapsed_ms = elapsed_ms;
    /* /ctrl/cmd freshness — the same window the old inline pipeline branch
     * used, so a dropped link zeroes actuation within DV_CTRL_CMD_STALE_MS. */
    uint32_t stamp = g_ctrl_cmd_stamp_ms.load();
    ctx.ctrl_cmd_fresh = (stamp != 0u) &&
                         ((now_ms - stamp) < DV_CTRL_CMD_STALE_MS);
    ctx.ctrl_accel = g_accel_cmd.load();
    ctx.ctrl_steer = g_steer_cmd.load();
    /* ECU DV R2D confirm (0x511). A requests_r2d mission gates its torque on
     * this — no drivetrain command until the ECU is actually in DV mode. */
    ctx.r2d_confirmed = g_can_r2d.load();
    return ctx;
}

/* ECU torque stream (0x507) is a 20 ms cyclic per the ECU contract — pace
 * every torque/steer TX to it (the control loop itself runs ~1 kHz and
 * would otherwise flood the shared ACU bus). */
static constexpr uint32_t TORQUE_TX_PERIOD_MS      = 20u;
/* Pipeline steering: /ctrl/cmd angular.z is normalised [-1..1], where +/-1 is
 * the pipeline's max road-wheel angle (control_node max_steer_deg). The DV
 * steering controller (0x521) takes an absolute angle in degrees in its own
 * output-shaft frame ("grados" -> Ir_a_Grados in IFS08-DV-STEERING). The column
 * mechanical limit is +/-100 deg, so grados is clamped there. Convert the
 * normalised road-wheel command through the real IFS-08 steering kinematics:
 *
 *   grados = norm * MAX_STEER_ROADWHEEL_DEG * STEERING_RATIO * MOTOR_TO_COLUMN
 *
 * STEERING_RATIO (volante:rueda) from the rack (87.9 mm/rev): the two candidate
 * geometries are 72.1 deg wheel = 5.0:1 and 84.5 deg wheel = 4.3:1. We take the
 * MORE CONSERVATIVE 5.0:1 (smaller road-wheel authority: 18.2 vs 21.1 deg ceiling).
 * MOTOR_TO_COLUMN = 1.1 (column:motor 1:1.1) is the stage between DV-STEERING's
 * output shaft and the column that its internal REDUCTORA=20 does NOT model;
 * if that 1.1 is already folded into DV-STEERING's REDUCTORA, set it to 1.0.
 *
 * Replaces the old norm * STEER_FULL_LOCK_DEG(65) placeholder (#71): 65 baked in
 * a column:wheel ratio of only 65/28 = 2.3 vs the real ~5.0, so the wheels
 * turned less than half the commanded angle and the LWS (column) diverged from
 * the road-wheel command on the bench.
 *
 * Closed set: the +/-100 column clamp / effective ratio E=5.5 gives a road-wheel
 * ceiling of 100/5.5 = ~18.2 deg. MAX_STEER_ROADWHEEL_DEG is set to that so
 * norm=1 lands exactly on the clamp (linear across the whole range, no early
 * saturation, reaches the mechanical limit at full command).
 *
 * Sign convention (ROS +z = CCW/left) is handled in IFS08-DV-STEERING, not here.
 *
 * OPEN (uDV team, #71):
 *   - pipeline: control_node max_steer_deg MUST be set to the SAME 18.2 (not 28)
 *     or norm's meaning diverges — the car can't exceed ~18.2 deg road-wheel;
 *   - if MOTOR_TO_COLUMN(1.1) is already folded into DV-STEERING's REDUCTORA,
 *     E=5.0 -> ceiling 20.0, so set both MAX_STEER_ROADWHEEL_DEG and the
 *     pipeline max_steer_deg to 20.0 instead. */
static constexpr float    MAX_STEER_ROADWHEEL_DEG  = 18.2f;  /* = clamp/E (100/5.5); MUST match control_node max_steer_deg */
static constexpr float    STEERING_RATIO           = 5.0f;   /* column : road-wheel (72.1 deg) */
static constexpr float    MOTOR_TO_COLUMN          = 1.1f;   /* DV-STEERING output : column (E = 5.0*1.1 = 5.5) */
static constexpr float    STEER_GRADOS_MAX_DEG     = 100.0f; /* column mechanical limit +/-100 deg */

/* Normalised road-wheel command [-1..1] -> DV-STEERING "grados" (0x521). */
static inline float steer_norm_to_grados(float norm)
{
    float g = norm * MAX_STEER_ROADWHEEL_DEG * STEERING_RATIO * MOTOR_TO_COLUMN;
    if (g >  STEER_GRADOS_MAX_DEG) g =  STEER_GRADOS_MAX_DEG;
    if (g < -STEER_GRADOS_MAX_DEG) g = -STEER_GRADOS_MAX_DEG;
    return g;
}
/* DV ready-to-drive request (0x510) retry period until the ECU confirms
 * on 0x511 (acyclic; the ECU latches the edge for the drive cycle). */
static constexpr uint32_t R2D_REQ_PERIOD_MS        = 100u;
/* Mandated minimum time in AS READY before a RES GO may be honoured
 * (FS-Rules AS-Ready dwell). Gates READY->DRIVING via as_in.ready_dwell_elapsed. */
static constexpr uint32_t READY_DWELL_MS           = 5000u;
/* ASB low-pressure debounce: the tanks must read below 3 bar CONTINUOUSLY for
 * this long before the AS transition is told the stored energy is gone.
 *
 * Needed because the trip is expensive and one-way: !asb_pressure_ok while
 * armed raises Emergency, which LATCHES until ASMS-off — so a single spurious
 * low sample ends the run. The pressure signal is DC behind a 10k/1k divider
 * and the loop samples it at ~1 kHz, so real noise is uncorrelated sample to
 * sample while a genuine leak/vent lasts far longer than this window. (ADC3
 * channel-to-channel crosstalk on exactly these taps is not hypothetical — see
 * the long-sample-time fix in hardware_io.c.)
 *
 * 100 ms is ~100 consecutive low samples: uncrossable by noise, and far inside
 * the FS-Rules T11.9.4 500 ms detect-and-safe bound. It costs nothing on a real
 * loss — the tanks cannot refill in 100 ms. */
static constexpr uint32_t ASB_LOW_DEBOUNCE_MS       = 100u;

/* Apply a mission's actuation intent to the CAN bus (the only place a mission
 * result reaches hardware). Two independent actuator paths:
 *  - send_steer_angle: absolute 0x521 steering angle at the mission's own
 *    cadence (inspection sweep / EBS-test hold).
 *  - send_accel / send_steer: the pipeline drive, paced by drive_tx_due to the
 *    ECU's 20 ms torque cycle. send_accel -> 0x507 torque; send_steer -> 0x521
 *    angle = steer_norm_to_grados(norm). They are separate so a stale link can
 *    zero the torque WITHOUT commanding steering — a 0x521 zero would snap the
 *    wheel to center mid-corner (the normalised 0x508 steer frame was retired). */
static void app_apply_mission_command(const MissionCommand& cmd, bool drive_tx_due)
{
    if (cmd.send_steer_angle)
    {
        Can::sendSteeringAngle(cmd.steer_angle_deg);
    }
    if (drive_tx_due)
    {
        /* Keep the 0x507 torque stream alive at the ECU's 20 ms cadence in
         * DRIVING even when the mission commands no torque (inspection): send
         * the commanded accel, or an explicit 0. That holds the ECU's DV mode
         * with a fresh zero rather than letting the stream go stale (the ECU
         * treats stale as 0 anyway, but an explicit 0 keeps dv_fresh true). */
        Can::sendAccel(cmd.send_accel ? cmd.accel_norm : 0.0f);
        if (cmd.send_steer)
        {
            Can::sendSteeringAngle(steer_norm_to_grados(cmd.steer_norm));
        }
    }
}

/**
 * @brief Reset ROS globals to default state
 */
static void reset_ros_globals(void)
{
    g_mission_going_cmd.store(false);
    g_accel_cmd.store(0.0f);
    g_steer_cmd.store(0.0f);
    g_emergency_cmd.store(false);
    g_finished_cmd.store(false);
    g_set_mission_in_progress.store(false);
    g_set_mission_ready.store(false);

    // Pipeline interface: forget the last dv/status + command so a reset
    // can't leave a stale "READY"/throttle latched across runs.
    g_dv_status.store(DV_STATUS_IDLE);
    g_dv_status_stamp_ms.store(0);
    g_ctrl_cmd_stamp_ms.store(0);
}

/**
 * @brief Reset CAN globals to default state
 */
static void reset_can_globals(void)
{
    g_can_listen_go.store(false);
    g_can_mission_id.store(-1);   /* -1 = none; 0 is a valid mission */
    g_can_r2d.store(false);  /* (g_can_go was redundant with this) */
    g_imu_vehicle_standstill.store(true);
    g_steer_motor_state.store(ESTADO_MOTOR_OFF);  /* clear a stale steering fault */
}

static void reset_state_telemetry(void)
{
    g_telemetry_as_state.store(static_cast<uint8_t>(ASState::OFF));
    g_telemetry_ebs_init_state.store(static_cast<uint8_t>(EBSInitState::Start));
    g_telemetry_asms_on.store(false);
    g_telemetry_ts_active.store(false);
    g_telemetry_sdc_res_open.store(false);
    g_telemetry_brakes_engaged.store(false);
    g_telemetry_r2d.store(false);
    g_telemetry_vehicle_standstill.store(true);
    g_telemetry_mission_selected.store(false);
    g_telemetry_mission_finished.store(false);
    g_telemetry_abs_checks_ok.store(false);
    g_telemetry_ebs_activated.store(false);
}

static void sync_state_telemetry(const StateManager& state_mgr,
                                 const EbsManager& ebs,
                                 ASState as_state)
{
    const StateManagerSignals& signals = state_mgr.getSignals();

    g_telemetry_as_state.store(static_cast<uint8_t>(as_state));
    g_telemetry_ebs_init_state.store(static_cast<uint8_t>(ebs.getInitState()));
    g_telemetry_asms_on.store(signals.asms_on);
    g_telemetry_ts_active.store(signals.ts_active);
    g_telemetry_sdc_res_open.store(signals.sdc_res_open);
    g_telemetry_brakes_engaged.store(signals.brakes_engaged);
    g_telemetry_r2d.store(signals.r2d);
    g_telemetry_vehicle_standstill.store(signals.vehicle_standstill);
    g_telemetry_mission_selected.store(signals.mission_selected);
    g_telemetry_mission_finished.store(signals.mission_finished);
    g_telemetry_abs_checks_ok.store(signals.abs_checks_ok);
    g_telemetry_ebs_activated.store(signals.ebs_activated);
    /* Sample the EBS air-tank pressures here so AppTask stays the SOLE reader of
     * hadc3 (the pit-diag/ros publishers only touch the atomics). Real ADC even
     * under BENCH_STUB_EBS_SENSORS, so the bench sees true tank pressure. */
    g_telemetry_ebs_pressure1_bar.store(hardware_io_read_actuator1_storage_pressure());
    g_telemetry_ebs_pressure2_bar.store(hardware_io_read_actuator2_storage_pressure());
}

/**
 * @brief Reset all systems
 */
static void reset_all(void)
{
    // Cancel ongoing ROS mission if any
    if (g_set_mission_in_progress.load())
    {
        send_cancel_set_mission_command();
    }

    if (g_mission_going_cmd.load())
    {
        send_cancel_mission_command();
    }

    // Reset global states
    reset_ros_globals();
    reset_can_globals();
    reset_state_telemetry();

    // Reset EBS and StateManager
    EbsManager::getInstance().reset();
    StateManager::getInstance().reset();

    // Clear task queues
    if (g_ros_cmd_queue != NULL)
    {
        xQueueReset(g_ros_cmd_queue);
    }

    g_reset_cmd.store(false);
}

/**
 * @brief C wrapper function that FreeRTOS calls
 */
extern "C" void StartAppTask(void *argument)
{
    (void)argument;  // Unused parameter

    EbsManager& ebs = EbsManager::getInstance();
    StateManager& state_mgr = StateManager::getInstance();

    EBSInitState ebs_state = EBSInitState::Start;
    ASState as_state = ASState::OFF;
    ASState last_as_state = ASState::OFF;  // Track last state for ASSI updates
    uint32_t ready_start_time = 0;
    /* Tick the ASB tanks first read below 3 bar; 0 = currently reading ok.
     * Drives the ASB_LOW_DEBOUNCE_MS window (see the read in the loop). */
    uint32_t asb_low_since_ms = 0;
    int last_mission_id = -1;
    bool set_mission_sent = false;
    bool start_mission_sent = false;   /* CAN start-mission fallback (dev) */

    /* Mission run state. mission_time stamps the current DRIVING run so the
     * mission clock (MissionCtx.mission_elapsed_ms) is relative to the GO edge;
     * mission_complete latches a standalone mission's self-finish -> FINISHED.
     * (The steering rate-limit now lives inside the mission, not here.)
     *
     * active_mission is the mission BOUND to the current DRIVING run: captured
     * from the selected mission on the GO edge and used for all of on_enter /
     * on_tick / is_complete / on_exit until the run ends. Binding the run means
     * a mid-run change of the selected code (a stray/corrupt 0x503 — the AMI
     * latches one mission per power cycle, so it can't happen normally) can
     * neither swap the running mission's actuation nor null it mid-drive (which
     * would leave DRIVING with brakes released and no command). It is non-null
     * for exactly the span as_state == DRIVING (set on the GO edge, which the
     * mission_valid gate only allows for a real mission; cleared when leaving
     * DRIVING). */
    uint32_t       mission_time     = 0;
    bool           mission_complete = false;
    const Mission* active_mission   = nullptr;

    // Send initial OFF status via CAN
    Can::sendAssiStatus(StateManager::getAssiStatusCode(as_state));

    /* Let StartCanTask bring up FDCAN3 before we emit frames in earnest
     * (the initial ASSI above is best-effort, dropped if CAN isn't up).
     * There is no ROS command queue on this branch: the action/command
     * layer (ros_interface / dv_msgs) is intentionally NOT integrated
     * yet, so the mission senders are stubs (see ros_task_commands). */
    osDelay(100);

    /* The IWDG is owned by the safety supervisor task. Register this
     * state-machine loop as a monitored liveness source so a stall here
     * trips the watchdog emergency — this replaces the old TIM3 timer. */
    safety_arm(SAFETY_TASK_APP);

    /* ASSI LEDs are rendered by assi_task.c (UART -> Arduino bridge); this
     * task only publishes the AS mode via assi_set_mode() below. The task
     * boots in AS_MODE_OFF, the correct AS Off indication (T14.9.1). */

    /* A build with any bench stub toggled on (bench_stubs.h) announces
     * itself on /debug once at boot — a stubbed image must never
     * masquerade as a flight build. Folds away when all toggles are 0. */
    if (BENCH_STUB_EBS_INIT || BENCH_STUB_EBS_SENSORS || BENCH_STUB_SDC ||
        BENCH_STUB_DVPC || BENCH_STUB_RES || BENCH_STUB_STEERING ||
        BENCH_STUB_IMU_ROS || BENCH_STUB_TS)
    {
        char stub_buf[128];   /* debugQueue element size */
        snprintf(stub_buf, sizeof(stub_buf),
                 "debug: BENCH STUBS COMPILED IN (ebs_init=%d ebs_sensors=%d sdc=%d dvpc=%d res=%d steering=%d imu_ros=%d ts=%d)",
                 BENCH_STUB_EBS_INIT, BENCH_STUB_EBS_SENSORS, BENCH_STUB_SDC,
                 BENCH_STUB_DVPC, BENCH_STUB_RES, BENCH_STUB_STEERING,
                 BENCH_STUB_IMU_ROS, BENCH_STUB_TS);
        (void)osMessageQueuePut(debugQueueHandle, &stub_buf, 0, 0);
    }

    // Main control loop
    while (1)
    {
        // State machine dispatcher REQUIREMENTS:

        //1. cuando asms esta off siempre retornar a la AS_off
        //2. Si asms on, ts on y RES ok (es decir res 0 mira la funcion can_c_get_res_status()). transicion a AS_ready
        //3. Si RES E-stop en cualquier momento transicionar a AS_emergency
        //4. EBS engaged (brakes on, D1/D2 LOW = fire) in every AS state except
        //   DRIVING; released (D1/D2 HIGH) in DRIVING (autonomous) and in
        //   manual/ASMS-off handling. The AS SDC is opened (D4 LOW) in the
        //   terminal safe states FINISHED / EMERGENCY (TS off / TSAL green +
        //   fail-safe EBS). IMPLEMENTED below: OFF (after init) / READY /
        //   FINISHED / EMERGENCY -> ebs.activateEBS(); DRIVING & manual ->
        //   ebs.deactivateEBS().
        //5. transicinar de AS_ready a AS_driving cuando se reciba el comando de start del RES
        //6. transicionar a emergency si el RES esta bien y el TS pasa a esta off.
        //7. configura para que la mision por defecto sea INSPECTION
        //8. AS_finished is reached when the mission ends: a pipeline mission on
        //   /dv/status FINISHED, a standalone mission (inspection) on its own
        //   timer (mission_complete). See as_transition.hpp.
        //9. La unica forma de salir de AS_emergency es pasar por asms off.

        // Check for reset command (issued by external supervisor)
        if (g_reset_cmd.load())
        {
            set_mission_sent = false;
            start_mission_sent = false;
            last_mission_id = -1;
            reset_all();
        }

        // Update state machine with all current signals
        state_mgr.update();
        ASState previous_as_state = as_state;
        bool asms_on = hardware_io_read_asms_on();
        bool ts_on = state_mgr.getSignals().ts_active;
        uint32_t now_ms = osKernelGetTickCount();
        int32_t res_status = can_c_get_res_status(now_ms, 150U);
        /* "go" also means the RES link is healthy (status 2 = received, go
         * asserted, no e-stop) — otherwise a GO held before arming would
         * block the READY condition forever (ported from fix/19). */
        bool res_ok = (res_status == 0) || (res_status == 2);
        bool res_go = (res_status == 2);
        bool res_estop = (res_status == 1);

        /* Steering stepper-driver grave fault (byte 5 of 0x528). EMERGENCIA is
         * an absolute cut-off requiring a physical reset — a dead steering
         * motor means the car can't steer, so treat it like an RES e-stop. */
        bool steer_emergency =
            (!BENCH_STUB_STEERING) &&
            (g_steer_motor_state.load() == ESTADO_MOTOR_EMERGENCIA);

        // --- Pipeline (DVPC) status, read level-triggered off /dv/status ---
        // The uDV owns the AS state machine; mission_control only answers
        // with this byte. It is also the DVPC liveness heartbeat: a stale
        // /dv/status while driving means the pipeline died -> safe state.
        uint8_t  dv_status     = g_dv_status.load();
        bool     dv_seen       = (g_dv_status_stamp_ms.load() != 0u);
        bool     dv_fresh      = dv_seen &&
                                 ((now_ms - g_dv_status_stamp_ms.load()) < DV_STATUS_STALE_MS);

        /* Selected mission (0-based AMI index on 0x503). NO default: when none
         * has been received (g_can_mission_id < 0) the mission stays UNRESOLVED
         * (nullptr) so the transition's mission_valid gate REFUSES GO — the car
         * must never enter DRIVING without an explicitly selected mission. (The
         * old "default to Inspection when none" silently made GO drivable with
         * no selection.) Read here — before the transition — so the state
         * machine can gate the GO / finish rules on the mission type. */
        int current_mission_id = g_can_mission_id.load();
        /* Resolve the mission ONCE per tick. nullptr = no selection / unknown /
         * non-driving code (SHUTDOWN, aux): the transition refuses GO
         * (mission_valid) so it can never enter DRIVING. mission_for_code(<0)
         * returns nullptr. Used below for the transition inputs, the GO/FINISHED
         * edges and the per-tick actuation dispatch. */
        const Mission* mission = mission_for_code(current_mission_id);

        // Decide the next AS state via the pure, host-tested transition (see
        // as_transition.hpp). dv_* fold in freshness: the "go" gate needs a
        // fresh DV READY, and a stale /dv/status while driving trips Emergency.
        // OFF->READY additionally requires the EBS init sequence Done (dev
        // 9395936): the init FSM only advances while OFF, so arming earlier
        // would strand it and ASBChecksOK could never pass.
        AsInputs as_in;
        as_in.asms_on       = asms_on;
        as_in.ts_on         = ts_on;
        as_in.res_estop     = res_estop;
        as_in.res_go        = res_go;
        as_in.res_ok        = res_ok;
        as_in.steer_emergency = steer_emergency;
        as_in.ebs_init_done = (ebs.getInitState() == EBSInitState::Done);
        /* ASB stored energy, debounced (see ASB_LOW_DEBOUNCE_MS). Stays "ok"
         * until the tanks have read low CONTINUOUSLY for the window; any single
         * good sample re-arms it. One-way in the dangerous direction only: once
         * the window elapses the transition raises Emergency, which latches. */
        {
            const bool asb_raw_ok = state_mgr.getSignals().asb_pressure_ok;
            if (asb_raw_ok)                 asb_low_since_ms = 0u;
            else if (asb_low_since_ms == 0u) asb_low_since_ms = now_ms;
            as_in.asb_pressure_ok =
                asb_raw_ok ||
                ((uint32_t)(now_ms - asb_low_since_ms) < ASB_LOW_DEBOUNCE_MS);
        }
        as_in.dv_fresh      = dv_fresh;
        as_in.dv_ready     = dv_fresh && (dv_status == DV_STATUS_READY);
        as_in.dv_finished  = dv_fresh && (dv_status == DV_STATUS_FINISHED);
        as_in.dv_emergency = dv_fresh && (dv_status == DV_STATUS_EMERGENCY ||
                                          dv_status == DV_STATUS_FAILED);
        /* While already DRIVING the run is bound to active_mission (captured at
         * GO), so the pipeline/heartbeat rules follow the mission that is
         * actually running, immune to a mid-run code change. Before DRIVING the
         * selected mission drives the entry gate. mission_valid always reflects
         * the selected code (it only gates the READY->DRIVING entry). */
        const Mission* gate_mission =
            (previous_as_state == ASState::DRIVING) ? active_mission : mission;
        as_in.mission_needs_pipeline = (gate_mission != nullptr) && gate_mission->needs_pipeline;
        as_in.mission_complete       = mission_complete;
        as_in.mission_valid          = (mission != nullptr);
        /* Standstill (ECU 0x506 rpm) — lets an end-of-mission FINISH win over the
         * ASB low-pressure trip once the car is actually stopped (see
         * as_transition.hpp finishing_at_standstill). */
        as_in.vehicle_standstill     = state_mgr.getSignals().vehicle_standstill;
        /* FS-Rules AS-Ready dwell: a GO is only honoured after the car has been
         * in READY for >= READY_DWELL_MS. ready_start_time is stamped on the
         * READY-entry tick below (0 while not in READY), so this reads false on
         * the entry tick and until the dwell elapses — refusing an early GO. */
        as_in.ready_dwell_elapsed    =
            (ready_start_time != 0u) &&
            ((hardware_io_now_ms() - ready_start_time) > READY_DWELL_MS);

        /* Bench stub (bench_stubs.h toggle, 0 on dev — this branch folds
         * away): no DVPC on the bench. While NO /dv/status has ever arrived,
         * fake a fresh pipeline READY so pipeline missions can arm/drive and
         * the lost-heartbeat rule stays quiet. A real pipeline (dv_seen)
         * wins permanently once it speaks. */
        if (BENCH_STUB_DVPC && !dv_seen)
        {
            as_in.dv_fresh     = true;
            as_in.dv_ready     = true;
            as_in.dv_finished  = false;
            as_in.dv_emergency = false;
        }

        as_state = as_next_state(previous_as_state, as_in);

        /* Steering-motor lifecycle + mission lifecycle, driven off the AS-state
         * edges:
         *   READY -> DRIVING (GO): energise the steering, (re)start the mission
         *   clock so the mission begins from elapsed 0, and fire mission
         *   on_enter (resets the mission's internal state).
         *   -> FINISHED: de-energise the steering and fire mission on_exit. */
        if (as_state == ASState::DRIVING && previous_as_state != ASState::DRIVING)
        {
            /* Bind the run to the selected mission. mission is non-null here: the
             * mission_valid gate only lets a real mission reach DRIVING. */
            active_mission   = mission;
            Can::sendSteeringStart();
            mission_time     = now_ms;
            mission_complete = false;
            if (active_mission != nullptr && active_mission->on_enter != nullptr)
            {
                MissionCtx ctx = app_build_mission_ctx(now_ms, 0u);
                active_mission->on_enter(&ctx);
            }
        }
        if (as_state == ASState::FINISHED && previous_as_state != ASState::FINISHED)
        {
            Can::sendSteeringStop();
            if (active_mission != nullptr && active_mission->on_exit != nullptr)
            {
                MissionCtx ctx = app_build_mission_ctx(now_ms, now_ms - mission_time);
                active_mission->on_exit(&ctx);
            }
        }
        /* Unbind the run when leaving DRIVING (to FINISHED / EMERGENCY / OFF).
         * Runs after the FINISHED on_exit above, which still needs it. */
        if (previous_as_state == ASState::DRIVING && as_state != ASState::DRIVING)
        {
            active_mission = nullptr;
        }

        sync_state_telemetry(state_mgr, ebs, as_state);

        // Send ASSI status if state changed
        if (as_state != last_as_state)
        {
            Can::sendAssiStatus(StateManager::getAssiStatusCode(as_state));
            last_as_state = as_state;
        }

        // Publish the AS state to the ASSI renderer task (assi_task.c), which
        // owns colours + flash timing over the UART/Arduino bridge (FS-Rules
        // T14.9.1). Unconditional — outside the asms_on gate — so the LEDs
        // always track the real state (e.g. go dark when ASMS drops to OFF,
        // blue steady in FINISHED).
        {
            assi_mode_t assi_mode = AS_MODE_OFF;
            switch (as_state)
            {
                case ASState::OFF:       assi_mode = AS_MODE_OFF;       break;
                case ASState::READY:     assi_mode = AS_MODE_READY;     break;
                case ASState::DRIVING:   assi_mode = AS_MODE_DRIVING;   break;
                case ASState::EMERGENCY: assi_mode = AS_MODE_EMERGENCY; break;
                case ASState::FINISHED:  assi_mode = AS_MODE_FINISHED;  break;
            }
            assi_set_mode(assi_mode);
        }

        /* Liveness beat to the safety supervisor (it owns the IWDG and
         * fires the emergency on a stall). Unconditional: the EBS-init
         * WaitLow step has its own 5 s timeout in EbsManager, so gating
         * the beat there (as the old TIM3 design did) would only
         * false-trip our 100 ms monitor during normal init. */
        safety_heartbeat(SAFETY_TASK_APP);

        /* ---- Centralised safe-state actuation (EBS + AS SDC) ----
         * The per-state release/fire + SDC-open rule is the pure, host-tested
         * as_actuation() (see as_actuation.hpp / tests/host/test_as_actuation.cpp)
         * — the single source of truth, applied here for every state including
         * manual (ASMS off). Exception: OFF drives the actuators through its EBS
         * init self-test (T15.2) until it completes; once Done/Failed OFF uses
         * the steady rule like every other state. */
        {
            const AsActuation act = as_actuation(as_state, asms_on);
            const bool off_init_running =
                asms_on && (as_state == ASState::OFF) &&
                (ebs_state != EBSInitState::Done) &&
                (ebs_state != EBSInitState::Failed);
            if (off_init_running)
            {
                ebs_state = ebs.initSequenceStep();   // ASB self-check (T15.2)
            }
            else if (act.ebs_release)
            {
                ebs.deactivateEBS();   // released: DRIVING (autonomous) or manual
            }
            else
            {
                ebs.activateEBS();     // fired: every other AS state (req #4)
            }
            if (act.sdc_open)
            {
                // Terminal safe state: open the AS SDC (D4 LOW) -> TS off / TSAL
                // green + EBS via the fail-safe path. Mirrors safety_monitor.c.
                hardware_io_set_as_close_sdc(false);
            }
            if (!asms_on)
            {
                ebs.SafeManual();   // manual handling: verify actuator tanks vented
            }
        }

        if (asms_on)
        {
            // Autonomous mode enabled

            // Reset state tracking when transitioning out of READY/DRIVING
            if (as_state != ASState::READY)
            {
                ready_start_time = 0;
                g_can_listen_go.store(false);
                start_mission_sent = false;
            }

            /* current_mission_id was resolved above (before the transition). */
            if (current_mission_id != last_mission_id)
            {
                last_mission_id = current_mission_id;
                set_mission_sent = false;
                start_mission_sent = false;
                g_set_mission_in_progress.store(false);
                g_set_mission_ready.store(false);
            }

            if (current_mission_id >= 0 && !set_mission_sent &&
                !g_set_mission_in_progress.load() && !g_set_mission_ready.load())
            {
                if (send_set_mission_command(current_mission_id))
                {
                    set_mission_sent = true;
                }
            }

            /* Pace all torque/steer TX to the ECU's 20 ms cycle (0x507
             * contract). One shared clock so DRIVING and not-DRIVING keep
             * the same steady stream cadence the ECU stale-detector needs. */
            static uint32_t last_torque_tx_ms = 0;
            bool torque_tx_due =
                ((uint32_t)(now_ms - last_torque_tx_ms) >= TORQUE_TX_PERIOD_MS);
            if (torque_tx_due)
            {
                last_torque_tx_ms = now_ms;
            }

            /* Zero torque when not driving (safety). No steering command
             * outside DRIVING: the steering motor is stopped (0x520), and a
             * 0x521 frame would command the wheel to CENTER, not "hold". */
            if (as_state != ASState::DRIVING && torque_tx_due)
            {
                Can::sendAccel(0.0f);
            }

            /* DV ready-to-drive handshake (0x510 -> 0x511): while DRIVING a
             * mission that requests_r2d, without the ECU's confirm (g_can_r2d,
             * from 0x511), re-request periodically. The ECU only honours it
             * while its own brake sensor confirms the EBS holding (0x505 verdict)
             * and latches the edge for the drive cycle — once confirmed,
             * requesting stops. Gated on requests_r2d (not needs_pipeline) so
             * inspection also enters DV mode (it commands zero torque but still
             * drives the handshake); EBS-test leaves it false. */
            if (as_state == ASState::DRIVING &&
                active_mission != nullptr && active_mission->requests_r2d &&
                !g_can_r2d.load())
            {
                static uint32_t last_r2d_req_ms = 0;
                if ((uint32_t)(now_ms - last_r2d_req_ms) >= R2D_REQ_PERIOD_MS)
                {
                    last_r2d_req_ms = now_ms;
                    Can::sendR2dRequest(1u);
                }
            }

            // State-specific logic
            switch (as_state)
            {
                case ASState::OFF:
                    /* EBS actuation (init self-test while arming, else engage)
                     * is handled by the centralised as_actuation() block above.
                     * Nothing state-specific here. */
                    break;

                case ASState::READY:
                    /* EBS engaged centrally above (as_actuation).
                     * Stamp the READY-entry time; the mandated dwell is enforced
                     * on the transition via as_in.ready_dwell_elapsed (built
                     * above from this timestamp). g_can_listen_go mirrors the
                     * same >= READY_DWELL_MS condition for telemetry (pit_diag). */
                    if (ready_start_time == 0)
                    {
                        ready_start_time = hardware_io_now_ms();
                    }
                    else if (hardware_io_now_ms() - ready_start_time > READY_DWELL_MS)
                    {
                        g_can_listen_go.store(true);
                    }
                    break;

                case ASState::DRIVING:
                {
                    /* EBS released centrally above (as_actuation).
                     * Keep the steering motor armed while DRIVING. The steering
                     * board runs the motor only while it keeps receiving
                     * motor_start (0x520=1) and cuts out on a command-timeout;
                     * a single start on the GO edge was NOT enough on the car
                     * (feat/18-ebs, car-proven). Re-send it, paced to the 20 ms
                     * torque clock to avoid flooding 0x520 (feat/18-ebs sent it
                     * every ~1 ms tick). Homing runs once on the board (guarded
                     * by !arrancado), so repeats do not re-home. The GO-edge
                     * start above still arms it immediately. */
                    if (torque_tx_due)
                    {
                        Can::sendSteeringStart();
                    }

                    // CAN start-mission FALLBACK (ported from dev): fire the
                    // (currently stubbed) start-mission command once when the
                    // legacy setup handshake reports ready, for a pipeline
                    // mission. Coexists with the primary dv/status handshake —
                    // the pipeline path needs nothing from this, but if the ROS
                    // command layer is ever rewired (see ros_task_commands.h)
                    // the legacy CAN orchestration picks up where dev left it.
                    // This is mission-ID orchestration, not per-tick actuation,
                    // so it stays in app_task rather than a mission body.
                    if (active_mission != nullptr && active_mission->needs_pipeline)
                    {
                        if (g_set_mission_ready.load() && !g_mission_going_cmd.load()
                            && !start_mission_sent)
                        {
                            if (send_start_mission_command(current_mission_id))
                            {
                                start_mission_sent = true;
                            }
                        }
                    }

                    // --- Per-mission actuation via the Mission vtable ---
                    // The mission body is pure: it reads the MissionCtx snapshot
                    // and returns the actuation, which we apply on the CAN bus.
                    // is_complete raises mission_complete -> FINISHED next tick
                    // (as_next_state). A pipeline mission (trackdrive) leaves
                    // is_complete nullptr and finishes via dv/status FINISHED.
                    // Emergency / finished exits are handled by the transition
                    // chain above — they move us out of DRIVING before this runs.
                    // Dispatch to the mission BOUND at the GO edge (active_mission),
                    // not the per-tick selected code, so a mid-run 0x503 change
                    // can neither swap the running mission's actuation nor null it
                    // here (which would leave DRIVING with brakes off and no
                    // command). active_mission is non-null for the whole DRIVING
                    // span (set on the GO edge, which mission_valid only allows for
                    // a real mission); the guard is defensive.
                    if (active_mission != nullptr)
                    {
                        MissionCtx ctx = app_build_mission_ctx(now_ms, now_ms - mission_time);
                        if (active_mission->on_tick != nullptr)
                        {
                            /* Pipeline missions stream the 0x507 torque paced to
                             * the ECU's 20 ms cycle (torque_tx_due); a mission's
                             * steering-angle channel keeps its own cadence. */
                            app_apply_mission_command(active_mission->on_tick(&ctx), torque_tx_due);
                        }
                        if (active_mission->is_complete != nullptr &&
                            active_mission->is_complete(&ctx))
                        {
                            mission_complete = true;
                        }
                    }
                    break;
                }

                case ASState::EMERGENCY:
                    /* EBS fired + AS SDC opened centrally above (as_actuation). */
                    // Cancel any active mission
                    if (g_set_mission_in_progress.load())
                    {
                        send_cancel_set_mission_command();
                    }

                    if (g_mission_going_cmd.load())
                    {
                        send_cancel_mission_command();
                    }
                    break;

                case ASState::FINISHED:
                    /* EBS fired + AS SDC opened centrally above (as_actuation).
                     * FINISHED latches until ASMS-off; the SDC stays open until a
                     * reset/power-cycle re-runs the EBS init (re-closes it). */
                    // Mission complete - ensure any mission is cancelled
                    if (g_set_mission_in_progress.load())
                    {
                        send_cancel_set_mission_command();
                    }

                    if (g_mission_going_cmd.load())
                    {
                        send_cancel_mission_command();
                    }
                    break;
            }
        }
        /* Manual mode (ASMS off): the EBS release + SafeManual() are applied by
         * the centralised as_actuation() block above; no autonomous per-state
         * logic runs when ASMS is off. */

        // Small delay to prevent CPU hogging (safety monitor expects our
        // heartbeat within its 100 ms deadline)
        osDelay(1);
    }
}
