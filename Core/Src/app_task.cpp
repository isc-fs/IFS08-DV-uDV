/**
 * @file    app_task.cpp
 * @brief   Application task implementation - Main state machine coordinator
 */

#include "app_task.h"

#include <cstdint>
#include <cstring>
#include <cmath>

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "cmsis_os.h"
    #include "hardware_io.h"
    #include "safety_monitor.h"  /* IWDG safety supervisor: heartbeat/arm */
    #include "assi_task.h"       /* ASSI mode API (UART/Arduino LED bridge) */
}

#include "state_manager.hpp"
#include "as_transition.hpp"   /* pure AS transition decision (host-tested) */
#include "ebs_manager.hpp"
#include "ros_task_commands.h"
#include "can_interface.hpp"
#include "can_task.h"
#include "ros_globals.h"
#include "can_globals.h"

extern "C" {
    #include "dv_interface.h"   /* dv/status + ctrl/cmd contract bytes + timings */
}

/* ------------------------------------------------------------------------
 * Mission dispatch
 *
 * The AMI board sends the 0-based mission menu index on CAN 0x503 byte[0]
 * (see IFS08-DV_AMI Core/Src/ami.c `missions[]`). Autonomous missions are
 * driven by the DV pipeline over /ctrl/cmd; the standalone ones (Inspection,
 * EBS test) run open-loop in this firmware with no pipeline.
 * ------------------------------------------------------------------------ */
enum AmiMission {
    MISSION_MANUAL     = 0,
    MISSION_ACCEL      = 1,
    MISSION_SKIDPAD    = 2,
    MISSION_AUTOCROSS  = 3,
    MISSION_TRACKDRIVE = 4,
    MISSION_EBS_TEST   = 5,
    MISSION_INSPECTION = 6,
    MISSION_SHUTDOWN   = 7,
};

/* Missions whose actuation comes from the DV pipeline (/ctrl/cmd). The rest
 * are standalone: run open-loop here, GO is not gated on the pipeline, and a
 * stale /dv/status does not trip them. */
static inline bool mission_needs_pipeline(int m)
{
    return m == MISSION_ACCEL || m == MISSION_SKIDPAD ||
           m == MISSION_AUTOCROSS || m == MISSION_TRACKDRIVE;
}

/* Inspection open-loop steering sweep: ±INSPECTION_STEER_AMP_DEG at
 * INSPECTION_STEER_FREQ_HZ, one 0x020 angle command per INSPECTION_STEER_DT_MS.
 * Phase derives from the mission-elapsed time so each run starts clean. */
static constexpr float    TWO_PI                   = 6.283185307f;
static constexpr float    INSPECTION_STEER_AMP_DEG = 90.0f;
static constexpr float    INSPECTION_STEER_FREQ_HZ = 0.3f;
static constexpr uint32_t INSPECTION_STEER_DT_MS   = 200u;
static constexpr uint32_t INSPECTION_DURATION_MS   = 30000u;   /* 30 s demo */

static inline float inspection_sweep_deg(uint32_t elapsed_ms)
{
    float t = (float)elapsed_ms / 1000.0f;
    return INSPECTION_STEER_AMP_DEG * sinf(TWO_PI * INSPECTION_STEER_FREQ_HZ * t);
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
    int last_mission_id = -1;
    bool set_mission_sent = false;
    bool start_mission_sent = false;   /* CAN start-mission fallback (dev) */

    /* Standalone-mission (inspection / EBS test) run state. */
    uint32_t mission_time = 0;         /* tick the current DRIVING run began   */
    uint32_t last_steer_emit = 0;      /* rate-limit for the inspection sweep  */
    bool     mission_complete = false; /* open-loop mission done -> FINISHED    */

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

    // Main control loop
    while (1)
    {
        // State machine dispatcher REQUIREMENTS:

        //1. cuando asms esta off siempre retornar a la AS_off
        //2. Si asms on, ts on y RES ok (es decir res 0 mira la funcion can_c_get_res_status()). transicion a AS_ready
        //3. Si RES E-stop en cualquier momento transicionar a AS_emergency
        //4. EBS engaged (brakes on, D1/D2 LOW = fire) in EVERY state except
        //   AS_driving; released (D1/D2 HIGH) only in AS_driving. The car is
        //   immobilised whenever it is not autonomously driving. IMPLEMENTED
        //   below: OFF (after init) / READY / FINISHED / EMERGENCY / manual ->
        //   ebs.activateEBS(); DRIVING -> ebs.deactivateEBS().
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

        /* Steering stepper-driver grave fault (byte 5 of 0x500). EMERGENCIA is
         * an absolute cut-off requiring a physical reset — a dead steering
         * motor means the car can't steer, so treat it like an RES e-stop. */
        bool steer_emergency =
            (g_steer_motor_state.load() == ESTADO_MOTOR_EMERGENCIA);

        // --- Pipeline (DVPC) status, read level-triggered off /dv/status ---
        // The uDV owns the AS state machine; mission_control only answers
        // with this byte. It is also the DVPC liveness heartbeat: a stale
        // /dv/status while driving means the pipeline died -> safe state.
        uint8_t  dv_status     = g_dv_status.load();
        bool     dv_seen       = (g_dv_status_stamp_ms.load() != 0u);
        bool     dv_fresh      = dv_seen &&
                                 ((now_ms - g_dv_status_stamp_ms.load()) < DV_STATUS_STALE_MS);

        /* Selected mission (0-based AMI index on 0x503). Default to Inspection
         * when none has been received. Read here — before the transition — so
         * the state machine can gate the GO / finish rules on the mission type
         * (standalone missions need no pipeline). */
        int current_mission_id = g_can_mission_id.load();
        if (current_mission_id < 0)
        {
            current_mission_id = MISSION_INSPECTION;
        }

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
        as_in.dv_fresh      = dv_fresh;
        as_in.dv_ready     = dv_fresh && (dv_status == DV_STATUS_READY);
        as_in.dv_finished  = dv_fresh && (dv_status == DV_STATUS_FINISHED);
        as_in.dv_emergency = dv_fresh && (dv_status == DV_STATUS_EMERGENCY ||
                                          dv_status == DV_STATUS_FAILED);
        as_in.mission_needs_pipeline = mission_needs_pipeline(current_mission_id);
        as_in.mission_complete       = mission_complete;

        as_state = as_next_state(previous_as_state, as_in);

        /* Steering-motor lifecycle + standalone-mission clock, driven off the
         * AS-state edges:
         *   READY -> DRIVING (GO): energise the steering and (re)start the
         *   mission clock so the inspection sweep begins from phase 0.
         *   -> FINISHED: de-energise the steering. */
        if (as_state == ASState::DRIVING && previous_as_state != ASState::DRIVING)
        {
            Can::sendSteeringStart();
            mission_time     = now_ms;
            last_steer_emit  = 0;
            mission_complete = false;
        }
        if (as_state == ASState::FINISHED && previous_as_state != ASState::FINISHED)
        {
            Can::sendSteeringStop();
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

            // Send zero control when not driving (safety)
            if (as_state != ASState::DRIVING)
            {
                Can::sendAccel(0.0f);
                Can::sendSteer(0.0f);
            }

            // State-specific logic
            switch (as_state)
            {
                case ASState::OFF:
                    // Run the EBS init / ASB self-check; once it completes (or
                    // fails) hold the brakes ENGAGED. EBS is engaged in every
                    // AS state except DRIVING (req #4 / FS-Rules: the car is
                    // immobilised whenever it is not autonomously driving).
                    if (ebs_state != EBSInitState::Done && ebs_state != EBSInitState::Failed)
                    {
                        ebs_state = ebs.initSequenceStep();
                    }
                    else
                    {
                        ebs.activateEBS();
                    }
                    break;

                case ASState::READY:
                    ebs.activateEBS();   // held engaged in READY (req #4)
                    // Wait 5 seconds in READY state before signaling "go" to CAN
                    if (ready_start_time == 0)
                    {
                        ready_start_time = hardware_io_now_ms();
                    }
                    else if (hardware_io_now_ms() - ready_start_time > 5000)
                    {
                        g_can_listen_go.store(true);
                    }
                    break;

                case ASState::DRIVING:
                {
                    ebs.deactivateEBS();  // release: DRIVING is the ONLY state
                                          // with the brakes off (req #4)

                    if (mission_needs_pipeline(current_mission_id))
                    {
                        // --- Autonomous mission: driven by the DV pipeline ---
                        // CAN start-mission FALLBACK (ported from dev): fire the
                        // (currently stubbed) start-mission command once when the
                        // legacy setup handshake reports ready. Coexists with the
                        // primary dv/status handshake — the pipeline path needs
                        // nothing from this, but if the ROS command layer is ever
                        // rewired (see ros_task_commands.h) the legacy CAN
                        // orchestration picks up where dev left it.
                        if (g_set_mission_ready.load() && !g_mission_going_cmd.load()
                            && !start_mission_sent)
                        {
                            if (send_start_mission_command(current_mission_id))
                            {
                                start_mission_sent = true;
                            }
                        }

                        // Stream the pipeline's latest normalised /ctrl/cmd to the
                        // inverter / steering ECU (the ECU expects a constant
                        // stream, so we send every tick from the latched value).
                        // Zero it if /ctrl/cmd goes stale, so a dropped link can
                        // never latch the last throttle/steering. Emergency and
                        // finished are handled by the transition chain above —
                        // they move us out of DRIVING before this runs.
                        // TODO(G2/G3, on-car): confirm the CONTROL_ACCEL /
                        // CONTROL_STEER frames, units, sign and full-lock scaling
                        // against the vehicle CAN DBC. The values here are the
                        // normalised [-1,1] contract, clamped on receive.
                        bool cmd_fresh = (g_ctrl_cmd_stamp_ms.load() != 0u) &&
                                         ((now_ms - g_ctrl_cmd_stamp_ms.load()) < DV_CTRL_CMD_STALE_MS);
                        if (cmd_fresh)
                        {
                            Can::sendAccel(g_accel_cmd.load());
                            Can::sendSteer(g_steer_cmd.load());
                        }
                        else
                        {
                            Can::sendAccel(0.0f);
                            Can::sendSteer(0.0f);
                        }
                    }
                    else if (current_mission_id == MISSION_INSPECTION)
                    {
                        // --- Standalone Inspection: open-loop steering demo ---
                        // Sweep the steering (0x020) to prove the steering
                        // system end-to-end, then self-finish after
                        // INSPECTION_DURATION_MS. The steering motor was started
                        // (0x010) on the GO edge above.
                        uint32_t elapsed = now_ms - mission_time;
                        if (now_ms - last_steer_emit >= INSPECTION_STEER_DT_MS)
                        {
                            Can::sendSteeringAngle(inspection_sweep_deg(elapsed));
                            last_steer_emit = now_ms;
                        }
                        if (elapsed >= INSPECTION_DURATION_MS)
                        {
                            // -> FINISHED on the next tick, through as_next_state.
                            mission_complete = true;
                        }
                    }
                    else if (current_mission_id == MISSION_EBS_TEST)
                    {
                        // --- Standalone EBS test: hold the wheels straight ---
                        // TODO(on-car): drive the motor to the rules test speed,
                        // command the EBS trigger, and verify the deceleration.
                        Can::sendSteeringAngle(0.0f);
                    }
                    break;
                }

                case ASState::EMERGENCY:
                    // EBS should already be active, but ensure it is
                    ebs.activateEBS();

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
                    ebs.activateEBS();   // held engaged in FINISHED (req #4):
                                         // car braked at standstill, mission done
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
        else
        {
            // Manual mode (ASMS off): verify the actuator tanks are safe for
            // manual handling, then hold the brakes ENGAGED — EBS is released
            // only in DRIVING (req #4).
            ebs.SafeManual();
            ebs.activateEBS();
        }

        // Small delay to prevent CPU hogging (safety monitor expects our
        // heartbeat within its 100 ms deadline)
        osDelay(1);
    }
}
