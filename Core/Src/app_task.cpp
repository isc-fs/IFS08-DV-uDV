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
    #include "assi.h"            /* ASSI status LEDs (D5/PB8), FS-Rules T14.9 */
}

#include "state_manager.hpp"
#include "ebs_manager.hpp"
#include "ros_task_commands.h"
#include "can_interface.hpp"
#include "can_task.h"
#include "ros_globals.h"
#include "can_globals.h"

extern "C" {
    #include "dv_interface.h"   /* dv/status + ctrl/cmd contract bytes + timings */
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
    g_can_mission_id.store(0);
    g_can_r2d.store(false);  /* (g_can_go was redundant with this) */
    g_imu_vehicle_standstill.store(true);
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

    // ASSI status LEDs (D5/PB8). assi_init drives the chain OFF, which is the
    // correct AS Off indication (T14.9.1). Track the last rendered colour so we
    // only re-clock the chain when it actually changes (state or flash edge).
    assi_init();
    uint8_t assi_last_r = 0xFF, assi_last_g = 0xFF, assi_last_b = 0xFF;

    // Main control loop
    while (1)
    {
        // State machine dispatcher REQUIREMENTS:

        //1. cuando asms esta off siempre retornar a la AS_off
        //2. Si asms on, ts on y RES ok (es decir res 0 mira la funcion can_c_get_res_status()). transicion a AS_ready
        //3. Si RES E-stop en cualquier momento transicionar a AS_emergency
        //4. Los frenos se sueltan cuando estan en HIGH. Es decir que solo se debe de activar los canales de EBS cuando esta en AS_driving. En el resto de estados los canales de EBS deben de estar desactivados. D1 y D2 low
        //5. transicinar de AS_ready a AS_driving cuando se reciba el comando de start del RES
        //6. transicionar a emergency si el RES esta bien y el TS pasa a esta off.
        //7. configura para que la mision por defecto sea INSPECTION
        //8. Por el momento no hay forma de transicinar a AS_finished.
        //9. La unica forma de salir de AS_emergency es pasar por asms off.

        // Check for reset command (issued by external supervisor)
        if (g_reset_cmd.load())
        {
            set_mission_sent = false;
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
        bool res_ok = (res_status == 0);
        bool res_go = (res_status == 2);
        bool res_estop = (res_status == 1);

        // --- Pipeline (DVPC) status, read level-triggered off /dv/status ---
        // The uDV owns the AS state machine; mission_control only answers
        // with this byte. It is also the DVPC liveness heartbeat: a stale
        // /dv/status while driving means the pipeline died -> safe state.
        uint8_t  dv_status     = g_dv_status.load();
        bool     dv_seen       = (g_dv_status_stamp_ms.load() != 0u);
        bool     dv_fresh      = dv_seen &&
                                 ((now_ms - g_dv_status_stamp_ms.load()) < DV_STATUS_STALE_MS);
        bool     dv_ready      = dv_fresh && (dv_status == DV_STATUS_READY);
        bool     dv_finished   = dv_fresh && (dv_status == DV_STATUS_FINISHED);
        bool     dv_emergency  = dv_fresh && (dv_status == DV_STATUS_EMERGENCY ||
                                              dv_status == DV_STATUS_FAILED);
        // Pipeline heartbeat lost while we were driving (link/DVPC dead).
        bool     dv_lost_driving = (previous_as_state == ASState::DRIVING) && !dv_fresh;

        if (!asms_on)
        {
            as_state = ASState::OFF;
        }
        else if (previous_as_state == ASState::EMERGENCY)
        {
            as_state = ASState::EMERGENCY;
        }
        else if (previous_as_state == ASState::FINISHED)
        {
            // Mission complete latches until ASMS off (rule #9 analogue).
            as_state = ASState::FINISHED;
        }
        else if (res_estop
                 || (!ts_on && (previous_as_state == ASState::DRIVING || previous_as_state == ASState::READY))
                 || (dv_emergency && (previous_as_state == ASState::DRIVING || previous_as_state == ASState::READY))
                 || dv_lost_driving)
        {
            // RES e-stop, TS lost, pipeline-raised emergency, or a lost
            // pipeline heartbeat mid-run -> AS Emergency.
            as_state = ASState::EMERGENCY;
        }
        else if (previous_as_state == ASState::DRIVING && dv_finished)
        {
            // Pipeline reported the mission finished (was RuntimeControl
            // outcome=finished) -> end the run cleanly.
            as_state = ASState::FINISHED;
        }
        else if (res_go && previous_as_state == ASState::READY && dv_ready)
        {
            // RES "go" is honoured ONLY while the pipeline reports READY —
            // the handshake that replaces waiting on the old SetMission result.
            as_state = ASState::DRIVING;
        }
        else if (res_ok && ts_on)
        {
            as_state = ASState::READY;
        }

        sync_state_telemetry(state_mgr, ebs, as_state);

        // Send ASSI status if state changed
        if (as_state != last_as_state)
        {
            Can::sendAssiStatus(StateManager::getAssiStatusCode(as_state));
            last_as_state = as_state;
        }

        // Drive the physical ASSI LEDs from the AS state (FS-Rules T14.9.1):
        //   Off -> off, Ready -> yellow steady, Driving -> yellow flashing,
        //   Emergency -> blue flashing, Finished -> blue steady.
        // Flash: 150 ms half-period => ~3.3 Hz, 50 % duty (rules: 2-5 Hz, 50 %).
        {
            const bool flash_on = ((hardware_io_now_ms() / 150u) & 1u) != 0u;
            uint8_t r = 0, g = 0, b = 0;
            switch (as_state)
            {
                case ASState::OFF:                                     break;
                case ASState::READY:     r = 255; g = 255;             break;
                case ASState::DRIVING:   if (flash_on) { r = 255; g = 255; } break;
                case ASState::EMERGENCY: if (flash_on) { b = 255; }    break;
                case ASState::FINISHED:  b = 255;                      break;
            }
            if (r != assi_last_r || g != assi_last_g || b != assi_last_b)
            {
                assi_set_all(r, g, b);
                assi_show();
                assi_last_r = r; assi_last_g = g; assi_last_b = b;
            }
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
            }

            int current_mission_id = g_can_mission_id.load();
            if (current_mission_id <= 0)
            {
                current_mission_id = 6;  // Default mission: Inspection
            }

            if (current_mission_id != last_mission_id)
            {
                last_mission_id = current_mission_id;
                set_mission_sent = false;
                g_set_mission_in_progress.store(false);
                g_set_mission_ready.store(false);
            }

            if (current_mission_id > 0 && !set_mission_sent &&
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
                    // Perform EBS initialization sequence steps
                    if (ebs_state != EBSInitState::Done && ebs_state != EBSInitState::Failed)
                    {
                        ebs_state = ebs.initSequenceStep();
                    }
                    else
                    {
                        ebs.deactivateEBS();
                    }
                    break;

                case ASState::READY:
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
                    // Mission complete - EBS already active, just ensure mission is cancelled
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
            // Manual mode: verify safe conditions (empty pressure tanks)
            ebs.SafeManual();
            ebs.deactivateEBS();
        }

        // Small delay to prevent CPU hogging (but watchdog timeout < 50ms)
        osDelay(1);
    }
}
