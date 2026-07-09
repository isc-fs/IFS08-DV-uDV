#include "ros_globals.h"
#include "as_state.h"
#include "dv_interface.h"

// Definitions of atomic feedback from PC, owned by RosTask
std::atomic<float> g_accel_cmd{0.0f};
std::atomic<float> g_steer_cmd{0.0f};
std::atomic<bool>  g_finished_cmd{false};
std::atomic<bool>  g_emergency_cmd{false};
std::atomic<bool>  g_mission_going_cmd{false};
std::atomic<bool>  g_set_mission_in_progress{false};
std::atomic<bool>  g_set_mission_ready{false};

// Stock-typed pipeline interface state (see dv_interface.h).
std::atomic<uint8_t>  g_dv_status{DV_STATUS_IDLE};
std::atomic<uint32_t> g_dv_status_stamp_ms{0};
std::atomic<uint32_t> g_ctrl_cmd_stamp_ms{0};
std::atomic<bool>     g_finish_brake_req{false};

// State telemetry snapshot, owned by AppTask and published by RosTask
std::atomic<uint8_t> g_telemetry_as_state{0};
std::atomic<uint8_t> g_telemetry_ebs_init_state{0};
std::atomic<bool>    g_telemetry_asms_on{false};
std::atomic<bool>    g_telemetry_ts_active{false};
std::atomic<bool>    g_telemetry_sdc_res_open{false};
std::atomic<bool>    g_telemetry_brakes_engaged{false};
std::atomic<bool>    g_telemetry_r2d{false};
std::atomic<bool>    g_telemetry_vehicle_standstill{true};
std::atomic<bool>    g_telemetry_mission_selected{false};
std::atomic<bool>    g_telemetry_mission_finished{false};
std::atomic<bool>    g_telemetry_abs_checks_ok{false};
std::atomic<bool>    g_telemetry_ebs_activated{false};

// IMU drop counter — incremented by imu_task whenever osMessageQueuePut
// to imuQueueHandle returns non-OK (queue full), read by ros_interface
// during the ~10 s periodic re-sync.
std::atomic<uint32_t> g_imu_drop_count{0};

// ROS command queue
QueueHandle_t g_ros_cmd_queue = nullptr;

// C-callable raw AS-state accessor (see as_state.h). Single source of
// truth shared by the ROS /as_state publisher and the CAN DV-logger
// 0x502 frame, so both read the same AppTask telemetry snapshot.
extern "C" uint8_t ros_get_as_state(void)
{
    return g_telemetry_as_state.load();
}

// C-callable snapshot of the 10 StateManagerSignals booleans, packed with the
// AS_SIG_* bits from as_state.h so the C ROS task can format them for /debug.
extern "C" uint16_t ros_get_state_signals(void)
{
    uint16_t b = 0;
    if (g_telemetry_asms_on.load())           b |= AS_SIG_ASMS_ON;
    if (g_telemetry_ts_active.load())         b |= AS_SIG_TS_ACTIVE;
    if (g_telemetry_sdc_res_open.load())      b |= AS_SIG_SDC_RES_OPEN;
    if (g_telemetry_ebs_activated.load())     b |= AS_SIG_EBS_ACTIVATED;
    if (g_telemetry_abs_checks_ok.load())     b |= AS_SIG_ABS_CHECKS_OK;
    if (g_telemetry_brakes_engaged.load())    b |= AS_SIG_BRAKES_ENGAGED;
    if (g_telemetry_mission_selected.load())  b |= AS_SIG_MISSION_SEL;
    if (g_telemetry_r2d.load())               b |= AS_SIG_R2D;
    if (g_telemetry_vehicle_standstill.load())b |= AS_SIG_STANDSTILL;
    if (g_telemetry_mission_finished.load())  b |= AS_SIG_MISSION_DONE;
    return b;
}

extern "C" uint8_t ros_get_ebs_init_state(void)
{
    return g_telemetry_ebs_init_state.load();
}

// --- Stock-typed pipeline interface bridge (see dv_interface.h) ---
// C-callable so the C micro-ROS callbacks in ros_task.c can drive the
// C++ command atomics without ever seeing std::atomic<>.

extern "C" void ros_set_ctrl_cmd_norm(float throttle_norm, float steering_norm,
                                      uint32_t now_ms)
{
    // Never trust the wire: clamp to the normalised [-1, 1] contract range.
    if (throttle_norm >  1.0f) throttle_norm =  1.0f;
    else if (throttle_norm < -1.0f) throttle_norm = -1.0f;
    if (steering_norm >  1.0f) steering_norm =  1.0f;
    else if (steering_norm < -1.0f) steering_norm = -1.0f;

    g_accel_cmd.store(throttle_norm);
    g_steer_cmd.store(steering_norm);
    g_ctrl_cmd_stamp_ms.store(now_ms);
}

extern "C" void ros_set_dv_status(uint8_t status, uint32_t now_ms)
{
    g_dv_status.store(status);
    g_dv_status_stamp_ms.store(now_ms);
}

extern "C" uint8_t ros_get_dv_status(void)
{
    return g_dv_status.load();
}

extern "C" void ros_set_finish_brake(bool on)
{
    g_finish_brake_req.store(on);
}
