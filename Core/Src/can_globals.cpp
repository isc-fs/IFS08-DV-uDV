#include "can_globals.h"
#include "bench_stubs.h"   /* BENCH_STUB_TS (0 on dev) */

// Definitions of CAN atomic signals
std::atomic<bool> g_can_r2d{false};
std::atomic<bool> g_imu_vehicle_standstill{true};
std::atomic<int>  g_can_mission_id{-1};  /* -1 = none received; 0 is a valid 0-based mission */
std::atomic<bool> g_can_listen_go{false};
std::atomic<bool> g_can_ts_active{false};
std::atomic<uint32_t> g_can_ts_active_stamp_ms{0};
std::atomic<bool> g_can_brake_over_limit{false};
std::atomic<int32_t> g_can_motor_rpm{0};
std::atomic<bool> g_can_sdc_res_open{false};
std::atomic<bool> g_reset_cmd{false};

// Steering controller feedback (FDCAN3 0x528)
std::atomic<float> g_steer_angle_actual{0.0f};
std::atomic<float> g_steer_angle_target{0.0f};
std::atomic<float> g_steer_angle_motor{0.0f};
// Default OFF (not EMERGENCIA) so an absent 0x528 never fabricates an emergency.
std::atomic<int8_t> g_steer_motor_state{ESTADO_MOTOR_OFF};
// Power-on zero-cal OK (byte 6 of 0x528). Default 0 (not calibrated) until a
// frame with the flag set arrives — an absent steering never reads as calibrated.
std::atomic<uint8_t> g_steer_cal_arranque_ok{0};

// Steering end-stop calibration status (0x529, FDCAN3, #113)
std::atomic<uint8_t>  g_steer_calib_phase{0};
std::atomic<uint8_t>  g_steer_calib_error{0};
std::atomic<int16_t>  g_steer_calib_center{0};
std::atomic<int16_t>  g_steer_calib_halfrange{0};
std::atomic<int16_t>  g_steer_calib_limit{0};
std::atomic<uint32_t> g_steer_calib_last_rx_tick{0};

// Calibration relay diag (uDV side, #113 / steering #21)
std::atomic<uint16_t> g_calib_trigger_rx_count{0};
std::atomic<uint16_t> g_calib_relay_count{0};
std::atomic<uint8_t>  g_calib_last_cmd{0};

// Steering sensor state (re-ported from v0.1)
std::atomic<int16_t>  g_steering_angle_raw{0};
std::atomic<int8_t>   g_steering_speed_raw{0};
std::atomic<uint8_t>  g_steering_status{0};
std::atomic<uint32_t> g_steering_last_rx_tick{0};

// RES CANopen state (re-ported from v0.1)
std::atomic<bool>     g_res_estop{false};
std::atomic<uint8_t>  g_res_go_signal{0};
std::atomic<uint8_t>  g_res_radio_quality{0};
std::atomic<bool>     g_res_pre_alarm{false};
std::atomic<uint32_t> g_res_last_rx_tick{0};
std::atomic<uint8_t>  g_res_raw0{0};        /* last 0x191 data[0] (pit-diag) */
std::atomic<uint16_t> g_res_rx_frame_count{0}; /* total FDCAN1 RES-queue frames (pit-diag) */
std::atomic<uint16_t> g_nmt_sent_count{0};     /* NMT set-operational TX count (pit-diag) */



/* ---- ECU 0x504 tractive-system-active view ------------------------------
 * Fail-safe: anything other than "a fresh frame that says yes" is NO. The
 * failure this guards is subtle — g_can_ts_active is a latched atomic, so if
 * the ECU dies or the ACU bus drops, the last received `true` would sit there
 * forever and the uDV would keep believing the TS is live with nobody home.
 * The stamp is what makes silence readable as "TS unknown -> treat as off".
 *
 * Deliberately NOT gated on ASMS: the ASMS is a separate signal the AS
 * transition already consumes on its own (as_in.asms_on). Folding it in here
 * would conflate "the autonomous system is switched on" with "the tractive
 * system is energised" — two different facts with two different owners. */
bool can_ts_active_fresh(uint32_t now_ms)
{
    const uint32_t last = g_can_ts_active_stamp_ms.load();

    /* Bench stub (bench_stubs.h, 0 on dev — folds away): with no ECU on the
     * bench no 0x504 ever arrives, `last` stays 0, TS reads permanently off and
     * AS READY is unreachable. Report a live TS instead, but ONLY while nothing
     * real has been seen: the moment a genuine 0x504 lands, `last` != 0 and the
     * real freshness logic below takes over for the rest of the power cycle
     * (so a real ECU saying "TS down" always wins over the stub). */
    if (BENCH_STUB_TS && last == 0u) return true;

    if (last == 0u)                            return false;  /* never received */
    if ((now_ms - last) >= TS_ACTIVE_STALE_MS) return false;  /* stale -> unknown */
    return g_can_ts_active.load();
}
