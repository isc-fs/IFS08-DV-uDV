/**
 * @file    pit_diag.cpp
 * @brief   Pit-diag frame builders + arm gate + cadence (see pit_diag.h).
 *
 * Pure-ish: reads the live atomics/getters, packs bytes, TXs on FDCAN2. No
 * blocking. Called from canTask (service) and rx_dispatch (arm).
 */
#include "pit_diag.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "fdcan.h"                /* hfdcan2 */

#include "can_globals.h"          /* RES / steer / mission atomics + accessors */
#include "ros_globals.h"          /* dv/status, ctrl cmds, setup + telemetry   */
#include "as_state.h"             /* ros_get_as_state / _state_signals / _ebs   */
#include "assi_task.h"            /* assi_get_mode                              */
#include "safety_monitor.h"       /* safety_diag_* health snapshots            */
#include "bench_stubs.h"          /* BENCH_STUB_* mask                          */

/* Build id injected by the Makefile (-DGIT_HASH=0x......); 0 if unset. */
#ifndef GIT_HASH
#define GIT_HASH 0u
#endif

/* ---- enable state (sticky) ---------------------------------------------- */
static volatile uint8_t s_armed = (PITDIAG_STREAM_ALWAYS != 0);

void pit_diag_arm_from_can(const uint8_t *data, uint8_t dlc)
{
    if (dlc < 4U) return;
    /* Big-endian magic, matching the ECU's FIELD_BE convention. */
    uint32_t magic = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8)  |  (uint32_t)data[3];
    if (magic == PITDIAG_ARM_MAGIC) {
        s_armed = 1u;
    }
}

uint8_t pit_diag_is_armed(void) { return s_armed; }

/* ---- helpers ------------------------------------------------------------ */
static inline void tx8(uint32_t id, const uint8_t d[8])
{
    FDCAN_TxHeaderTypeDef h = {
        .Identifier          = id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    /* Best-effort — a full TX FIFO just drops this diag frame (never blocks
     * the service loop). */
    (void)HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &h, d);
}

/* Age of a stamped event in ms, clamped to u16; 0xFFFF means "never seen". */
static inline uint16_t age_ms(uint32_t stamp, uint32_t now)
{
    if (stamp == 0u) return 0xFFFFu;
    uint32_t a = now - stamp;
    return (a > 0xFFFEu) ? 0xFFFEu : (uint16_t)a;
}

static inline int8_t clamp_pct(float f)   /* [-1,1] -> [-100,100] */
{
    int v = (int)(f * 100.0f + (f >= 0 ? 0.5f : -0.5f));
    if (v > 100) v = 100; if (v < -100) v = -100;
    return (int8_t)v;
}

static inline uint8_t stub_mask(void)
{
    return (uint8_t)((BENCH_STUB_EBS_INIT    ? 0x01u : 0u) |
                     (BENCH_STUB_DVPC        ? 0x02u : 0u) |
                     (BENCH_STUB_EBS_SENSORS ? 0x04u : 0u) |
                     (BENCH_STUB_SDC         ? 0x08u : 0u) |
                     (BENCH_STUB_STEERING    ? 0x10u : 0u) |
                     (BENCH_STUB_IMU_ROS     ? 0x20u : 0u) |
                     (BENCH_STUB_TS          ? 0x40u : 0u) |
                     (BENCH_STUB_DV_STOPPING ? 0x80u : 0u));
}

/* ---- frame builders ----------------------------------------------------- */
static void send_status(void)
{
    uint16_t sig = ros_get_state_signals();
    uint8_t d[8] = {
        ros_get_as_state(),
        (uint8_t)(sig & 0xFFu),
        (uint8_t)((sig >> 8) & 0xFFu),
        (uint8_t)(int8_t)g_can_mission_id.load(),
        ros_get_ebs_init_state(),
        stub_mask(),
        (uint8_t)assi_get_mode(),
        s_armed,                        /* [7] diag armed flag */
    };
    tx8(CAN_ID_PITDIAG_STATUS, d);
}

static void send_res(uint32_t now)
{
    int32_t st = can_c_get_res_status(now, 150u);
    uint8_t bits =
        (uint8_t)((g_res_estop.load()          ? 0x01u : 0u) |
                  (g_res_go_signal.load()      ? 0x02u : 0u) |
                  (g_res_pre_alarm.load()      ? 0x04u : 0u) |
                  (g_can_brake_over_limit.load()? 0x08u : 0u) |
                  (g_can_listen_go.load()      ? 0x10u : 0u) |
                  (g_can_sdc_res_open.load()   ? 0x20u : 0u) |
                  (g_can_ts_active.load()      ? 0x40u : 0u));
    uint16_t res_age = age_ms(g_res_last_rx_tick.load(), now);
    uint8_t d[8] = {
        g_res_raw0.load(),               /* [0] raw 0x191 data[0]           */
        (uint8_t)(int8_t)st,             /* [1] RES status (-2..2)          */
        bits,                            /* [2] decoded input bits          */
        g_res_radio_quality.load(),      /* [3] RES radio quality           */
        (uint8_t)(res_age & 0xFFu),      /* [4] RES frame age ms (LE)       */
        (uint8_t)(res_age >> 8),         /* [5]                             */
        (uint8_t)g_steer_motor_state.load(), /* [6] ESTADO_MOTOR (-1/0/1)   */
        g_steering_status.load(),        /* [7] LWS status byte             */
    };
    tx8(CAN_ID_PITDIAG_RES, d);
}

static void send_pipe(uint32_t now)
{
    uint16_t dv_age   = age_ms(g_dv_status_stamp_ms.load(), now);
    uint16_t ctrl_age = age_ms(g_ctrl_cmd_stamp_ms.load(), now);
    uint8_t setup =
        (uint8_t)((g_set_mission_in_progress.load() ? 0x01u : 0u) |
                  (g_set_mission_ready.load()       ? 0x02u : 0u) |
                  (g_mission_going_cmd.load()       ? 0x04u : 0u) |
                  (g_emergency_cmd.load()           ? 0x08u : 0u) |
                  (g_finished_cmd.load()            ? 0x10u : 0u));
    uint8_t d[8] = {
        g_dv_status.load(),              /* [0] /dv/status byte             */
        (uint8_t)(dv_age & 0xFFu),       /* [1] dv age ms (LE)              */
        (uint8_t)(dv_age >> 8),          /* [2]                             */
        (uint8_t)clamp_pct(g_accel_cmd.load()),  /* [3] accel cmd %         */
        (uint8_t)clamp_pct(g_steer_cmd.load()),  /* [4] steer cmd (norm*100)*/
        (uint8_t)(ctrl_age & 0xFFu),     /* [5] ctrl/cmd age ms (LE)        */
        (uint8_t)(ctrl_age >> 8),        /* [6]                             */
        setup,                           /* [7] mission-setup bits          */
    };
    tx8(CAN_ID_PITDIAG_PIPE, d);
}

static void send_health(uint32_t now)
{
    uint16_t freew = (uint16_t)(xPortGetFreeHeapSize() >> 2);          /* words */
    uint16_t minw  = (uint16_t)(xPortGetMinimumEverFreeHeapSize() >> 2);
    uint8_t flags =
        (uint8_t)((safety_diag_reset_flag() ? 0x01u : 0u) |   /* IWDG-reset boot */
                  (safety_diag_latched()    ? 0x02u : 0u));   /* emergency latched */
    uint8_t d[8] = {
        (uint8_t)(freew & 0xFFu),        /* [0-1] free heap /4 (LE)         */
        (uint8_t)(freew >> 8),
        (uint8_t)(minw & 0xFFu),         /* [2-3] min-ever heap /4 (LE)     */
        (uint8_t)(minw >> 8),
        safety_diag_armed_mask(),        /* [4] armed task bitmask          */
        flags,                           /* [5] reset/latched flags         */
        (uint8_t)(int8_t)safety_diag_stalled_task(), /* [6] stalled id, -1  */
        (uint8_t)((now / 1000u) & 0xFFu),/* [7] uptime s (wraps 256)        */
    };
    tx8(CAN_ID_PITDIAG_HEALTH, d);
}

static void send_fwinfo(uint32_t now)
{
    uint32_t h = (uint32_t)GIT_HASH;
    uint8_t d[8] = {
        (uint8_t)(h & 0xFFu),            /* [0-3] git short hash (LE)       */
        (uint8_t)((h >> 8) & 0xFFu),
        (uint8_t)((h >> 16) & 0xFFu),
        (uint8_t)((h >> 24) & 0xFFu),
        stub_mask(),                     /* [4] compiled-in stub mask       */
        (uint8_t)(configTOTAL_HEAP_SIZE / 1024u), /* [5] heap size KB       */
        (uint8_t)((now / 1000u) & 0xFFu),/* [6] uptime s                    */
        0u,                              /* [7] spare                       */
    };
    tx8(CAN_ID_PITDIAG_FWINFO, d);
}

/* FDCAN1 (RES bus) health — so a bench that CANNOT tap FDCAN1 can still tell,
 * from the FDCAN2 stream, whether the bus is dead or the RES is just silent:
 *   - bus_off / TEC pinned + LEC=ACK(3)  -> electrical: RES not ACKing /
 *     termination / RES unpowered (the uDV is alone on the bus).
 *   - error-active, rx_frames == 0        -> FDCAN1 healthy, RES genuinely
 *     silent (no 0x191 and no 0x711 boot-up arriving).
 * nmt_sent proves the uDV is actively (re)commanding the RES operational. */
static void send_canhealth(void)
{
    FDCAN_ProtocolStatusTypeDef ps = {0};
    FDCAN_ErrorCountersTypeDef  ec = {0};
    (void)HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    (void)HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
    uint8_t flags =
        (uint8_t)((ps.BusOff        ? 0x01u : 0u) |
                  (ps.ErrorPassive  ? 0x02u : 0u) |
                  (ps.Warning       ? 0x04u : 0u));
    uint16_t rxf = g_res_rx_frame_count.load();
    uint16_t nmt = g_nmt_sent_count.load();
    uint8_t d[8] = {
        flags,                              /* [0] b0=BusOff b1=ErrPassive b2=Warning */
        (uint8_t)ps.LastErrorCode,          /* [1] LEC: 3=ACK(no other node) 0/7=ok   */
        (uint8_t)(ec.TxErrorCnt & 0xFFu),   /* [2] TEC (0..255)                       */
        (uint8_t)(ec.RxErrorCnt & 0xFFu),   /* [3] REC (0..127)                       */
        (uint8_t)(rxf & 0xFFu),             /* [4-5] FDCAN1 filtered RX frame count LE */
        (uint8_t)(rxf >> 8),
        (uint8_t)(nmt & 0xFFu),             /* [6] NMT set-operational TX count (low)  */
        (uint8_t)(ps.LastErrorCode == 3u ? 1u : 0u), /* [7] convenience: ACK-error now */
    };
    tx8(CAN_ID_PITDIAG_CANHEALTH, d);
}

/* Steering end-stop calibration status relay (#113): forwards the steering's
 * 0x529 (received on FDCAN3) to the pit tool on FDCAN2, so MingoCAN can show
 * calibration progress without tapping the steering bus. */
static void send_calib(void)
{
    /* Layout matches the MingoCAN host decoder verbatim (can-flasher#428/#430):
     * all 8 bytes are calib data — phase, error, then three int16 LE ×0.1° values,
     * no age byte. */
    int16_t c   = g_steer_calib_center.load();
    int16_t hr  = g_steer_calib_halfrange.load();
    int16_t lim = g_steer_calib_limit.load();
    uint8_t d[8] = {
        g_steer_calib_phase.load(),      /* [0] phase (0 idle .. 9 OK, 10 FAIL) */
        g_steer_calib_error.load(),      /* [1] error code                       */
        (uint8_t)(c & 0xFFu),   (uint8_t)((uint16_t)c >> 8),     /* [2-3] center x0.1deg    */
        (uint8_t)(hr & 0xFFu),  (uint8_t)((uint16_t)hr >> 8),    /* [4-5] half-range x0.1   */
        (uint8_t)(lim & 0xFFu), (uint8_t)((uint16_t)lim >> 8),   /* [6-7] motor limit x0.1  */
    };
    tx8(CAN_ID_PITDIAG_CALIB, d);
}

/* Live steering angle relay so a CAN-only pit (MingoCAN on FDCAN2) can SEE what
 * the LWS is measuring — essential during manual end-stop calibration, where the
 * operator turns the wheel to the stops and needs the live angle (the raw LWS
 * 0x2B0 and the 0x528 feedback are on FDCAN3, invisible to the pit otherwise). */
static void send_steer(void)
{
    int16_t lws_raw = g_steering_angle_raw.load();           /* 0x2B0, x0.1 deg    */
    int16_t act = (int16_t)(g_steer_angle_actual.load() * 10.0f);  /* 0x528, ->x0.1 */
    int16_t tgt = (int16_t)(g_steer_angle_target.load() * 10.0f);
    uint8_t d[8] = {
        (uint8_t)(lws_raw & 0xFFu), (uint8_t)((uint16_t)lws_raw >> 8), /* [0-1] LWS raw x0.1deg */
        (uint8_t)(act & 0xFFu),     (uint8_t)((uint16_t)act >> 8),     /* [2-3] actual x0.1deg  */
        (uint8_t)(tgt & 0xFFu),     (uint8_t)((uint16_t)tgt >> 8),     /* [4-5] target x0.1deg  */
        g_steering_status.load(),                    /* [6] LWS status byte            */
        (uint8_t)(int8_t)g_steer_motor_state.load(), /* [7] ESTADO_MOTOR (-1/0/1/2)    */
    };
    tx8(CAN_ID_PITDIAG_STEER, d);
}

/* Calibration RELAY diag (uDV side): closes the loop on the 0x7DF->0x522 path so
 * the bench can tell where a stuck calibration is. rx_count = 0x7DF frames seen
 * on FDCAN2 (reached the uDV); relay_count = 0x522 frames TX'd on FDCAN3 (uDV
 * commanded the steering). Read: rx=0 -> trigger never arrived (wrong bus / not
 * armed-tool / not sent); rx>0 & relay=0 -> received but not relayed (armed=0 or
 * bad cmd); relay>0 but steering idle -> it's the steering (steering #21). */
static void send_calib_dbg(void)
{
    uint16_t rx  = g_calib_trigger_rx_count.load();
    uint16_t rly = g_calib_relay_count.load();
    uint8_t d[8] = {
        (uint8_t)(rx & 0xFFu),  (uint8_t)(rx >> 8),   /* [0-1] 0x7DF rx count (FDCAN2) LE */
        (uint8_t)(rly & 0xFFu), (uint8_t)(rly >> 8),  /* [2-3] 0x522 relay count (FDCAN3) LE */
        g_calib_last_cmd.load(),      /* [4] last cmd relayed (1 start / 2 abort) */
        pit_diag_is_armed(),          /* [5] armed gate (0x7DF ignored if 0)      */
        0u, 0u,
    };
    tx8(CAN_ID_PITDIAG_CALIBDBG, d);
}

/* EBS air-tank storage pressures (A5/A4), so a CAN-only bench can watch the
 * tanks charge/vent and validate the sensor scaling (PRES_DIVIDER) against a
 * gauge. AppTask samples the real ADC into the telemetry atomics, so this is
 * the true pressure even when BENCH_STUB_EBS_SENSORS fakes the pass/fail gate.
 * ok-bits use the same 3.0 bar (30 dbar) ACTUATOR_STORAGE_THRESHOLD the EBS
 * self-check applies. */
static void send_ebs(void)
{
    int16_t p1 = ros_get_ebs_pressure1_dbar();   /* deci-bar, x0.1 bar */
    int16_t p2 = ros_get_ebs_pressure2_dbar();
    uint8_t ok = (uint8_t)(((p1 > 30) ? 0x01u : 0u) |   /* tank1 > 3.0 bar */
                           ((p2 > 30) ? 0x02u : 0u));   /* tank2 > 3.0 bar */
    uint8_t d[8] = {
        (uint8_t)(p1 & 0xFFu), (uint8_t)((uint16_t)p1 >> 8),  /* [0-1] tank1 dbar LE */
        (uint8_t)(p2 & 0xFFu), (uint8_t)((uint16_t)p2 >> 8),  /* [2-3] tank2 dbar LE */
        ros_get_ebs_init_state(),   /* [4] EBS init state (Start=0 .. Failed=7 Done=8) */
        stub_mask(),                /* [5] stub mask (bit2=EBS_SENSORS -> gate faked)  */
        ok,                         /* [6] b0=tank1>3bar b1=tank2>3bar (CheckPressure) */
        0u,                         /* [7] spare                                       */
    };
    tx8(CAN_ID_PITDIAG_EBS, d);
}

/* ---- cadence ------------------------------------------------------------ */
void pit_diag_service(uint32_t now_ms)
{
    if (!s_armed) return;

    static uint32_t last_fast = 0u;   /* status/res/pipe/health @100 ms */
    static uint32_t last_slow = 0u;   /* fwinfo @1 s                    */

    if ((uint32_t)(now_ms - last_fast) >= 100u) {
        last_fast = now_ms;
        send_status();
        send_res(now_ms);
        send_pipe(now_ms);
        send_health(now_ms);
        send_canhealth();
        send_calib();
        send_steer();
        send_calib_dbg();
        send_ebs();
    }
    if ((uint32_t)(now_ms - last_slow) >= 1000u) {
        last_slow = now_ms;
        send_fwinfo(now_ms);
    }
}
