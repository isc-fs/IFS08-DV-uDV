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
    return (uint8_t)((BENCH_STUB_EBS_INIT ? 0x01u : 0u) |
                     (BENCH_STUB_DVPC     ? 0x02u : 0u));
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
    }
    if ((uint32_t)(now_ms - last_slow) >= 1000u) {
        last_slow = now_ms;
        send_fwinfo(now_ms);
    }
}
