/**
 * @file safety_monitor.c
 * @brief FreeRTOS safety supervisor — heartbeats, IWDG refresh, EBS
 *        emergency. Decision logic lives in the pure safety_eval.c.
 *
 * See safety_monitor.h for the two-tier design. EBS polarity here is the
 * fix/15 convention, confirmed against the car hardware:
 *   D1/D2 HIGH = fire EBS, LOW = normal.   (NOTE: dev's hardware_io used
 *   the opposite, active-low convention — do NOT copy it here.)
 */
#include "safety_monitor.h"

#include "main.h"        /* HAL GPIO + D1/D2 pin macros            */
#include "cmsis_os.h"    /* osDelay, osKernelGetTickCount          */

#include "iwdg.h"
#include "safety_eval.h"

/* C-callable ASSI-EMERGENCY CAN emitter (defined in can_interface.cpp,
 * declared extern "C" in can_interface.hpp). Forward-declared here so
 * this C file doesn't include the C++ header. */
extern void can_interface_send_assi_emergency_from_isr(void);

/* --- tuning ---------------------------------------------------------- */

/* Monitor cadence (SAFETY_PERIOD_MS) and per-task stall deadlines
 * (SAFETY_DEADLINE_*_MS) live in safety_monitor.h as the single source of
 * truth, so the host test suite can assert them against the FS-Rules
 * limits (T11.9.4 ≤ 500 ms). imuTask ticks ~2.5 ms (400 Hz), canTask
 * ~5 ms, app_task ~1-2 ms — 100 ms is many missed iterations: never
 * false-trips on jitter, still fails safe quickly. */

/* Re-emit the ASSI EMERGENCY frame this often while latched, so external
 * systems keep seeing it even if the first frame was lost. */
#define ASSI_RESEND_PERIOD_MS    100u

/* --- shared liveness state (lock-free, single-writer per counter) ---- */

static volatile uint32_t s_heartbeat[SAFETY_NUM_TASKS];
static volatile uint8_t  s_armed[SAFETY_NUM_TASKS];

/* Set from main() when this boot followed an IWDG reset (see
 * safety_flag_watchdog_reset). Makes the supervisor come up latched. */
static volatile uint8_t  s_boot_watchdog_reset = 0u;

void safety_flag_watchdog_reset(void)
{
    s_boot_watchdog_reset = 1u;
}

void safety_heartbeat(safety_task_id_t id)
{
    if ((unsigned)id < (unsigned)SAFETY_NUM_TASKS) {
        s_heartbeat[id]++;
    }
}

void safety_arm(safety_task_id_t id)
{
    if ((unsigned)id < (unsigned)SAFETY_NUM_TASKS) {
        s_armed[id] = 1u;
    }
}

/* --- emergency action ------------------------------------------------ */

/* Drive the vehicle to the rules-defined safe state. Re-asserted every
 * cycle while latched so the state is actively held even if something
 * else perturbs the pins.
 *
 * FS-Rules 2026 define the safe state as (T15.3.5 / T11.9.5):
 *   - brakes engaged (EBS fired), AND
 *   - an OPEN SDC (which, on an EV, also opens the AIRs / kills the TS).
 *
 * So both actions are required for compliance — firing EBS alone is not
 * the safe state:
 *   1. EBS actuators: D1/D2 HIGH (fix/15 polarity, HIGH = fire). The
 *      EBS itself is a passive, mechanical-energy system (T15.2.1).
 *   2. Open the SDC: D4 LOW. This matches dev's hardware_io
 *      (set_as_close_sdc(false) → D4 LOW = open) and fix/15's power-on
 *      GPIO state (D4 LOW), i.e. LOW is the fail-safe "open" level — the
 *      same state a reset/power-loss lands in (T15.2.1).
 *
 * ⚠️  CONFIRM with EE that D4 is the SDC-close line and LOW = open before
 *     bench testing. Driving D4 LOW is low-risk regardless: it is the
 *     power-on/reset level, so worst case (D4 unused on this board) it
 *     is a no-op. */
static void safety_enter_safe_state(void)
{
    /* Brakes: fire EBS. */
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
    /* Open the SDC (→ AIRs open / TS off). Required by T15.3.5/T11.9.5. */
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
}

/* --- supervisor task ------------------------------------------------- */

void StartSafetyTask(void *argument)
{
    (void)argument;

    /* Arm the hardware backstop. From here the IWDG must be refreshed
     * within its timeout or the MCU resets (→ EBS via hardware). */
    iwdg_init();

    task_watch_t watch[SAFETY_NUM_TASKS];
    for (int i = 0; i < SAFETY_NUM_TASKS; ++i) {
        watch[i].deadline_ms    = SAFETY_DEADLINE_IMU_MS; /* default */
        watch[i].last_count     = 0u;
        watch[i].last_change_ms = 0u;
        watch[i].watching       = false;
    }
    watch[SAFETY_TASK_IMU].deadline_ms = SAFETY_DEADLINE_IMU_MS;
    watch[SAFETY_TASK_CAN].deadline_ms = SAFETY_DEADLINE_CAN_MS;
    watch[SAFETY_TASK_APP].deadline_ms = SAFETY_DEADLINE_APP_MS;

    /* Come up already latched if this boot followed an IWDG reset (a
     * prior fatal hang) — main() flagged it via safety_flag_watchdog_reset.
     * We hold the safe state and report EMERGENCY instead of re-arming. */
    uint8_t  emergency_latched = s_boot_watchdog_reset;
    uint32_t last_assi_ms = 0u;

    for (;;) {
        uint32_t now = osKernelGetTickCount();   /* 1 kHz tick → ms */

        uint32_t counts[SAFETY_NUM_TASKS];
        bool     armed[SAFETY_NUM_TASKS];
        for (int i = 0; i < SAFETY_NUM_TASKS; ++i) {
            counts[i] = s_heartbeat[i];
            armed[i]  = (s_armed[i] != 0u);
        }

        int stalled = safety_eval(watch, counts, armed, SAFETY_NUM_TASKS, now);
        if (stalled >= 0) {
            emergency_latched = 1u;   /* latch — never auto-clears */
        }

        if (emergency_latched) {
            safety_enter_safe_state();         /* hold EBS fired + SDC open */
            if ((uint32_t)(now - last_assi_ms) >= ASSI_RESEND_PERIOD_MS) {
                can_interface_send_assi_emergency_from_isr();
                last_assi_ms = now;
            }
        }

        /* Tier-2 backstop: refresh the IWDG as long as THIS task runs.
         * We keep refreshing even after latching emergency so the node
         * stays alive and keeps reporting (matching the old watchdog,
         * which did not reset on an app stall). The IWDG only fires if
         * the monitor itself can no longer run — the catastrophic-hang
         * case software cannot self-detect — and that reset fires the
         * EBS through the hardware fail-safe. */
        iwdg_refresh();

        osDelay(SAFETY_PERIOD_MS);
    }
}
