/**
 * @file test_rules_compliance.cpp
 * @brief Pin the watchdog TIMING BUDGET against the FS-Rules limits.
 *
 * test_safety_eval proves the stall-detection LOGIC is correct; this file
 * proves the production NUMBERS are rules-compliant. The constants live in
 * iwdg.h / safety_monitor.h (HAL-free), so they are the exact values the
 * firmware ships. Most checks are static_assert — a future edit that
 * pushes a deadline or the IWDG timeout out of the rules budget fails the
 * BUILD, not just the run.
 *
 * Rules referenced (FS-Rules 2026):
 *   T11.9.2(d) — loss/delay of a digital SCS detected by a timeout.
 *   T11.9.4    — that detection delay must not exceed 500 ms.
 *   T15.3.2    — ASB + its SCS continuously monitored for failures.
 *   T15.3.4/5  — auto-transition to the safe state (this is what the
 *                detected stall triggers in safety_monitor.c).
 *
 * NOT covered here (needs the bench, see docs/WATCHDOG_AND_STATE_MACHINE.md):
 *   the EBS reaction time T15.4.1 (≤200 ms, SDC-open→deceleration) and the
 *   ≥10 m/s² deceleration are properties of the EBS hydraulics, not firmware.
 */
#include <cstdio>

extern "C" {
#include "safety_eval.h"
#include "safety_monitor.h"
#include "iwdg.h"
}

/* ---- compile-time guarantees -------------------------------------------- */

/* T11.9.4: every per-task stall deadline must trigger the safe state well
 * within the 500 ms message-loss detection bound. */
static_assert(SAFETY_DEADLINE_IMU_MS <= FS_RULES_T11_9_4_MAX_DETECT_MS,
              "IMU stall deadline exceeds FS-Rules T11.9.4 (500 ms)");
static_assert(SAFETY_DEADLINE_CAN_MS <= FS_RULES_T11_9_4_MAX_DETECT_MS,
              "CAN stall deadline exceeds FS-Rules T11.9.4 (500 ms)");
static_assert(SAFETY_DEADLINE_APP_MS <= FS_RULES_T11_9_4_MAX_DETECT_MS,
              "app stall deadline exceeds FS-Rules T11.9.4 (500 ms)");

/* The IWDG total-hang backstop must also reset within the 500 ms bound. */
static_assert(IWDG_TIMEOUT_MS_NOMINAL <= FS_RULES_T11_9_4_MAX_DETECT_MS,
              "IWDG timeout exceeds FS-Rules T11.9.4 (500 ms)");

/* A healthy monitor must refresh the IWDG with comfortable margin: the
 * monitor runs every SAFETY_PERIOD_MS, so the IWDG window must be several
 * times larger or scheduling jitter could trip a spurious reset. */
static_assert(IWDG_TIMEOUT_MS_NOMINAL >= 5u * SAFETY_PERIOD_MS,
              "IWDG timeout too tight vs the monitor refresh period");

/* The monitor must sample faster than the shortest deadline it enforces. */
static_assert(SAFETY_PERIOD_MS < SAFETY_DEADLINE_IMU_MS, "monitor too slow");
static_assert(SAFETY_PERIOD_MS < SAFETY_DEADLINE_CAN_MS, "monitor too slow");
static_assert(SAFETY_PERIOD_MS < SAFETY_DEADLINE_APP_MS, "monitor too slow");

/* Correctness: the prescaler divider must match the PR register encoding
 * (div = 4 * 2^PR), or IWDG_TIMEOUT_MS_NOMINAL is computed from a lie. */
static_assert(IWDG_PRESCALER_DIV == (4u << IWDG_PRESCALER_PR),
              "IWDG_PRESCALER_DIV inconsistent with IWDG_PRESCALER_PR");

/* Sanity: the shipped config yields the documented ~100 ms. */
static_assert(IWDG_TIMEOUT_MS_NOMINAL == 100u,
              "IWDG nominal timeout drifted from the documented 100 ms");

/* ---- runtime functional check ------------------------------------------- */

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

/* Drive the real safety_eval with the PRODUCTION deadline and confirm a
 * stalled task is actually flagged within the T11.9.4 bound — i.e. the
 * logic + the shipped number together satisfy the rule. */
static void test_production_deadline_detects_within_t11_9_4(void)
{
    task_watch_t w[1];
    w[0].deadline_ms = SAFETY_DEADLINE_CAN_MS;  /* the shipped value */
    w[0].last_count = 0; w[0].last_change_ms = 0; w[0].watching = false;

    bool armed[1] = { true };
    uint32_t counts[1] = { 7 };  /* frozen heartbeat = stalled task */

    CHECK(safety_eval(w, counts, armed, 1, 0) == -1);              /* baseline */
    /* Healthy right up to the deadline... */
    CHECK(safety_eval(w, counts, armed, 1, SAFETY_DEADLINE_CAN_MS) == -1);
    /* ...stalled one ms past it... */
    CHECK(safety_eval(w, counts, armed, 1, SAFETY_DEADLINE_CAN_MS + 1u) == 0);
    /* ...and that detection point is within the rules bound. */
    CHECK((SAFETY_DEADLINE_CAN_MS + 1u) <= FS_RULES_T11_9_4_MAX_DETECT_MS);
}

int main(void)
{
    std::printf("compile-time budget asserts passed "
                "(IWDG nominal = %u ms, deadlines = %u/%u/%u ms, "
                "T11.9.4 limit = %u ms)\n",
                (unsigned)IWDG_TIMEOUT_MS_NOMINAL,
                (unsigned)SAFETY_DEADLINE_IMU_MS,
                (unsigned)SAFETY_DEADLINE_CAN_MS,
                (unsigned)SAFETY_DEADLINE_APP_MS,
                (unsigned)FS_RULES_T11_9_4_MAX_DETECT_MS);

    test_production_deadline_detects_within_t11_9_4();

    std::printf("\nrules_compliance: %d checks, %d failures\n",
                g_checks, g_failures);
    if (g_failures == 0) {
        std::printf("All tests green.\n");
        return 0;
    }
    return 1;
}
