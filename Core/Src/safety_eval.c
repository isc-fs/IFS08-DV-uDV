/**
 * @file safety_eval.c
 * @brief Pure task-stall detection logic. No HAL / FreeRTOS / CMSIS.
 *
 * See safety_eval.h for the model. Kept dependency-free so it builds and
 * unit-tests on the host exactly as it runs on the target.
 */
#include "safety_eval.h"

int safety_eval(task_watch_t *w, const uint32_t *counts,
                const bool *armed, int n, uint32_t now_ms)
{
    for (int i = 0; i < n; ++i) {
        if (!armed[i]) {
            /* Not in steady state yet (or disarmed): ignore and force a
             * fresh baseline if it arms later. */
            w[i].watching = false;
            continue;
        }

        if (!w[i].watching) {
            /* First cycle this task is armed: capture baseline, healthy. */
            w[i].watching      = true;
            w[i].last_count    = counts[i];
            w[i].last_change_ms = now_ms;
            continue;
        }

        if (counts[i] != w[i].last_count) {
            /* Heartbeat advanced — reset the deadline timer. */
            w[i].last_count    = counts[i];
            w[i].last_change_ms = now_ms;
        } else {
            /* No advance: wrap-safe elapsed-time check. Strictly greater
             * than the deadline counts as stalled (== is still alive). */
            uint32_t elapsed = now_ms - w[i].last_change_ms;
            if (elapsed > w[i].deadline_ms) {
                return i;
            }
        }
    }
    return -1;
}
