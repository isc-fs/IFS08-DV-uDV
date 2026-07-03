/**
 * @file safety_eval.h
 * @brief Pure task-stall detection logic for the safety monitor.
 *
 * This header + safety_eval.c contain ZERO HAL / FreeRTOS / CMSIS
 * dependencies on purpose: the stall-detection decision is the
 * safety-critical core of the IWDG watchdog supervisor, so it is kept
 * as a pure function that can be exhaustively unit-tested on the host
 * (see tests/host/test_safety_eval.cpp). The FreeRTOS plumbing
 * (heartbeat counters, IWDG refresh, EBS emergency) lives in
 * safety_monitor.c and only calls into here for the decision.
 *
 * Model: each monitored task increments a heartbeat counter every loop
 * iteration once it has reached steady state ("armed"). The monitor
 * samples the counters periodically; a task whose counter has not
 * advanced within its deadline is "stalled". Tasks that have not armed
 * yet (e.g. imuTask during its ~6 s gyro-bias calibration, or a task
 * still in init) are ignored so boot-time blocking never false-trips.
 */
#ifndef SAFETY_EVAL_H
#define SAFETY_EVAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Per-task watch state. Owned by the monitor; mutated by safety_eval. */
typedef struct {
    uint32_t deadline_ms;     /**< max ms allowed without a heartbeat advance */
    uint32_t last_count;      /**< last observed heartbeat value              */
    uint32_t last_change_ms;  /**< time (ms) the heartbeat last advanced      */
    bool     watching;        /**< true once the task has armed at least once */
} task_watch_t;

/**
 * @brief Evaluate the health of all monitored tasks.
 *
 * For each task i in [0, n):
 *   - If armed[i] is false, the task is ignored (and its watch state is
 *     reset so it re-baselines cleanly if it arms later).
 *   - The first time a task is seen armed, its baseline is captured and
 *     it is treated as healthy for that cycle.
 *   - Thereafter, if counts[i] advanced since last call the deadline
 *     timer is reset; if it has NOT advanced for longer than
 *     deadline_ms, the task is reported stalled.
 *
 * Counter and time values may wrap (uint32) — equality is used for
 * "advanced" and modular subtraction for elapsed time, both wrap-safe.
 *
 * @param w       per-task watch state array (length n), mutated in place
 * @param counts  current heartbeat counters (length n)
 * @param armed   whether each task is in steady state and wants watching
 * @param n       number of tasks
 * @param now_ms  current monotonic time in milliseconds
 * @return index of the first stalled task, or -1 if all watched tasks
 *         are healthy (unwatched tasks never cause a stall).
 */
int safety_eval(task_watch_t *w, const uint32_t *counts,
                const bool *armed, int n, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_EVAL_H */
