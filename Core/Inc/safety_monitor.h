/**
 * @file safety_monitor.h
 * @brief Two-tier safety watchdog supervisor (FreeRTOS + IWDG).
 *
 * Replaces the old TIM3 software-timer watchdog with the STM32H7
 * hardware IWDG plus a software task-health monitor:
 *
 *   Tier 1 — software monitor (this task). Watches the safety-critical
 *     real-time tasks via heartbeat counters. If one stalls past its
 *     deadline it latches the graceful emergency (assert EBS, emit ASSI
 *     EMERGENCY on CAN) — the behaviour the old TIM3 ISR provided.
 *
 *   Tier 2 — IWDG hardware backstop (iwdg.c). The monitor refreshes the
 *     IWDG every cycle; if the monitor task itself can no longer run
 *     (scheduler dead, interrupts wedged, CPU locked) the IWDG resets
 *     the MCU and the EBS fires via the hardware fail-safe. This is the
 *     failure mode a software-only watchdog cannot catch.
 *
 * Monitored tasks self-arm once they reach steady state, so boot-time
 * blocking (imuTask's ~6 s gyro-bias calibration, the micro-ROS task
 * waiting on the agent) never false-trips. The micro-ROS task is not
 * monitored — it legitimately blocks for unbounded periods.
 */
#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/** Identifiers for the safety-critical tasks the monitor watches. */
typedef enum {
    SAFETY_TASK_IMU = 0,  /**< imuTask 400 Hz sample loop   */
    SAFETY_TASK_CAN = 1,  /**< canTask RX/TX service loop   */
    SAFETY_TASK_APP = 2,  /**< app_task state-machine loop  */
    SAFETY_NUM_TASKS      /**< count — keep last            */
} safety_task_id_t;

/**
 * @brief Record one liveness beat for a monitored task.
 * Call once per loop iteration from the task's steady-state loop.
 * Lock-free (single-writer 32-bit counter).
 */
void safety_heartbeat(safety_task_id_t id);

/**
 * @brief Mark a task as having reached steady state (start watching it).
 * Call once, after the task's init/calibration completes and just
 * before its real-time loop. Until armed, a task is ignored by the
 * monitor (so long blocking init never trips the watchdog).
 */
void safety_arm(safety_task_id_t id);

/**
 * @brief Record that this boot followed an IWDG watchdog reset.
 * Called once from main() reset-cause detection (before the scheduler
 * starts) when RCC_FLAG_IWDG1RST was set. Makes the safety task come up
 * already LATCHED in emergency — a watchdog reset means a prior fatal
 * hang, so we must not silently re-arm into normal operation.
 */
void safety_flag_watchdog_reset(void);

/**
 * @brief FreeRTOS entry point for the safety supervisor task.
 * Starts the IWDG, then loops: evaluate task health, refresh the IWDG,
 * and latch the graceful emergency on a stall.
 */
void StartSafetyTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
