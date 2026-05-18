/**
 * @file    watchdog_monitor.h
 * @brief   Hardware timer-based watchdog for detecting app stalls
 * @details Independent timer ISR monitors if app is responsive.
 *          If no kick for >50ms, ISR triggers emergency:
 *          - Activates EBS (brakes)
 *          - Opens SDC (disconnects power)
 *          - Notifies ASMS task (emergency lights)
 *          - Sets flag for app_task (state machine update)
 */

#ifndef INC_WATCHDOG_MONITOR_H_
#define INC_WATCHDOG_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize hardware watchdog timer (50ms timeout on dedicated timer)
 */
void watchdog_monitor_init(void);

/**
 * @brief Kick/refresh the watchdog timer (call every loop iteration)
 *        Resets the hardware timer countdown
 */
void watchdog_monitor_kick(void);

/**
 * @brief Notify that watchdog has been triggered (to be called from ISR)
 */
void watchdog_set_triggered(void);

/**
 * @brief Query whether watchdog has been triggered.
 * @return true if the watchdog has been triggered.
 */
bool watchdog_is_triggered(void);

/**
 * @brief Clear the watchdog triggered flag (set to false).
 */
void watchdog_clear_triggered(void);

/**
 * @brief Atomically consume the watchdog triggered flag (read-and-clear).
 * @return true if the watchdog had been triggered prior to clear.
 */
bool watchdog_consume_triggered(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_WATCHDOG_MONITOR_H_ */
