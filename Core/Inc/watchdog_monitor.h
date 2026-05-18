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
 * @brief Global flag set by ISR when watchdog expires
 *        App task should check this and handle emergency
 */
extern volatile bool g_watchdog_triggered;

#ifdef __cplusplus
}
#endif

#endif /* INC_WATCHDOG_MONITOR_H_ */
