/**
 * @file    dwt_time.h
 * @brief   Shared high-resolution µs timer using the DWT cycle counter.
 *
 * Single source of truth for microsecond-resolution local time across the
 * firmware.  Replaces the two previously-duplicated dwt_micros() copies in
 * imu_task.cpp and ros_interface.cpp (each with its own overflow counter
 * — those would drift apart because they were called at different rates,
 * and the slow caller (ros_interface, ~10 s) could miss DWT wraps
 * (DWT->CYCCNT is 32 bits at the CPU clock; on this board that's ~8.13 s
 * wrap period at 528 MHz).
 *
 * Call dwt_init() once from main.c before the FreeRTOS scheduler starts.
 * After that, dwt_micros() is safe to call from any task context.  Not
 * intended to be called from ISRs.
 */

#ifndef INC_DWT_TIME_H_
#define INC_DWT_TIME_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Enable the DWT cycle counter and reset the wrap-tracking state.
 *         Must be called once from main(), before the scheduler starts and
 *         before any task is allowed to call dwt_micros().
 */
void dwt_init(void);

/**
 * @brief  Return a monotonically-increasing microsecond timestamp.
 *
 * Internally combines a 32-bit DWT->CYCCNT reading with a 32-bit overflow
 * counter to form a 64-bit cycle count; divides by SystemCoreClock/1e6
 * for microseconds.  Thread-safe via a FreeRTOS critical section — every
 * caller contributes to the shared overflow counter, so even a slow caller
 * never misses a wrap as long as at least one caller fires often enough
 * (the 400 Hz IMU task is the guaranteed observer here).
 *
 * @retval Microseconds since dwt_init(), 64-bit, wrap-safe for ~580k years.
 */
uint64_t dwt_micros(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DWT_TIME_H_ */
