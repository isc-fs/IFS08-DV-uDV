/**
 * @file    dwt_time.c
 * @brief   Shared high-resolution µs timer using the DWT cycle counter.
 *          See dwt_time.h for the design rationale.
 */

#include "dwt_time.h"
#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

/* Wrap-tracking state.  Updated under a FreeRTOS critical section so
 * concurrent callers from different tasks can't corrupt the 64-bit
 * overflow count or miss a wrap. */
static volatile uint32_t s_dwt_last           = 0;
static volatile uint64_t s_dwt_overflow_count = 0;

void dwt_init(void)
{
    /* Enable trace + DWT cycle counter.  Idempotent — safe to call again. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    s_dwt_last           = 0;
    s_dwt_overflow_count = 0;
}

uint64_t dwt_micros(void)
{
    uint32_t now;
    uint64_t total_cycles;

    /* Atomically read DWT->CYCCNT and update the wrap counter so that two
     * tasks calling dwt_micros() simultaneously can't both miss the same
     * wrap event.  taskENTER_CRITICAL disables interrupts on the current
     * core for ~10 cycles — negligible vs the 400 Hz IMU period. */
    taskENTER_CRITICAL();
    now = DWT->CYCCNT;
    if (now < s_dwt_last)
    {
        s_dwt_overflow_count++;
    }
    s_dwt_last   = now;
    total_cycles = ((uint64_t)s_dwt_overflow_count << 32) | (uint64_t)now;
    taskEXIT_CRITICAL();

    return total_cycles / (SystemCoreClock / 1000000U);
}
