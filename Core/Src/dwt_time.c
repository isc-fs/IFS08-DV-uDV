/**
 ******************************************************************************
 * @file    dwt_time.c
 * @brief   DWT cycle-counter microsecond timestamp helper.
 *
 * Extracted from freertos.c so both the IMU task and the ROS task can share a
 * single implementation without bloating the CubeMX-generated file.
 ******************************************************************************
 */
#include "dwt_time.h"
#include "main.h"   /* DWT, CoreDebug, SystemCoreClock (CMSIS) */

/* Wrap-tracking state. Shared by all callers — same (non-reentrant) behaviour
 * as the original static-inline dwt_micros() in freertos.c. */
static uint32_t dwt_last = 0;
static uint64_t dwt_overflow_count = 0;

uint64_t dwt_micros(void)
{
  uint32_t now = DWT->CYCCNT;
  if (now < dwt_last) dwt_overflow_count++;
  dwt_last = now;
  uint64_t total_cycles = (dwt_overflow_count << 32) | (uint64_t)now;
  return total_cycles / (SystemCoreClock / 1000000U);
}
