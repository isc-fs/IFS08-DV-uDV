/**
 * @file    watchdog_monitor.c
 * @brief   Hardware timer-based watchdog implementation
 * @details Uses a dedicated timer with ISR to detect app stalls.
 *          Timer overflow triggers emergency without MCU reset.
 */

#include "watchdog_monitor.h"
#include "main.h"
#include "hardware_io.h"

// Global flag set by ISR when watchdog expires
volatile bool g_watchdog_triggered = false;

// Timer handle (provided by CubeMX-generated tim.c)
extern TIM_HandleTypeDef htim3;  // TIM3 configured as watchdog timer

void watchdog_monitor_init(void)
{
    // Timer is already initialized by MX_TIM3_Init() from CubeMX
    // Start the timer counting down
    HAL_TIM_Base_Start_IT(&htim3);
}

void watchdog_monitor_kick(void)
{
    // Reset the timer counter to start from ARR again
    // This prevents the timer from expiring
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

