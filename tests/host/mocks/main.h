/*
 * Stand-in for Core/Inc/main.h on the host. The real main.h drags in
 * peripheral handle externs and pin defines we don't need for pure-logic
 * tests. We only forward the HAL types ws2812.h cares about.
 */
#ifndef MAIN_H_TEST_SHIM
#define MAIN_H_TEST_SHIM

#include "stm32h7xx_hal.h"

#endif
