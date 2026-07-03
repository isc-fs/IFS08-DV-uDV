/* Host-test stub: just enough of FreeRTOS.h for can_globals.h /
 * ros_globals.h to compile off-target (no scheduler — the state machine
 * and EBS manager logic under test never call into FreeRTOS). */
#pragma once
#include <stdint.h>
#include <stddef.h>
