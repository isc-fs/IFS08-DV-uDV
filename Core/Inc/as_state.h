#ifndef AS_STATE_H
#define AS_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Raw AS state-machine state (ASState enum, see state_manager.hpp):
 *   OFF=0  READY=1  DRIVING=2  EMERGENCY=3  FINISHED=4
 *
 * Snapshot owned by AppTask (g_telemetry_as_state), refreshed every state
 * machine tick. C-callable so the C ROS task and the C++ CAN layer share
 * ONE source of truth. This is the raw state — NOT the ASSI indicator
 * code on /assi/state, and NOT the DV-logger 0x502 nibble; those are
 * separate encodings derived from this value. Defined in ros_globals.cpp.
 */
uint8_t ros_get_as_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AS_STATE_H */
