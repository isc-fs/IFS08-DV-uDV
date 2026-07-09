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

/* Bit positions in the word returned by ros_get_state_signals(), one per
 * StateManagerSignals field (see state_manager.hpp). Shared here so the C++
 * packer (ros_globals.cpp) and the C unpacker (ros_task.c) cannot drift. */
enum {
    AS_SIG_ASMS_ON        = 1u << 0,
    AS_SIG_TS_ACTIVE      = 1u << 1,
    AS_SIG_SDC_RES_OPEN   = 1u << 2,
    AS_SIG_EBS_ACTIVATED  = 1u << 3,
    AS_SIG_ABS_CHECKS_OK  = 1u << 4,
    AS_SIG_BRAKES_ENGAGED = 1u << 5,
    AS_SIG_MISSION_SEL    = 1u << 6,
    AS_SIG_R2D            = 1u << 7,
    AS_SIG_STANDSTILL     = 1u << 8,
    AS_SIG_MISSION_DONE   = 1u << 9,
};

/* Snapshot of the 10 StateManagerSignals booleans packed into one word using
 * the AS_SIG_* bits above. Same AppTask telemetry source as ros_get_as_state. */
uint16_t ros_get_state_signals(void);

/* Raw EBSInitState (see ebs_manager.hpp): Start=0 .. Failed=7, Done=8. */
uint8_t ros_get_ebs_init_state(void);

/* EBS air-tank storage pressures in deci-bar (bar x10, signed), clamped to
 * int16. AppTask samples the real ADC; valid even when BENCH_STUB_EBS_SENSORS
 * fakes the pass/fail gate. */
int16_t ros_get_ebs_pressure1_dbar(void);
int16_t ros_get_ebs_pressure2_dbar(void);

#ifdef __cplusplus
}
#endif

#endif /* AS_STATE_H */
