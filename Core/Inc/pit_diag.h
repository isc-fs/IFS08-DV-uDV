/**
 * @file    pit_diag.h
 * @brief   Pit-diag: CAN-only observability stream on FDCAN2 (ACU bus).
 *
 * Re-emits the uDV's internal state as CAN frames so a bench with just a CAN
 * tool can read it — no ROS / USB / DVPC / agent needed (that is exactly the
 * gap /debug can't cover on a bare bench). Modelled on the ECU (0x700-0x707 /
 * 0x7E0) and AMS (0x6C0-0x6C9) pit-diag. See docs/PIT_DIAG_STUDY.md.
 *
 * IDs sit in a gap between the ECU and AMS ranges on the shared ACU bus, and
 * are LOW priority so the diag stream never wins arbitration over a safety
 * frame. Gated by an arm frame (sticky) so the car bus stays quiet until a pit
 * tool enables it; PITDIAG_STREAM_ALWAYS forces it on for the bench.
 */
#ifndef PIT_DIAG_H
#define PIT_DIAG_H

#include <stdint.h>

/* TX frames (uDV -> pit tool), ~10 Hz except fw-info (~1 Hz). */
#define CAN_ID_PITDIAG_STATUS  0x7A0u  /* AS state, signals, mission, EBS-init, stubs */
#define CAN_ID_PITDIAG_RES     0x7A1u  /* raw 0x191, RES status/bits, radio, age, steer */
#define CAN_ID_PITDIAG_PIPE    0x7A2u  /* dv/status + age, ctrl cmds + age, setup bits */
#define CAN_ID_PITDIAG_HEALTH  0x7A3u  /* heap, task mask, reset cause, uptime, latched */
#define CAN_ID_PITDIAG_FWINFO  0x7A4u  /* git hash, stub mask, heap size (~1 Hz) */
#define CAN_ID_PITDIAG_CANHEALTH 0x7A5u /* FDCAN1 protocol status + err counters + RES rx/NMT counts */
#define CAN_ID_PITDIAG_CALIB   0x7A6u  /* steering end-stop calib status relay (#113) */
#define CAN_ID_PITDIAG_STEER   0x7A7u  /* live steering: LWS raw + actual/target angle, LWS+motor state */
#define CAN_ID_PITDIAG_CALIBDBG 0x7A8u /* uDV calib-relay diag: 0x7DF rx count, 0x522 relay count, armed */
#define CAN_ID_PITDIAG_EBS     0x7A9u  /* EBS tank pressures (2x int16 dbar), init-state, stub mask, ok-bits */

/* RX arm frame (pit tool -> uDV): 4-byte magic, big-endian. */
#define CAN_ID_PITDIAG_ARM     0x7DEu
#define PITDIAG_ARM_MAGIC      0xDEADBEEFu
/* RX steering-calibration trigger (pit tool -> uDV): byte0 1=start / 2=abort.
 * Honoured only while the stream is armed (see pit_diag_is_armed). */
#define CAN_ID_PITDIAG_CALIB_TRIGGER 0x7DFu

/* Bench convenience: stream unconditionally (no arm frame needed). Keep 0 for
 * the car so the ACU bus stays quiet until a pit tool arms it. */
#ifndef PITDIAG_STREAM_ALWAYS
#define PITDIAG_STREAM_ALWAYS  0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Called from rx_dispatch for a frame whose id == CAN_ID_PITDIAG_ARM.
 * A correct magic sticky-enables the stream for the rest of the power cycle. */
void pit_diag_arm_from_can(const uint8_t *data, uint8_t dlc);

/* Called from canTask (~10 Hz). Builds + TXs the diag frames on FDCAN2 when
 * enabled (armed, or PITDIAG_STREAM_ALWAYS). No-op otherwise. */
void pit_diag_service(uint32_t now_ms);

/* Is the diag stream armed? Gates the operator steering-calibration trigger so
 * calibration can only be kicked off from a deliberate pit-diag session (#113). */
uint8_t pit_diag_is_armed(void);

#ifdef __cplusplus
}
#endif

#endif /* PIT_DIAG_H */
