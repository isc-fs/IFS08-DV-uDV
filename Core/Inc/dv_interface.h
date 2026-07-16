/**
 * @file    dv_interface.h
 * @brief   Stock-typed uDV <-> DV-pipeline (mission_control) interface.
 *
 * Firmware-side mirror of the pipeline's single source of truth,
 * IFS08-DV-PIPELINE `mission_control/interface_contract.py`. The DV
 * pipeline (the "DVPC") and this firmware exchange ONLY standard ROS 2
 * message types, so no custom messages and no micro-ROS library recompile
 * are needed. Nobody "calls" anybody: both sides continuously broadcast a
 * status byte and react to the other's.
 *
 *   uDV -> pipeline (uplink, published by ros_task):
 *     assi/state    std_msgs/UInt8   AS state byte (FS-Rules T14.9)
 *     ami/mission   std_msgs/Int32   selected AMI mission INDEX (0..9 raw)
 *
 *   pipeline -> uDV (downlink):
 *     dv/status     std_msgs/UInt8        pipeline lifecycle byte (below)
 *     ctrl/cmd      geometry_msgs/Twist   linear.x = throttle  [-1, 1]
 *                                         angular.z = steering [-1, 1]
 *     force_ebs     std_srvs/SetBool      emergency-brake request (served
 *                                         by the uDV; see force_ebs_callback)
 *
 * Both UInt8 byte topics are each other's liveness heartbeat and must be
 * published at a steady cadence (>= ~2 Hz, not edge-only): a stale
 * dv/status holds the uDV in a safe state; a stale assi/state deactivates
 * the pipeline. The uDV OWNS the AS state machine and the actuation gate
 * (it only applies ctrl/cmd while in AS Driving); mission_control only
 * reacts to assi/state + ami/mission and answers with dv/status.
 */
#ifndef DV_INTERFACE_H
#define DV_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* AS state bytes on assi/state (FS-Rules T14.9). MUST match the encoding
 * emitted by Can::sendAssiStatus / StateManager::getAssiStatusCode and the
 * pipeline's interface_contract.AS_*. NOTE this is the ASSI *indicator*
 * code, distinct from the raw ASState enum (as_state.h). */
#define AS_STATE_OFF        0u
#define AS_STATE_EMERGENCY  1u
#define AS_STATE_READY      2u
#define AS_STATE_DRIVING    3u
#define AS_STATE_FINISHED   4u

/* dv/status bytes — the pipeline's own lifecycle, reported back as the
 * prepare/run handshake. The uDV gates AS Ready->Driving on DV_STATUS_READY
 * and reacts to FINISHED / EMERGENCY / FAILED. Mirrors interface_contract.DV_*. */
#define DV_STATUS_IDLE      0u   /* nothing prepared                    */
#define DV_STATUS_PREPARING 1u   /* configure / JIT in flight           */
#define DV_STATUS_READY     2u   /* prepared OK (the "go" precondition) */
#define DV_STATUS_RUNNING   3u   /* activated, streaming ctrl/cmd       */
#define DV_STATUS_FINISHED  4u   /* mission complete                    */
#define DV_STATUS_EMERGENCY 5u   /* pipeline raised EBS                 */
#define DV_STATUS_FAILED    6u   /* prepare/activate error              */
/* End-of-mission stop (issue #176). The mission is over and the car must be
 * brought to a standstill so AS Finished — which the rules only allow at a
 * standstill — becomes reachable. NOT an emergency: the SDC stays CLOSED and
 * the AS state stays DRIVING; the pipeline follows with DV_STATUS_FINISHED
 * once the car has actually stopped, and that byte performs the normal
 * DRIVING->FINISHED transition (EBS + SDC open).
 *
 * This is deliberately NOT a service brake. The IFS08 has no pipeline-
 * controllable service brake; the only brake pneumatics are the two EBS
 * actuators, which are binary (fire / release, no modulation). So this byte
 * buys exactly one thing — the final stop that closes out a mission — and must
 * not be used to modulate speed during a run. See docs/PIPELINE_INTERFACE.md.
 */
#define DV_STATUS_STOPPING  7u   /* brake to standstill, SDC stays closed */

/* Interface topic / service names (empty-namespace node cubemx_node). */
#define DV_TOPIC_ASSI_STATE   "assi/state"
#define DV_TOPIC_AMI_MISSION  "ami/mission"
#define DV_TOPIC_DV_STATUS    "dv/status"
#define DV_TOPIC_CTRL_CMD     "ctrl/cmd"
#define DV_SERVICE_FORCE_EBS  "force_ebs"

/* Liveness windows (ms). dv/status is published at ~10 Hz (100 ms period),
 * so 400 ms = 4 missed cycles: jitter-safe, yet under the FS-Rules T11.9.4
 * cap of 500 ms to detect a lost safety-critical message and enter the safe
 * state. Coordinated with the pipeline, which watches assi/state on the same
 * 400 ms window (_ASSI_STALE_S in mission_control_node); both sides publish
 * at 10 Hz and time out at 4 missed cycles, so on a link loss each drops to
 * a safe state within the rules bound. */
#define DV_STATUS_STALE_MS   400u
/* FS-Rules T11.9.4 cap (ms): a lost safety-critical message must be detected
 * and the safe state entered within this bound. DV_STATUS_STALE_MS must stay
 * strictly under it. Mirrors HEARTBEAT_STALE_CAP_S in the pipeline's
 * interface_contract.py (0.5 s); the two repos have no build-time link, so
 * the static_assert below is the firmware-side guard against silent drift. */
#define DV_STATUS_STALE_CAP_MS 500u
#if defined(__cplusplus)
static_assert(DV_STATUS_STALE_MS < DV_STATUS_STALE_CAP_MS,
              "DV_STATUS_STALE_MS must stay under the FS-Rules T11.9.4 500 ms "
              "detect-and-safe cap (keep in lockstep with the pipeline's "
              "HEARTBEAT_STALE_S in interface_contract.py)");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(DV_STATUS_STALE_MS < DV_STATUS_STALE_CAP_MS,
               "DV_STATUS_STALE_MS must stay under the FS-Rules T11.9.4 500 ms "
               "detect-and-safe cap (keep in lockstep with the pipeline's "
               "HEARTBEAT_STALE_S in interface_contract.py)");
#endif
/* While AS Driving, zero actuation if ctrl/cmd stops arriving within this
 * window, so a dropped link can never latch the last throttle/steering. */
#define DV_CTRL_CMD_STALE_MS  500u

/* C-callable bridge between the C micro-ROS callbacks (ros_task.c) and the
 * C++ command atomics (ros_globals.cpp). ros_task.c is compiled as C and
 * cannot touch the std::atomic<> globals directly, so it routes through
 * these — the same idiom as ros_get_as_state() / can_c_get_*(). */

/* Store a normalised ctrl/cmd. throttle/steering are clamped to [-1, 1];
 * now_ms (osKernelGetTickCount) timestamps it for the AppTask staleness
 * gate. Physical scaling + sign live downstream / on the AppTask send
 * path — see the ctrl/cmd handling in app_task.cpp. */
void    ros_set_ctrl_cmd_norm(float throttle_norm, float steering_norm, uint32_t now_ms);

/* Store the latest dv/status byte + its arrival time (osKernelGetTickCount). */
void    ros_set_dv_status(uint8_t status, uint32_t now_ms);

/* Latest dv/status byte (raw; caller checks freshness separately). */
uint8_t ros_get_dv_status(void);

#ifdef __cplusplus
}
#endif

#endif /* DV_INTERFACE_H */
