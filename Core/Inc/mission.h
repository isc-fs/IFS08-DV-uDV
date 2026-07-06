/**
 * @file    mission.h
 * @brief   Per-mission dispatch interface — the seam between the AS state
 *          machine (app_task) and each mission's actuation logic.
 *
 * The AMI board sends the 0-based mission menu index on CAN 0x503 byte[0]
 * (see IFS08-DV_AMI Core/Src/ami.c `missions[]`). app_task resolves that code
 * to a `const Mission*` via mission_for_code() ONCE per tick and drives it
 * through this vtable. Each mission lives in its own translation unit
 * (mission_inspection.cpp, mission_pipeline.cpp, ...) so its control law is
 * isolated and host-unit-testable in the same dependency-free way the AS
 * transition core lives in as_transition.hpp.
 *
 * DESIGN — missions are PURE, like as_transition.hpp. A mission body never
 * touches the CAN driver, the RTOS or the ROS globals: on_tick() consumes an
 * immutable MissionCtx snapshot and RETURNS a MissionCommand describing what
 * to actuate; app_task applies it via Can::. This keeps every mission testable
 * off-target with no HAL/CAN/RTOS stubs (link the mission .cpp + a plain test).
 *
 * Classification: `needs_pipeline` is a DATA field, not a call, because the AS
 * transition (as_transition.hpp) reads it in READY to gate GO and to arm the
 * lost-heartbeat rule — it must be known BEFORE DRIVING, so it cannot be an
 * on_tick-time query. Pipeline missions (accel/skidpad/autocross/trackdrive)
 * are driven by the DV pipeline over /ctrl/cmd; standalone ones (inspection,
 * EBS test) run open-loop here with no pipeline gate.
 */
#ifndef MISSION_H
#define MISSION_H

#include <cstdint>

/* AMI mission codes — the 0-based menu index the AMI board transmits on CAN
 * 0x503 byte[0] (IFS08-DV_AMI ami.c `missions[]`). Codes 8/9 ("aux1"/"aux2")
 * exist in the AMI menu but have no uDV mission and resolve to nullptr. */
enum AmiMission {
    MISSION_MANUAL     = 0,
    MISSION_ACCEL      = 1,
    MISSION_SKIDPAD    = 2,
    MISSION_AUTOCROSS  = 3,
    MISSION_TRACKDRIVE = 4,
    MISSION_EBS_TEST   = 5,
    MISSION_INSPECTION = 6,
    MISSION_SHUTDOWN   = 7,
};

/**
 * Immutable per-tick snapshot handed to a mission. app_task builds it from the
 * mission clock and the latched pipeline command; a mission reads it and reads
 * nothing else, so its behaviour is a pure function of these fields.
 */
struct MissionCtx {
    uint32_t now_ms;             /**< osKernelGetTickCount() this tick          */
    uint32_t mission_elapsed_ms; /**< ms since the READY->DRIVING (GO) edge      */
    bool     ctrl_cmd_fresh;     /**< /ctrl/cmd seen within DV_CTRL_CMD_STALE_MS  */
    float    ctrl_accel;         /**< latest normalised throttle  [-1, 1]        */
    float    ctrl_steer;         /**< latest normalised steering  [-1, 1]        */
};

/**
 * Actuation a mission wants applied THIS tick. Default-init = emit nothing.
 * Actuator paths (app_task applies each iff its `send_*` flag is set):
 *   - send_steer_angle: absolute 0x020 steering angle, at the mission's own
 *     cadence -> Can::sendSteeringAngle. Used by inspection / EBS test.
 *   - send_accel: normalised ECU torque on 0x507 (pipeline missions), paced by
 *     app_task to the ECU's 20 ms cycle -> Can::sendAccel.
 *   - send_steer: normalised pipeline steering, applied as an absolute 0x020
 *     angle (norm * STEER_FULL_LOCK_DEG), same 20 ms pace. Kept separate from
 *     send_accel so a stale link can zero torque WITHOUT commanding steering (a
 *     0x020 zero would snap the wheel to center; the old 0x508 frame is gone).
 */
struct MissionCommand {
    bool  send_steer_angle;  /**< emit an absolute steering-angle command (0x020) */
    float steer_angle_deg;   /**< degrees (steering firmware clamps to ±60)       */
    bool  send_accel;        /**< emit normalised ECU torque (0x507)              */
    float accel_norm;        /**< normalised throttle [-1, 1]                     */
    bool  send_steer;        /**< emit normalised pipeline steering (-> 0x020)    */
    float steer_norm;        /**< normalised steering [-1, 1]                     */
};

/**
 * A mission's behaviour, as a small vtable app_task dispatches to. Every
 * callback except on_tick may be nullptr (a no-op mission is all-nullptr):
 *   on_enter    — called once on the READY->DRIVING (GO) edge (reset state)
 *   on_tick     — called every DRIVING tick; returns the actuation to apply
 *   is_complete — polled every DRIVING tick; true -> DRIVING to FINISHED. A
 *                 mission that ends via the pipeline (dv/status FINISHED, e.g.
 *                 trackdrive) leaves this nullptr so it never double-signals.
 *   on_exit     — called once on the ->FINISHED edge
 */
struct Mission {
    const char*    name;
    bool           needs_pipeline;
    void           (*on_enter)(const MissionCtx* ctx);
    MissionCommand (*on_tick)(const MissionCtx* ctx);
    bool           (*is_complete)(const MissionCtx* ctx);
    void           (*on_exit)(const MissionCtx* ctx);
};

/**
 * Resolve an AMI mission code (0x503 byte[0]) to its Mission, or nullptr if the
 * code is out of range or has no uDV mission (SHUTDOWN / aux). app_task refuses
 * the READY->DRIVING edge for a nullptr mission, so an unknown code can never
 * release the brakes with no mission running.
 */
const Mission* mission_for_code(int ami_code);

#endif /* MISSION_H */
