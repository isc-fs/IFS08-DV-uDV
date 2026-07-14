/**
 * @file    emergency_centering.hpp
 * @brief   Pure emergency steering-centering ramp (host-unit-testable).
 *
 * When the AS state machine enters EMERGENCY the car is braking to a stop under
 * the EBS. This helper decides, once per control tick, what steering command
 * the uDV should send so the wheels are brought SMOOTHLY back to centre while
 * the car is still rolling, and are then LEFT WHERE THEY ARE once it is at
 * standstill — moving a loaded steering rack with the car stopped only scrubs
 * the tyres and risks stepper step-loss, and buys nothing.
 *
 * Design: the ramp is RATE-limited, not duration-limited. A bounded angular
 * rate is the physically meaningful quantity (tyre scrub, stepper load,
 * predictability) and is independent of how far off-centre the wheel started —
 * unlike a fixed total time, which would make a full-lock wheel slew fastest,
 * exactly the case you want to be gentlest. The rate is expressed through an
 * intuitive "time to centre from full lock" constant so it still reads like a
 * duration when tuning.
 *
 * Standstill source: NOT the live vehicle-speed signal. In an emergency the AS
 * SDC opens and the tractive system is shut down, so the ECU's motor-rpm relay
 * (0x506) is unreliable for the whole window this code runs — it can stick at
 * the last moving value (no ECU staleness guard, IFS08-CE-ECU
 * vehicle_service.cpp) or read a false zero. Instead the caller derives
 * `standstill` from the IMU zero-velocity detector (zupt.h, run in imu_task on
 * the always-powered uDV board — no tractive-system dependency), with
 * EMERG_STOP_MAX_MS as a fixed time backstop so the wheels never actuate
 * indefinitely if the car never reads "quiet" (persistent vibration / stale IMU).
 *
 * PURE: no HAL / RTOS / CAN / libm dependency. The caller carries `target_deg`
 * as the ramp state, passes in `standstill`, and applies the result on the CAN
 * bus (Can::sendSteeringStart / sendSteeringAngle / sendSteeringStop). See
 * app_task.cpp's EMERGENCY case.
 */
#ifndef EMERGENCY_CENTERING_HPP
#define EMERGENCY_CENTERING_HPP

/* Intuitive tuning knob: how long to bring the wheels from FULL LOCK back to
 * centre. Keep it AT OR BELOW the EBS time-to-standstill, or the standstill
 * freeze (below) truncates the centring before it finishes. Bench/track-tune. */
#ifndef EMERG_CENTER_FROM_FULLLOCK_S
#define EMERG_CENTER_FROM_FULLLOCK_S   1.5f
#endif

/* Full-lock span used to turn the time above into a slew rate. Mirrors
 * app_task's STEER_FULL_LOCK_DEG; overridable so a test can pin it. */
#ifndef EMERG_CENTER_FULLLOCK_DEG
#define EMERG_CENTER_FULLLOCK_DEG      65.0f
#endif

/* Derived slew rate (deg/s) — ~43 deg/s at the defaults above. This is the one
 * value the ramp actually uses; the two constants above only make it readable. */
#ifndef EMERG_CENTER_RATE_DEG_S
#define EMERG_CENTER_RATE_DEG_S  (EMERG_CENTER_FULLLOCK_DEG / EMERG_CENTER_FROM_FULLLOCK_S)
#endif

/* Deadband: |setpoint| at or below this counts as centred -> stop. The wheel
 * comes to rest within this of centre; the board's stepper detent holds it. */
#ifndef EMERG_CENTER_TOL_DEG
#define EMERG_CENTER_TOL_DEG           1.0f
#endif

/* Fixed time backstop (ms): the wheels never actuate longer than this in an
 * emergency, even if the IMU never reports standstill (persistent vibration
 * keeps it "not quiet", or the IMU reading goes stale). A pure time cap — not a
 * measured constant. Set above the worst-case EBS stop time. */
#ifndef EMERG_STOP_MAX_MS
#define EMERG_STOP_MAX_MS              4000u
#endif

/** Inputs for one tick of the ramp. */
struct EmergCenterIn {
    float target_deg;    /**< current ramp setpoint (caller-carried state)  */
    bool  standstill;    /**< vehicle at standstill (shaft not turning)     */
    float dt_s;          /**< time since the previous step, seconds         */
};

/** What the caller should do on the CAN bus this tick. */
struct EmergCenterOut {
    bool  arm;            /**< keep the steering motor energised (0x520=1)   */
    bool  send_angle;     /**< emit angle_cmd_deg (0x521)                    */
    float angle_cmd_deg;  /**< ramped absolute setpoint, deg                 */
    bool  stop;           /**< clean de-energise (0x520=0): board holds pos  */
    float new_target_deg; /**< updated ramp state to carry to the next tick  */
};

/**
 * @brief One tick of the emergency-centering ramp.
 *
 * Freezes (clean stop, wheel left where it is) at standstill OR once centred;
 * otherwise steps the setpoint toward 0 by at most EMERG_CENTER_RATE_DEG_S*dt,
 * commanding it while keeping the motor armed. Clamped so it never crosses 0.
 */
inline EmergCenterOut emergency_center_step(const EmergCenterIn& in)
{
    EmergCenterOut o = { false, false, 0.0f, false, in.target_deg };

    const bool centered = (in.target_deg > -EMERG_CENTER_TOL_DEG) &&
                          (in.target_deg <  EMERG_CENTER_TOL_DEG);

    /* "at standstill don't move it more, even if not centred" + job-done once
     * centred: freeze the ramp and de-energise cleanly. This is the board's
     * ordinary clean-stop path (re-armable), NOT its permanent-halt cut-off. */
    if (in.standstill || centered) {
        o.stop = true;
        return o;
    }

    /* Rolling & off-centre: step toward 0, clamped so we never overshoot sign. */
    const float step = EMERG_CENTER_RATE_DEG_S * in.dt_s;
    float t = in.target_deg;
    if (t > 0.0f) { t -= step; if (t < 0.0f) t = 0.0f; }
    else          { t += step; if (t > 0.0f) t = 0.0f; }

    o.new_target_deg = t;
    o.arm            = true;
    o.send_angle     = true;
    o.angle_cmd_deg  = t;
    return o;
}

#endif /* EMERGENCY_CENTERING_HPP */
