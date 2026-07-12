/**
 * @file    zupt.h
 * @brief   Zero-velocity (standstill) detector from IMU quietness — pure,
 *          host-testable, libm-free.
 *
 * A stationary car is "quiet": the gyro reads ~0 (no rotation) and the
 * accelerometer magnitude sits at ~1 g (no net linear acceleration). A rolling
 * or braking car is not — road vibration and load transfer stir the gyro, and
 * braking adds a horizontal component so |accel| = sqrt(1 + a_brake^2) > 1 g.
 * When both signals stay quiet for a debounce window, we declare standstill.
 *
 * This is used to decide when to STOP actively centring the steering in an AS
 * emergency (see emergency_centering.hpp / app_task.cpp). It replaces the
 * tractive-system rpm signal, which is unreliable once the SDC opens, and needs
 * no drivetrain geometry or velocity integration — only two thresholds that are
 * read straight off the IMU noise floor with the car stationary in the pit (not
 * a driving experiment).
 *
 * SCOPE: it distinguishes hard-braking / rolling from stopped — the emergency
 * premise (the EBS is actively decelerating the car). It is NOT a general
 * constant-velocity-coast detector; a car gliding at steady speed on glass-
 * smooth tarmac can also look quiet. That is a compound failure (EBS not
 * decelerating) and out of scope here; the caller keeps a fixed time backstop.
 *
 * PURE: the caller computes the scalar |accel| (g) and |gyro| (dps, bias-
 * compensated) each IMU sample and feeds them in; this header only thresholds
 * and debounces. Run it at the IMU sample rate so it sees the vibration.
 */
#ifndef ZUPT_H
#define ZUPT_H

#include <stdint.h>
#include <stdbool.h>

/* |gyro| (dps) below this counts as "not rotating". Set from the stationary
 * bias-compensated gyro noise floor + margin. COMMISSION: read in the pit. */
#ifndef ZUPT_GYRO_STILL_DPS
#define ZUPT_GYRO_STILL_DPS   3.0f
#endif

/* ||accel| - 1 g| below this counts as "not accelerating/decelerating". Set
 * above the stationary accel-magnitude noise, below the smallest braking decel
 * we must reject. COMMISSION: read in the pit. */
#ifndef ZUPT_ACCEL_BAND_G
#define ZUPT_ACCEL_BAND_G     0.08f
#endif

/* How long both signals must stay quiet before standstill is declared (ms).
 * Rejects brief quiet transients mid-motion. */
#ifndef ZUPT_DEBOUNCE_MS
#define ZUPT_DEBOUNCE_MS      200u
#endif

typedef struct {
    uint32_t quiet_since_ms;   /* tick when the current quiet run began */
    bool     in_quiet;         /* currently inside a quiet run */
    bool     standstill;       /* latest debounced verdict */
} zupt_t;

static inline void zupt_reset(zupt_t *z)
{
    z->quiet_since_ms = 0u;
    z->in_quiet       = false;
    z->standstill     = false;
}

/**
 * @brief Feed one IMU sample; returns the current (debounced) standstill verdict.
 * @param now_ms        monotonic tick (ms)
 * @param accel_mag_g   |accelerometer| in g (includes gravity → ~1 at rest)
 * @param gyro_mag_dps  |gyro| in dps, BIAS-COMPENSATED
 */
static inline bool zupt_update(zupt_t *z, uint32_t now_ms,
                               float accel_mag_g, float gyro_mag_dps)
{
    float accel_dev = accel_mag_g - 1.0f;
    if (accel_dev < 0.0f) accel_dev = -accel_dev;      /* |accel| deviation from 1 g */

    const bool quiet = (gyro_mag_dps < ZUPT_GYRO_STILL_DPS) &&
                       (accel_dev    < ZUPT_ACCEL_BAND_G);

    if (quiet) {
        if (!z->in_quiet) {                 /* start of a new quiet run */
            z->in_quiet       = true;
            z->quiet_since_ms = now_ms;
        }
        z->standstill =
            ((uint32_t)(now_ms - z->quiet_since_ms) >= ZUPT_DEBOUNCE_MS);
    } else {
        z->in_quiet   = false;
        z->standstill = false;
    }
    return z->standstill;
}

#endif /* ZUPT_H */
