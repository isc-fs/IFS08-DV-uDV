/**
 * @file    imu_align.h
 * @brief   IMU-to-car mounting alignment: a fixed yaw offset applied ONCE at
 *          the single point every consumer reads the sample from.
 *
 * The BMI088 is bolted to the uDV board, which is not necessarily square with
 * the car's chassis. A pure YAW mount error (the board rotated about its
 * vertical axis relative to the car) rotates the horizontal accel/gyro axes
 * away from the car frame, so /imu, the 0x512 ECU broadcast and the derived
 * roll/pitch would all be reported in the SENSOR frame, not the car frame.
 *
 * imu_service_step() applies this rotation to the scaled sample the instant it
 * leaves the driver — BEFORE the attitude filter and BEFORE it is copied into
 * the queue — so every downstream user (roll/pitch, CAN 0x512, ROS /imu) sees
 * one consistent CAR-frame sample. Fix it once, it is right everywhere.
 *
 * === Convention ===
 * IMU_YAW_OFFSET_DEG is the yaw angle FROM the car's forward (+X) axis TO the
 * IMU's forward (+X) axis, measured CCW about the shared vertical (+Z, up)
 * axis (right-hand rule). Equivalently: how many degrees counter-clockwise the
 * board is rotated relative to the car when viewed from above.
 *   - board turned 90 deg CCW (its +X points to the car's LEFT)  -> set  +90
 *   - board turned 30 deg CW  (its +X points 30 deg to the RIGHT) -> set  -30
 * A pure yaw offset leaves the vertical axis untouched (az and gz/yaw-rate are
 * invariant), so roll/pitch — derived from the rotated accel — come out about
 * the car's axes for free.
 *
 * Default 0.0 = board aligned with the car (flight-clean; folds to a no-op).
 * Set it WITHOUT editing this file, the same way as a bench toggle — pass it
 * as a build define so flight source and the override stay separate:
 *     make CONFIG="-DIMU_YAW_OFFSET_DEG=12.5"      (Make, production)
 *     cmake ... -DIMU_YAW_OFFSET_DEG=12.5          (CMake / IDE)
 * Because the macro is #ifndef-guarded, a -D wins and this 0.0 default is
 * skipped. Measure the angle once at commissioning (board vs chassis, or by
 * comparing heading while driving a straight line); a non-zero value announces
 * itself on /debug at boot so the flashed offset can be verified in the pit.
 */
#ifndef IMU_ALIGN_H
#define IMU_ALIGN_H

#include "bmi088.h"   /* bmi088_scaled_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Yaw mounting offset in DEGREES (see convention above). #ifndef-guarded so a
 * -D override at build time wins and this flight default of 0 is skipped. */
#ifndef IMU_YAW_OFFSET_DEG
#define IMU_YAW_OFFSET_DEG 0.0f
#endif

typedef struct {
  float cos_yaw;   /* precomputed cos(offset) — set by imu_align_init */
  float sin_yaw;   /* precomputed sin(offset) — set by imu_align_init */
} imu_align_t;

/* Precompute the rotation from a yaw offset in DEGREES. Call once at init;
 * the trig is paid here, never in the 400 Hz sample path. NULL-safe. */
void imu_align_init(imu_align_t *a, float yaw_offset_deg);

/* Rotate a scaled sample in place from the IMU frame into the car frame: the
 * horizontal accel (ax,ay) and gyro (gx,gy) pairs are rotated; the vertical
 * axis (az, gz) is left untouched. A zero offset is an exact no-op. NULL-safe. */
void imu_align_apply(const imu_align_t *a, bmi088_scaled_t *m);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ALIGN_H */
