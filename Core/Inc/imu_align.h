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
 * >>> THE OFFSET IS SET IN EXACTLY ONE PLACE: the IMU_YAW_OFFSET_DEG #define
 *     below. Change that one line if the board is remounted, rebuild, reflash.
 *     Nothing else in the tree carries the value. <<<
 * It is currently 101.0 deg (the board's present mounting). 0.0 would mean a
 * perfectly square mount. The active value announces itself on /debug at boot
 * so it can be verified in the pit. To re-measure after a remount: board vs
 * chassis with a protractor, or compare heading while driving a straight line.
 *
 * SIGN CHECK: 101 assumes the convention above (CCW car->IMU). If a straight-
 * line test drive shows the corrected heading/accel is mirrored, the board is
 * rotated the other way — flip the sign to -101 (one character).
 */
#ifndef IMU_ALIGN_H
#define IMU_ALIGN_H

#include "bmi088.h"   /* bmi088_scaled_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ======================= THE ONE PLACE TO SET IT =======================
 * IMU->car yaw mounting offset in DEGREES (see convention + sign check above).
 * This single line is the source of truth for the whole firmware; edit it when
 * the board is remounted. Currently 101 deg = the board's present mounting.
 * (#ifndef-guarded only so the host unit test can exercise other angles.) */
#ifndef IMU_YAW_OFFSET_DEG
#define IMU_YAW_OFFSET_DEG 101.0f
#endif
/* ======================================================================= */

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
