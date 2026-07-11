/**
 * @file    imu_align.c
 * @brief   Fixed IMU->car yaw mounting rotation. See imu_align.h for the
 *          convention and how to set IMU_YAW_OFFSET_DEG at build time.
 */
#include "imu_align.h"

#include <math.h>

#define IMU_ALIGN_DEG2RAD 0.01745329251994329577f

void imu_align_init(imu_align_t *a, float yaw_offset_deg)
{
  if (!a) return;
  const float r = yaw_offset_deg * IMU_ALIGN_DEG2RAD;
  a->cos_yaw = cosf(r);
  a->sin_yaw = sinf(r);
}

void imu_align_apply(const imu_align_t *a, bmi088_scaled_t *m)
{
  if (!a || !m) return;

  const float c = a->cos_yaw;
  const float s = a->sin_yaw;

  /* Rotate the horizontal accel + gyro components by +offset about +Z (up),
   * mapping IMU-frame components to car-frame components:
   *   car_x = c*imu_x - s*imu_y
   *   car_y = s*imu_x + c*imu_y
   * az and gz (vertical / yaw-rate) are invariant under a pure yaw offset. */
  const float ax = m->ax_g;
  const float ay = m->ay_g;
  m->ax_g = c * ax - s * ay;
  m->ay_g = s * ax + c * ay;

  const float gx = m->gx_dps;
  const float gy = m->gy_dps;
  m->gx_dps = c * gx - s * gy;
  m->gy_dps = s * gx + c * gy;
}
