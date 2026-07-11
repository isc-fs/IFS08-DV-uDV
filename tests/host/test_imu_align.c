/*
 * Tests for imu_align.c — the fixed IMU->car yaw mounting rotation applied to
 * every scaled sample in imu_service_step (before the attitude filter and the
 * CAN 0x512 / ROS /imu fan-out). Pure math, no HAL.
 *
 * Convention (imu_align.h): IMU_YAW_OFFSET_DEG is the CCW angle about +Z (up)
 * from the car's forward (+X) to the IMU's forward (+X). The applied rotation
 * maps IMU-frame components to car-frame components:
 *   car_x = cos*imu_x - sin*imu_y
 *   car_y = sin*imu_x + cos*imu_y
 * az and gz (vertical / yaw-rate) are invariant.
 */
#include "test.h"
#include "imu_align.h"

static bmi088_scaled_t make_sample(void)
{
  /* Distinct, asymmetric values so axis swaps/sign errors can't hide. */
  bmi088_scaled_t m;
  m.ax_g = 0.20f;  m.ay_g = -0.50f;  m.az_g = 0.98f;
  m.gx_dps = 12.0f; m.gy_dps = -3.0f; m.gz_dps = 7.5f;
  return m;
}

static void test_zero_offset_is_identity(void)
{
  imu_align_t a;
  imu_align_init(&a, 0.0f);

  bmi088_scaled_t m = make_sample();
  imu_align_apply(&a, &m);

  ASSERT_NEAR_FLOAT(m.ax_g,  0.20f, 1e-6f);
  ASSERT_NEAR_FLOAT(m.ay_g, -0.50f, 1e-6f);
  ASSERT_NEAR_FLOAT(m.az_g,  0.98f, 1e-6f);
  ASSERT_NEAR_FLOAT(m.gx_dps, 12.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(m.gy_dps, -3.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(m.gz_dps,  7.5f, 1e-6f);
}

static void test_90ccw_maps_axes(void)
{
  /* +90 deg: cos=0, sin=1 -> car_x = -imu_y, car_y = +imu_x. A forward car
   * accel that the (left-pointing) IMU reads as (0,-1) must come back as
   * (+1,0) in the car frame. */
  imu_align_t a;
  imu_align_init(&a, 90.0f);

  bmi088_scaled_t m = {0};
  m.ax_g = 0.0f; m.ay_g = -1.0f;   /* IMU sees forward accel on -Y */
  imu_align_apply(&a, &m);
  ASSERT_NEAR_FLOAT(m.ax_g, 1.0f, 1e-5f);   /* recovered car forward */
  ASSERT_NEAR_FLOAT(m.ay_g, 0.0f, 1e-5f);

  /* General vector: car_x = -imu_y, car_y = imu_x. */
  bmi088_scaled_t n = make_sample();
  const float ax = n.ax_g, ay = n.ay_g, gx = n.gx_dps, gy = n.gy_dps;
  imu_align_apply(&a, &n);
  ASSERT_NEAR_FLOAT(n.ax_g, -ay, 1e-5f);
  ASSERT_NEAR_FLOAT(n.ay_g,  ax, 1e-5f);
  ASSERT_NEAR_FLOAT(n.gx_dps, -gy, 1e-5f);
  ASSERT_NEAR_FLOAT(n.gy_dps,  gx, 1e-5f);
}

static void test_neg90_maps_axes(void)
{
  /* -90 deg: cos=0, sin=-1 -> car_x = +imu_y, car_y = -imu_x. */
  imu_align_t a;
  imu_align_init(&a, -90.0f);

  bmi088_scaled_t m = make_sample();
  const float ax = m.ax_g, ay = m.ay_g, gx = m.gx_dps, gy = m.gy_dps;
  imu_align_apply(&a, &m);
  ASSERT_NEAR_FLOAT(m.ax_g,  ay, 1e-5f);
  ASSERT_NEAR_FLOAT(m.ay_g, -ax, 1e-5f);
  ASSERT_NEAR_FLOAT(m.gx_dps,  gy, 1e-5f);
  ASSERT_NEAR_FLOAT(m.gy_dps, -gx, 1e-5f);
}

static void test_vertical_axes_untouched(void)
{
  /* A pure yaw offset must never disturb az or gz (yaw rate). */
  imu_align_t a;
  imu_align_init(&a, 37.0f);

  bmi088_scaled_t m = make_sample();
  imu_align_apply(&a, &m);
  ASSERT_NEAR_FLOAT(m.az_g,   0.98f, 1e-6f);
  ASSERT_NEAR_FLOAT(m.gz_dps, 7.5f,  1e-6f);
}

static void test_horizontal_norm_preserved(void)
{
  /* Rotation is length-preserving: |(ax,ay)| and |(gx,gy)| are invariant. */
  imu_align_t a;
  imu_align_init(&a, 51.3f);

  bmi088_scaled_t m = make_sample();
  const float a_norm = sqrtf(m.ax_g * m.ax_g + m.ay_g * m.ay_g);
  const float g_norm = sqrtf(m.gx_dps * m.gx_dps + m.gy_dps * m.gy_dps);
  imu_align_apply(&a, &m);
  ASSERT_NEAR_FLOAT(sqrtf(m.ax_g * m.ax_g + m.ay_g * m.ay_g), a_norm, 1e-4f);
  ASSERT_NEAR_FLOAT(sqrtf(m.gx_dps * m.gx_dps + m.gy_dps * m.gy_dps), g_norm, 1e-4f);
}

static void test_360_is_identity(void)
{
  imu_align_t a;
  imu_align_init(&a, 360.0f);

  bmi088_scaled_t m = make_sample();
  imu_align_apply(&a, &m);
  ASSERT_NEAR_FLOAT(m.ax_g,  0.20f, 1e-4f);
  ASSERT_NEAR_FLOAT(m.ay_g, -0.50f, 1e-4f);
  ASSERT_NEAR_FLOAT(m.gx_dps, 12.0f, 1e-4f);
  ASSERT_NEAR_FLOAT(m.gy_dps, -3.0f, 1e-4f);
}

static void test_null_safe(void)
{
  imu_align_t a;
  imu_align_init(&a, 10.0f);
  bmi088_scaled_t m = make_sample();

  /* Should not crash. */
  imu_align_init(NULL, 10.0f);
  imu_align_apply(NULL, &m);
  imu_align_apply(&a, NULL);
}

int main(void)
{
  TEST_BEGIN();
  TEST_CASE(test_zero_offset_is_identity);
  TEST_CASE(test_90ccw_maps_axes);
  TEST_CASE(test_neg90_maps_axes);
  TEST_CASE(test_vertical_axes_untouched);
  TEST_CASE(test_horizontal_norm_preserved);
  TEST_CASE(test_360_is_identity);
  TEST_CASE(test_null_safe);
  TEST_END();
}
