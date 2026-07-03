/*
 * Tests for attitude.c
 *
 * Scope: the deterministic, hardware-independent parts of the complementary
 * filter — initialization, gyro bias setter, and the inline radian→degree
 * accessors. attitude_update_cordic depends on the STM32 CORDIC peripheral
 * for atan2 and is not exercised here (would require an emulator).
 */
#include "test.h"
#include "attitude.h"

#define RAD_PER_DEG (3.14159265358979323846f / 180.0f)

static void test_init_zeroes_state(void)
{
  attitude_t a;
  /* Fill with garbage so we can confirm init clears it */
  a.roll = 99.0f;
  a.pitch = -77.0f;
  a.gx_off_dps = 1.0f;
  a.gy_off_dps = 2.0f;
  a.gz_off_dps = 3.0f;
  a.last_ms = 12345;

  attitude_init(&a, 0.98f);

  ASSERT_NEAR_FLOAT(a.roll, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.pitch, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.alpha, 0.98f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.gx_off_dps, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.gy_off_dps, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.gz_off_dps, 0.0f, 1e-6f);
  ASSERT_EQ_INT(a.last_ms, 0);
}

static void test_init_alpha_preserved(void)
{
  attitude_t a;
  attitude_init(&a, 0.5f);
  ASSERT_NEAR_FLOAT(a.alpha, 0.5f, 1e-6f);

  attitude_init(&a, 0.0f);
  ASSERT_NEAR_FLOAT(a.alpha, 0.0f, 1e-6f);

  attitude_init(&a, 1.0f);
  ASSERT_NEAR_FLOAT(a.alpha, 1.0f, 1e-6f);
}

static void test_init_null_safe(void)
{
  /* Should not crash */
  attitude_init(NULL, 0.98f);
}

static void test_set_gyro_bias(void)
{
  attitude_t a;
  attitude_init(&a, 0.98f);
  attitude_set_gyro_bias_dps(&a, 1.5f, -2.5f, 0.75f);
  ASSERT_NEAR_FLOAT(a.gx_off_dps,  1.5f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.gy_off_dps, -2.5f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.gz_off_dps,  0.75f, 1e-6f);

  /* Other fields untouched */
  ASSERT_NEAR_FLOAT(a.roll, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.pitch, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(a.alpha, 0.98f, 1e-6f);
}

static void test_set_gyro_bias_null_safe(void)
{
  attitude_set_gyro_bias_dps(NULL, 1.0f, 2.0f, 3.0f);
}

static void test_roll_deg_conversion(void)
{
  attitude_t a;
  attitude_init(&a, 0.98f);

  a.roll = 0.0f;
  ASSERT_NEAR_FLOAT(attitude_roll_deg(&a), 0.0f, 1e-4f);

  a.roll = RAD_PER_DEG * 45.0f;
  ASSERT_NEAR_FLOAT(attitude_roll_deg(&a), 45.0f, 1e-3f);

  a.roll = RAD_PER_DEG * -90.0f;
  ASSERT_NEAR_FLOAT(attitude_roll_deg(&a), -90.0f, 1e-3f);

  a.roll = RAD_PER_DEG * 180.0f;
  ASSERT_NEAR_FLOAT(attitude_roll_deg(&a), 180.0f, 1e-3f);
}

static void test_pitch_deg_conversion(void)
{
  attitude_t a;
  attitude_init(&a, 0.98f);

  a.pitch = 0.0f;
  ASSERT_NEAR_FLOAT(attitude_pitch_deg(&a), 0.0f, 1e-4f);

  a.pitch = RAD_PER_DEG * 30.0f;
  ASSERT_NEAR_FLOAT(attitude_pitch_deg(&a), 30.0f, 1e-3f);

  a.pitch = RAD_PER_DEG * -15.5f;
  ASSERT_NEAR_FLOAT(attitude_pitch_deg(&a), -15.5f, 1e-3f);
}

int main(void)
{
  TEST_BEGIN();
  TEST_CASE(test_init_zeroes_state);
  TEST_CASE(test_init_alpha_preserved);
  TEST_CASE(test_init_null_safe);
  TEST_CASE(test_set_gyro_bias);
  TEST_CASE(test_set_gyro_bias_null_safe);
  TEST_CASE(test_roll_deg_conversion);
  TEST_CASE(test_pitch_deg_conversion);
  TEST_END();
}
