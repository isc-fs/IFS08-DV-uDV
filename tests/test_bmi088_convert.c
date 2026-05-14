/*
 * Tests for bmi088_convert_scaled — the pure scaling math that turns
 * raw int16 BMI088 register values into engineering units (g, dps).
 *
 * Scale factors per bmi088_bind / bmi088_init_minimal:
 *   acc_lsb_per_g   = 5460.0  (±6g range)
 *   gyr_lsb_per_dps =   16.4  (±2000 dps range)
 */
#include "test.h"
#include "bmi088.h"

static bmi088_t dev;

static void setup_dev(void)
{
  dev.bus = NULL; /* not used by convert */
  dev.acc_addr_7b = BMI088_ACC_ADDR_7B;
  dev.gyr_addr_7b = BMI088_GYR_ADDR_7B;
  dev.acc_lsb_per_g   = 5460.0f;
  dev.gyr_lsb_per_dps =   16.4f;
}

static void test_convert_zero(void)
{
  setup_dev();
  bmi088_raw_t raw = {0, 0, 0, 0, 0, 0};
  bmi088_scaled_t s;
  bmi088_convert_scaled(&dev, &raw, &s);
  ASSERT_NEAR_FLOAT(s.ax_g, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.ay_g, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.az_g, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.gx_dps, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.gy_dps, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.gz_dps, 0.0f, 1e-6f);
}

static void test_convert_accel_1g(void)
{
  setup_dev();
  /* 5460 LSB on the Z axis (gravity) should read +1.0 g */
  bmi088_raw_t raw = {0, 0, 5460, 0, 0, 0};
  bmi088_scaled_t s;
  bmi088_convert_scaled(&dev, &raw, &s);
  ASSERT_NEAR_FLOAT(s.ax_g, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.ay_g, 0.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.az_g, 1.0f, 1e-4f);
}

static void test_convert_accel_negative(void)
{
  setup_dev();
  /* -5460 on X should be -1.0 g; verifies signed handling */
  bmi088_raw_t raw = {-5460, 2730, 0, 0, 0, 0};
  bmi088_scaled_t s;
  bmi088_convert_scaled(&dev, &raw, &s);
  ASSERT_NEAR_FLOAT(s.ax_g, -1.0f, 1e-4f);
  ASSERT_NEAR_FLOAT(s.ay_g,  0.5f, 1e-4f);  /* 2730 / 5460 */
}

static void test_convert_gyro_full_scale(void)
{
  setup_dev();
  /* At ±2000 dps, full-scale raw is 32767. 32767 / 16.4 ≈ 1998.0 dps */
  bmi088_raw_t raw = {0, 0, 0, 32767, -32768, 1640};
  bmi088_scaled_t s;
  bmi088_convert_scaled(&dev, &raw, &s);
  ASSERT_NEAR_FLOAT(s.gx_dps,  1998.0f, 1.0f);
  ASSERT_NEAR_FLOAT(s.gy_dps, -1998.0f, 1.0f);
  ASSERT_NEAR_FLOAT(s.gz_dps,   100.0f, 0.1f);  /* 1640 / 16.4 = 100.0 */
}

static void test_convert_independent_axes(void)
{
  setup_dev();
  /* Each axis should map independently — set distinct values and verify */
  bmi088_raw_t raw = {100, 200, 300, 400, 500, 600};
  bmi088_scaled_t s;
  bmi088_convert_scaled(&dev, &raw, &s);
  ASSERT_NEAR_FLOAT(s.ax_g, 100.0f / 5460.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.ay_g, 200.0f / 5460.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.az_g, 300.0f / 5460.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(s.gx_dps, 400.0f / 16.4f, 1e-4f);
  ASSERT_NEAR_FLOAT(s.gy_dps, 500.0f / 16.4f, 1e-4f);
  ASSERT_NEAR_FLOAT(s.gz_dps, 600.0f / 16.4f, 1e-4f);
}

static void test_convert_null_safety(void)
{
  setup_dev();
  bmi088_raw_t raw = {1, 2, 3, 4, 5, 6};
  bmi088_scaled_t s = {99.0f, 99.0f, 99.0f, 99.0f, 99.0f, 99.0f};
  /* Should be no-ops, not crash, and leave outputs unchanged */
  bmi088_convert_scaled(NULL, &raw, &s);
  bmi088_convert_scaled(&dev, NULL, &s);
  bmi088_convert_scaled(&dev, &raw, NULL);
  ASSERT_NEAR_FLOAT(s.ax_g, 99.0f, 1e-6f); /* unchanged after NULL calls */
}

static void test_bind_param_validation(void)
{
  /* bmi088_bind should reject null pointers and null bus->hi2c */
  bmi088_t d;
  I2C_Utils_t bus;
  I2C_HandleTypeDef hi2c;
  bus.hi2c = &hi2c;
  ASSERT_EQ_INT(bmi088_bind(NULL, &bus, 0x18, 0x68), BMI088_ERR_PARAM);
  ASSERT_EQ_INT(bmi088_bind(&d, NULL, 0x18, 0x68), BMI088_ERR_PARAM);

  bus.hi2c = NULL;
  ASSERT_EQ_INT(bmi088_bind(&d, &bus, 0x18, 0x68), BMI088_ERR_PARAM);

  bus.hi2c = &hi2c;
  ASSERT_EQ_INT(bmi088_bind(&d, &bus, 0x18, 0x68), BMI088_OK);
  ASSERT_NEAR_FLOAT(d.acc_lsb_per_g, 5460.0f, 1e-6f);
  ASSERT_NEAR_FLOAT(d.gyr_lsb_per_dps, 16.4f, 1e-6f);
  ASSERT_EQ_INT(d.acc_addr_7b, 0x18);
  ASSERT_EQ_INT(d.gyr_addr_7b, 0x68);
}

int main(void)
{
  TEST_BEGIN();
  TEST_CASE(test_convert_zero);
  TEST_CASE(test_convert_accel_1g);
  TEST_CASE(test_convert_accel_negative);
  TEST_CASE(test_convert_gyro_full_scale);
  TEST_CASE(test_convert_independent_axes);
  TEST_CASE(test_convert_null_safety);
  TEST_CASE(test_bind_param_validation);
  TEST_END();
}
