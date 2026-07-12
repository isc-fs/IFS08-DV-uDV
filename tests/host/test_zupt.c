/**
 * @file test_zupt.c
 * @brief Host unit tests for the pure zero-velocity (standstill) detector
 *        (Core/Inc/zupt.h): quietness thresholds + debounce.
 *
 * Build + run:  make test_zupt
 */

/* Pin thresholds so the arithmetic is deterministic (header guards each). */
#define ZUPT_GYRO_STILL_DPS   3.0f
#define ZUPT_ACCEL_BAND_G     0.08f
#define ZUPT_DEBOUNCE_MS      200u

#include "test.h"
#include "zupt.h"

/* A "stationary" sample: 1 g, negligible rotation. */
static const float STILL_ACCEL = 1.00f;
static const float STILL_GYRO  = 0.5f;

/* Fresh detector reports not-standstill until the debounce elapses, then latches. */
static void test_debounce(void)
{
    zupt_t z; zupt_reset(&z);
    ASSERT_FALSE(z.standstill);

    ASSERT_FALSE(zupt_update(&z, 1000u,        STILL_ACCEL, STILL_GYRO));  /* t0 */
    ASSERT_FALSE(zupt_update(&z, 1000u + 199u, STILL_ACCEL, STILL_GYRO));  /* just under */
    ASSERT_TRUE (zupt_update(&z, 1000u + 200u, STILL_ACCEL, STILL_GYRO));  /* debounce met */
    ASSERT_TRUE (zupt_update(&z, 1000u + 500u, STILL_ACCEL, STILL_GYRO));  /* stays latched */
}

/* Braking (accel magnitude well above 1 g) is never "quiet", so it never
 * declares standstill however long it lasts. */
static void test_braking_not_still(void)
{
    zupt_t z; zupt_reset(&z);
    for (uint32_t t = 0; t <= 1000u; t += 20u) {
        /* ~1 g decel -> |accel| = sqrt(1+1) ~ 1.41 g, far outside the band. */
        ASSERT_FALSE(zupt_update(&z, 1000u + t, 1.41f, STILL_GYRO));
    }
}

/* High gyro (rotation / vibration) is not quiet even at 1 g. */
static void test_gyro_rejects(void)
{
    zupt_t z; zupt_reset(&z);
    ASSERT_FALSE(zupt_update(&z, 1000u,        STILL_ACCEL, 5.0f));
    ASSERT_FALSE(zupt_update(&z, 1000u + 500u, STILL_ACCEL, 5.0f));  /* never latches */
}

/* Accel band edge: 0.05 g deviation is quiet, 0.10 g is not. */
static void test_accel_band(void)
{
    zupt_t z; zupt_reset(&z);
    ASSERT_FALSE(zupt_update(&z, 100u,        1.05f, STILL_GYRO));  /* quiet, pre-debounce */
    ASSERT_TRUE (zupt_update(&z, 100u + 200u, 1.05f, STILL_GYRO));  /* quiet held -> latched */

    zupt_reset(&z);
    ASSERT_FALSE(zupt_update(&z, 100u,        1.10f, STILL_GYRO));  /* 0.10 > 0.08 band */
    ASSERT_FALSE(zupt_update(&z, 100u + 500u, 1.10f, STILL_GYRO));  /* never latches */
}

/* A motion blip mid-quiet resets the debounce: standstill must be re-earned. */
static void test_blip_resets(void)
{
    zupt_t z; zupt_reset(&z);
    (void)zupt_update(&z, 2000u,        STILL_ACCEL, STILL_GYRO);   /* quiet begins */
    ASSERT_TRUE (zupt_update(&z, 2000u + 200u, STILL_ACCEL, STILL_GYRO));  /* latched */
    ASSERT_FALSE(zupt_update(&z, 2000u + 210u, 1.41f,       STILL_GYRO));  /* blip -> reset */
    ASSERT_FALSE(zupt_update(&z, 2000u + 220u, STILL_ACCEL, STILL_GYRO));  /* re-arming */
    ASSERT_TRUE (zupt_update(&z, 2000u + 420u, STILL_ACCEL, STILL_GYRO));  /* re-earned */
}

int main(void)
{
    TEST_BEGIN();
    TEST_CASE(test_debounce);
    TEST_CASE(test_braking_not_still);
    TEST_CASE(test_gyro_rejects);
    TEST_CASE(test_accel_band);
    TEST_CASE(test_blip_resets);
    TEST_END();
}
