/**
 * @file test_emergency_centering.cpp
 * @brief Host unit tests for the pure emergency steering-centering ramp
 *        (Core/Inc/emergency_centering.hpp).
 *
 * Safety-relevant: on an AS EMERGENCY the wheels must be brought back toward
 * centre GENTLY (bounded slew rate) while the car is still rolling, and must be
 * LEFT ALONE once it is at standstill. This suite pins the tuning constants and
 * asserts that behaviour independently of app_task's CAN plumbing.
 *
 * Build + run:  make test_emergency_centering
 */

/* Pin the tuning knobs so the arithmetic is deterministic (the header guards
 * each with #ifndef). 40 deg/s @ 20 ms tick => 0.8 deg/step; tol 1.0 deg. */
#define EMERG_CENTER_RATE_DEG_S   40.0f
#define EMERG_CENTER_TOL_DEG      1.0f

#include "test.h"
#include "emergency_centering.hpp"

static const float DT = 0.02f;         /* 20 ms tick (matches TORQUE_TX_PERIOD_MS) */
static const float STEP = EMERG_CENTER_RATE_DEG_S * DT;   /* 0.8 deg */

/* While rolling and off-centre: step toward 0 by exactly rate*dt, motor armed,
 * angle emitted, no stop. Checked for both signs. */
static void test_ramps_toward_zero(void)
{
    EmergCenterIn in = { 60.0f, /*standstill*/ false, DT };
    EmergCenterOut o = emergency_center_step(in);
    ASSERT_TRUE(o.arm);
    ASSERT_TRUE(o.send_angle);
    ASSERT_FALSE(o.stop);
    ASSERT_NEAR_FLOAT(o.new_target_deg, 60.0f - STEP, 1e-4f);
    ASSERT_NEAR_FLOAT(o.angle_cmd_deg,  60.0f - STEP, 1e-4f);

    in.target_deg = -60.0f;
    o = emergency_center_step(in);
    ASSERT_TRUE(o.arm);
    ASSERT_FALSE(o.stop);
    ASSERT_NEAR_FLOAT(o.new_target_deg, -60.0f + STEP, 1e-4f);
}

/* At standstill: freeze exactly where we are (even if far off-centre) and emit
 * a clean stop — never a movement command. This is the "don't move it more"
 * requirement. */
static void test_standstill_freezes(void)
{
    EmergCenterIn in = { 25.0f, /*standstill*/ true, DT };
    EmergCenterOut o = emergency_center_step(in);
    ASSERT_TRUE(o.stop);
    ASSERT_FALSE(o.arm);
    ASSERT_FALSE(o.send_angle);
    ASSERT_NEAR_FLOAT(o.new_target_deg, 25.0f, 1e-6f);   /* unchanged */
}

/* Once within the deadband: job done -> stop (de-energise, board holds). */
static void test_centered_stops(void)
{
    EmergCenterIn in = { 0.5f, /*standstill*/ false, DT };
    EmergCenterOut o = emergency_center_step(in);
    ASSERT_TRUE(o.stop);
    ASSERT_FALSE(o.send_angle);
}

/* The step is clamped so it never crosses 0 (no sign flip / no oscillation),
 * even when rate*dt overshoots the remaining angle. */
static void test_no_overshoot(void)
{
    EmergCenterIn in = { 2.0f, /*standstill*/ false, /*dt*/ 0.1f };  /* step 4.0 > 2.0 */
    EmergCenterOut o = emergency_center_step(in);
    ASSERT_NEAR_FLOAT(o.new_target_deg, 0.0f, 1e-6f);

    in.target_deg = -2.0f;
    o = emergency_center_step(in);
    ASSERT_NEAR_FLOAT(o.new_target_deg, 0.0f, 1e-6f);
}

/* Full run: from full lock the ramp reaches centre in ~"time from full lock",
 * monotonically, never changing sign, and then reports a stop. */
static void test_full_ramp_duration(void)
{
    float target = EMERG_CENTER_FULLLOCK_DEG;   /* 65 deg default */
    float prev = target;
    float t_s = 0.0f;
    int guard = 0;
    bool stopped = false;

    for (; guard < 100000; ++guard)
    {
        EmergCenterIn in = { target, /*standstill*/ false, DT };
        EmergCenterOut o = emergency_center_step(in);
        if (o.stop) { stopped = true; break; }
        /* monotonic toward 0, never negative for a positive start */
        ASSERT_TRUE(o.new_target_deg <= prev);
        ASSERT_TRUE(o.new_target_deg >= 0.0f);
        prev = o.new_target_deg;
        target = o.new_target_deg;
        t_s += DT;
    }
    ASSERT_TRUE(stopped);
    /* Expected: 65 deg / 40 deg/s = 1.625 s, within a couple of ticks. */
    ASSERT_NEAR_FLOAT(t_s, EMERG_CENTER_FULLLOCK_DEG / EMERG_CENTER_RATE_DEG_S, 2.0f * DT);
}

int main(void)
{
    TEST_BEGIN();
    TEST_CASE(test_ramps_toward_zero);
    TEST_CASE(test_standstill_freezes);
    TEST_CASE(test_centered_stops);
    TEST_CASE(test_no_overshoot);
    TEST_CASE(test_full_ramp_duration);
    TEST_END();
}
