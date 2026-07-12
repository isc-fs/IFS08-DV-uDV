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
 * each with #ifndef). 40 deg/s @ 20 ms tick => 0.8 deg/step; tol 1.0 deg;
 * 2000 rpm/s decel => stop_ms = |rpm0|/2; window capped at 4000 ms. */
#define EMERG_CENTER_RATE_DEG_S   40.0f
#define EMERG_CENTER_TOL_DEG      1.0f
#define EMERG_DECEL_RPM_PER_S     2000.0f
#define EMERG_STOP_MAX_MS         4000u

#include "test.h"
#include "emergency_centering.hpp"

static const float    DT     = 0.02f;   /* 20 ms tick (matches TORQUE_TX_PERIOD_MS) */
static const uint32_t DT_MS  = 20u;
static const float    STEP   = EMERG_CENTER_RATE_DEG_S * DT;   /* 0.8 deg */

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

/* Feedforward time-to-standstill from the rpm captured at the emergency edge:
 * |rpm0| / decel, sign-agnostic, 0 -> 0, clamped to the max window. */
static void test_stop_time_estimate(void)
{
    ASSERT_EQ_INT(emergency_stop_time_ms(0),      0);       /* stopped -> freeze now */
    ASSERT_EQ_INT(emergency_stop_time_ms(2000),   1000);    /* 2000/2000 s */
    ASSERT_EQ_INT(emergency_stop_time_ms(500),    250);
    ASSERT_EQ_INT(emergency_stop_time_ms(-1000),  500);     /* uses |rpm0| */
    ASSERT_EQ_INT(emergency_stop_time_ms(100000), (long)EMERG_STOP_MAX_MS);  /* clamp */
}

/* End-to-end composition as app_task runs it: seed a deadline from rpm0 on the
 * edge, then each tick derive standstill from the local clock (never live rpm)
 * and feed the ramp. Returns the resting setpoint and whether it froze while
 * still off-centre. */
static void run_emergency(int32_t rpm0, float start_deg,
                          float* out_final, bool* out_frozen_offcenter)
{
    uint32_t t        = 1000u;                               /* arbitrary start tick */
    uint32_t deadline = t + emergency_stop_time_ms(rpm0);
    float    target   = start_deg;
    bool     frozen_off = false;

    for (int i = 0; i < 100000; ++i) {
        bool standstill = ((int32_t)(t - deadline) >= 0);    /* wrap-safe, as app_task */
        EmergCenterIn in = { target, standstill, DT };
        EmergCenterOut o = emergency_center_step(in);
        target = o.new_target_deg;
        if (o.stop) {
            frozen_off = (target > EMERG_CENTER_TOL_DEG) ||
                         (target < -EMERG_CENTER_TOL_DEG);
            break;
        }
        t += DT_MS;
    }
    *out_final = target;
    *out_frozen_offcenter = frozen_off;
}

/* High speed: the stop window (2000 ms) outlasts the centring ramp (~1500 ms
 * from 60 deg), so the wheels fully centre and stop via the centred branch. */
static void test_feedforward_high_speed_centers(void)
{
    float final_deg; bool frozen_off;
    run_emergency(/*rpm0*/ 4000, /*start*/ 60.0f, &final_deg, &frozen_off);
    ASSERT_FALSE(frozen_off);
    ASSERT_TRUE(final_deg <= EMERG_CENTER_TOL_DEG && final_deg >= -EMERG_CENTER_TOL_DEG);
}

/* Low speed: the stop window (200 ms) is far shorter than the ramp, so the
 * feedforward freeze truncates centring with the wheel still well off-centre —
 * the "don't move it at standstill even if not centred" requirement. */
static void test_feedforward_low_speed_freezes_offcenter(void)
{
    float final_deg; bool frozen_off;
    run_emergency(/*rpm0*/ 400, /*start*/ 60.0f, &final_deg, &frozen_off);
    ASSERT_TRUE(frozen_off);
    ASSERT_TRUE(final_deg > 40.0f);        /* only ~8 deg of travel in 200 ms */
}

/* Zero rpm at the edge (slow / stopped / entered from READY): freeze on the
 * first tick, wheel left exactly where it was — no actuation at standstill. */
static void test_feedforward_zero_rpm_no_motion(void)
{
    float final_deg; bool frozen_off;
    run_emergency(/*rpm0*/ 0, /*start*/ 30.0f, &final_deg, &frozen_off);
    ASSERT_TRUE(frozen_off);
    ASSERT_NEAR_FLOAT(final_deg, 30.0f, 1e-6f);   /* unchanged */
}

int main(void)
{
    TEST_BEGIN();
    TEST_CASE(test_ramps_toward_zero);
    TEST_CASE(test_standstill_freezes);
    TEST_CASE(test_centered_stops);
    TEST_CASE(test_no_overshoot);
    TEST_CASE(test_full_ramp_duration);
    TEST_CASE(test_stop_time_estimate);
    TEST_CASE(test_feedforward_high_speed_centers);
    TEST_CASE(test_feedforward_low_speed_freezes_offcenter);
    TEST_CASE(test_feedforward_zero_rpm_no_motion);
    TEST_END();
}
