/**
 * @file    as_stop_latch.hpp
 * @brief   Pure DV_STOPPING latch decision (host-unit-testable).
 *
 * The end-of-mission stop (issue #176) must be STICKY: once the pipeline has
 * been believed (>= DV_STOPPING_MIN_STREAK consecutive STOPPING messages, the
 * debounce), the EBS stays fired and the torque stays zeroed for the rest of
 * the run — the mission is over and the car must not move again; it only waits
 * for AS Finished. A level read of /dv/status would instead RELEASE the brakes
 * the moment the pipeline reverts to RUNNING (byte 3), and a 7->3->7 oscillation
 * would chatter the EBS and vent the air tanks. This latch removes both.
 *
 * Pure and header-only, the same pattern as as_transition.hpp / as_actuation.hpp:
 * the caller (app_task) supplies the debounced arm signal and whether the car is
 * still in AS Driving; this only decides the next latched state. No hardware /
 * RTOS / ROS dependency, so it is exhaustively host-testable
 * (tests/host/test_as_stop_latch.cpp).
 */
#ifndef AS_STOP_LATCH_HPP
#define AS_STOP_LATCH_HPP

#include <stdint.h>

/* Consecutive DV_STATUS_STOPPING messages required before the stop is armed
 * (issue #176 debounce). At the pipeline's 10 Hz /dv/status cadence, 3 messages
 * ≈ 200–300 ms of the pipeline CONSISTENTLY commanding the stop before a single
 * newton of brake is applied — long enough that no single spurious byte 7 can
 * stab the EBS at speed, trivial against a stop that takes seconds from a car
 * that has already finished its mission. Counted per message in
 * ros_set_dv_status (see g_dv_stopping_streak); the loop rate is irrelevant. */
#ifndef DV_STOPPING_MIN_STREAK
#define DV_STOPPING_MIN_STREAK 3u
#endif

/**
 * @brief Advance the sticky DV_STOPPING latch by one tick.
 *
 * @param prev_latched  the latch state from the previous tick.
 * @param in_driving    the car is (still) in AS Driving this tick. The latch is
 *                      meaningful only during a run: it is CLEARED the instant we
 *                      leave Driving (to FINISHED / EMERGENCY / OFF), so the next
 *                      run starts fresh. Clearing on the state exit — rather than
 *                      on any pipeline byte — is what makes it un-releasable by a
 *                      7->3 revert mid-stop.
 * @param arm           the debounced, fresh, pipeline-mission-gated request to
 *                      begin the stop (>= DV_STOPPING_MIN_STREAK consecutive
 *                      STOPPING messages). Rising edge sets the latch; it is
 *                      irrelevant once latched (the stop never un-arms mid-run).
 * @return the latched state for THIS tick, to feed as_actuation()'s dv_stopping
 *         and the mission torque inhibit.
 *
 * Truth: outside Driving -> false (cleared). Inside Driving -> sticky OR of the
 * previous latch and a fresh arm, so it can only ever go false->true within a
 * run, never true->false. Idempotent while arm holds.
 */
inline bool stop_latch_next(bool prev_latched, bool in_driving, bool arm)
{
    if (!in_driving)
        return false;              /* cleared on leaving Driving — fresh next run */
    return prev_latched || arm;    /* sticky for the rest of the run              */
}

#endif /* AS_STOP_LATCH_HPP */
