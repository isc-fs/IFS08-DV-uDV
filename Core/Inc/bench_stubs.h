/**
 * @file    bench_stubs.h
 * @brief   Bench stub toggles — the ONE place a bench dependency is faked.
 *
 * Mirrors the ECU pattern (IFS08-CE-ECU ecu_config.hpp + bench/car-stubs): the
 * stub hooks live in the main code PERMANENTLY, gated on the constants below.
 * Each is consumed as `if (BENCH_STUB_X)` — a compile-time constant the compiler
 * folds away, so a car build carries zero stub code, but BOTH branches always
 * compile (no #ifdef bitrot). When any toggle is 1 the firmware announces it on
 * /debug at boot ("BENCH STUBS COMPILED IN") so a stubbed image can never
 * masquerade as a flight build; the pit-diag stub mask carries them too.
 *
 * === dev is flight-clean: every toggle DEFAULTS TO 0 here. ===
 * Do NOT edit the defaults in this file to 1. Turn a stub on for a bench build
 * ONE of two ways, neither of which touches dev:
 *   (a) preferred — pass it as a build define, no source edit, no branch:
 *         make BENCH="-DBENCH_STUB_STEERING=1 -DBENCH_STUB_DVPC=1"
 *       (the Makefile forwards $(BENCH) into the compile flags). Same source as
 *       flight; the difference lives only in the build command.
 *   (b) a thin `bench/<scenario>` branch: a single commit that #defines the
 *       toggle(s) ABOVE the guarded defaults below. Never merge it to dev.
 *
 * Because each toggle is `#ifndef`-guarded, a -D override (a) or a bench branch
 * #define (b) wins and this file's 0 is skipped.
 *
 *   BENCH_STUB_EBS_INIT — EbsManager::initSequenceStep() reports Done at once,
 *     skipping the WHOLE self-check. NO actuator fires and the AS SDC line is
 *     NOT closed — the "nothing wired" desk-bench shortcut. MUST be 0 for on-car
 *     valve bring-up (use EBS_SENSORS instead) — at 1 the sequence is skipped so
 *     D1/D2 never fire and the AS SDC relay (D4) stays open.
 *
 *   BENCH_STUB_EBS_SENSORS — on-car EBS valve bring-up WITHOUT compressed air.
 *     Runs the REAL init sequence (actuators fire A1 then A2, SDC line closed for
 *     real); only the pressure / SDC-feedback SENSOR reads are faked to pass
 *     (checkStoragePressures A5/A4, checkBrakeLinePressure 0x505, the Start/WaitLow
 *     SDC-ready read). The TS wait (ASMS A3 ∧ TSMS A6) stays REAL. Do NOT set with
 *     EBS_INIT (EBS_INIT wins and suppresses actuation — see the #error below).
 *
 *   BENCH_STUB_SDC — fake only the SDC-ready read (A1=RES_1_IN on this PCB).
 *     Redundant while EBS_SENSORS=1; exists to fake SDC on its own and to run the
 *     EBS pressure checks for real. Does NOT touch the D4 SDC drive.
 *
 *   BENCH_STUB_DVPC — while NO /dv/status has ever arrived (no DVPC on the bench)
 *     the AS inputs are forced to a fresh pipeline READY so pipeline missions can
 *     arm/drive and the lost-heartbeat watchdog can't fire. The moment a real
 *     pipeline publishes, its bytes win and the stub goes inert for the rest of
 *     the power cycle.
 *
 *   BENCH_STUB_RES — while NO real 0x191 has arrived, can_c_get_res_status()
 *     reports a healthy link (OK: no e-stop/GO) so a bare bench reaches AS READY.
 *     Goes inert the moment a real RES frame arrives. NOTE: with a real RES box
 *     present, keep this 0 or it pins the status to OK and NEVER reports GO.
 *
 *   BENCH_STUB_STEERING — fully decouple the steering: the uDV sends NOTHING to
 *     the steering board (no 0x010 motor-start, no 0x020 angle) AND ignores its
 *     ESTADO_MOTOR feedback so a latched EMERGENCIA can't trip AS. For an
 *     UNCALIBRATED steering that must not be driven. Calibration (0x30) is NOT
 *     suppressed — that path must still work to bring the steering up.
 *
 *   BENCH_STUB_IMU_ROS — suppress ONLY the ROS `/imu` publish (ros_task.c).
 *     For the `prerun/` branch type: replaying a recorded rosbag ON the car so
 *     the bag's `/imu` is the only source (else the live publish and the bag
 *     collide on the same topic). IMU sampling, attitude, and the 0x512 CAN
 *     IMU broadcast to the ECU all stay LIVE — only the ROS topic is gated,
 *     and the `/imu` publisher entity is still created so micro-ROS entity
 *     counts are unchanged (no lib rebuild). See PRERUN.md on the prerun branch.
 */
#ifndef BENCH_STUBS_H
#define BENCH_STUBS_H

/* ============================================================================
 * prerun/ BRANCH ACTIVATION — DO NOT MERGE TO dev. See PRERUN.md.
 * This is the ONLY functional divergence of prerun/ from dev: it #defines the
 * IMU-publish stub to 1 ABOVE the guarded defaults below (the documented
 * "thin bench branch" pattern), so every build on this branch suppresses the
 * live ROS /imu publish and a replayed rosbag's /imu is the only source.
 * Everything else stays flight-clean (all other stubs 0). Keep this the sole
 * source change here so rebasing onto dev stays a 2-line divergence.
 * ============================================================================ */
#define BENCH_STUB_IMU_ROS 1

/* ==== flight defaults: ALL 0. Override per bench build via -D (see header). ==== */
#ifndef BENCH_STUB_EBS_INIT
#define BENCH_STUB_EBS_INIT    0
#endif
#ifndef BENCH_STUB_EBS_SENSORS
#define BENCH_STUB_EBS_SENSORS 0
#endif
#ifndef BENCH_STUB_SDC
#define BENCH_STUB_SDC         0
#endif
#ifndef BENCH_STUB_DVPC
#define BENCH_STUB_DVPC        0
#endif
#ifndef BENCH_STUB_RES
#define BENCH_STUB_RES         0
#endif
#ifndef BENCH_STUB_STEERING
#define BENCH_STUB_STEERING    0
#endif
#ifndef BENCH_STUB_IMU_ROS
#define BENCH_STUB_IMU_ROS     0
#endif

/* EBS_INIT skips the whole sequence (no actuation); EBS_SENSORS runs it for real
 * with faked reads. Setting both means EBS_INIT wins and silently suppresses the
 * actuation EBS_SENSORS was turned on to get — never useful, so make it an error. */
#if BENCH_STUB_EBS_INIT && BENCH_STUB_EBS_SENSORS
#error "BENCH_STUB_EBS_INIT and BENCH_STUB_EBS_SENSORS are mutually exclusive (EBS_INIT skips the actuation EBS_SENSORS runs)."
#endif

#endif /* BENCH_STUBS_H */
