/**
 * @file    bench_stubs.h
 * @brief   Bench stub toggles — THE one file you edit to stub a dependency.
 *
 * Mirrors the ECU pattern (IFS08-CE-ECU ecu_config.hpp + bench/car-stubs):
 * the stub hooks live in the main code permanently, gated on the constants
 * below. dev keeps everything 0 (= real behavior, stub branches compile to
 * nothing); a bench build flips the toggle(s) here, rebuilds and flashes.
 * Keep bench flips on a `bench/...` branch as a single one-line commit on
 * top of dev — never merge a 1 to dev.
 *
 * The toggles are consumed as `if (BENCH_STUB_X)` — a constant the compiler
 * folds away, so a car build carries zero stub code, but BOTH branches are
 * always compiled (no #ifdef bitrot). When any toggle is 1 the firmware
 * announces it on /debug at boot ("BENCH STUBS COMPILED IN") so a stubbed
 * image can never masquerade as a flight build.
 *
 *   BENCH_STUB_EBS_INIT — EbsManager::initSequenceStep() reports Done
 *     immediately, skipping the actuator/pressure self-check that needs the
 *     EBS pneumatics + SDC feedback (without them CheckActuator1 can never
 *     pass and the car can never arm). activate/deactivateEBS() still drive
 *     D1/D2 for real — observable on a scope.
 *     NOTE: the skipped sequence also performs hardware_io_set_as_close_sdc()
 *     on the way through WaitTS; with the stub on, the AS SDC relay is never
 *     closed. Fine when TSMS is driven by a bench switch; not fine if the rig
 *     emulates car wiring through the SDC relay.
 *
 *   BENCH_STUB_DVPC — while NO /dv/status has ever been received (no DVPC
 *     on the bench), the AS inputs are forced to a fresh pipeline READY so
 *     pipeline missions can arm/drive and the lost-heartbeat watchdog does
 *     not fire. The moment a real pipeline publishes, its bytes win and the
 *     stub goes inert for the rest of the power cycle.
 *
 *   BENCH_STUB_RES — while NO real RES frame (0x191) has ever arrived (no RES
 *     box on the bench), can_c_get_res_status() reports a healthy link (OK: no
 *     e-stop, no GO) instead of the never-received/timeout code. This is what
 *     lets a bare bench reach AS READY (rule 8 gates on res_ok), since the RES
 *     GO/status is consumed from CAN 0x191, not the A2 GO_RES pin. The moment a
 *     real RES frame arrives the stub goes inert (real e-stop/GO/timeout win),
 *     so a CAN tool sending a real 0x191 GO still drives READY->DRIVING.
 *     NOTE: with no real RES, the status is stuck at OK — it never returns GO,
 *     so the car reaches READY but does not self-advance to DRIVING. Send a
 *     real 0x191 GO frame (or use the real RES) to cross the GO edge.
 */
#ifndef BENCH_STUBS_H
#define BENCH_STUBS_H

/* ==== Toggle stubs here: 0 = real behavior (dev default), 1 = stubbed ==== */

/* TEMPORARY (pre-production bench/car testing): these stubs are ON. This is a
 * deliberate, owner-approved exception to the "never merge a 1 to dev" rule
 * while there is no EBS-pneumatics rig / DVPC / RES box on the bench. REVERT
 * ALL TO 0 before any production / competition run (the firmware still
 * announces "BENCH STUBS COMPILED IN" on /debug at boot so a stubbed image is
 * obvious). */

/* EBS init self-check -> Done immediately */
#define BENCH_STUB_EBS_INIT   1
/* fake fresh pipeline READY until the first real /dv/status arrives */
#define BENCH_STUB_DVPC       1
/* fake a healthy RES link (OK, no e-stop/GO) until a real 0x191 arrives */
#define BENCH_STUB_RES        1

#endif /* BENCH_STUBS_H */
