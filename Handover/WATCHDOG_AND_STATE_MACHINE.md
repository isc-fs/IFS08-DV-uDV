# Watchdog + AS state machine — `feat/15-iwdg-watchdog`

*Verified against source at `dev` @ `8af44e0` (2026-07-07).*

Branch built on Carlos's latest `fix/15` (`745995d`). Three commits:

1. `feat(safety): migrate app-stall watchdog to hardware IWDG + task monitor`
2. `feat(safety): latch emergency on reboot after an IWDG reset`
3. `feat(asms): integrate fix/17 state machine (AS FSM + EBS init) onto the IWDG watchdog`

This doc is the source of truth for what was integrated and — most
importantly — the **OPEN ITEMS** that are stubbed or unverified, so they
don't get missed.

---

## 1. Hardware watchdog (IWDG + software monitor)

Two tiers (`Core/Src/iwdg.c`, `Core/Src/safety_monitor.c`,
`Core/Src/safety_eval.c`):

- **IWDG (hardware backstop)** — direct-register, ~100 ms on the LSI, no
  HAL-module/`.ioc` dependency (survives a CubeMX regen). Refreshed by
  exactly one owner: the safety supervisor task.
- **`safetyTask` (software monitor)** — high priority, ~10 ms. Watches the
  heartbeats of `imuTask`, `canTask`, and `appTask` (the state machine).
  On a stall it latches the rules-defined **safe state**: fire EBS
  (D1/D2 **LOW**) + open the SDC (D4 **LOW**) + emit ASSI EMERGENCY, and
  keeps reporting. If the monitor itself can't run (total hang), the IWDG
  resets the MCU → EBS fires via the hardware fail-safe.
- **Reboot reset-cause detection** (`main.c`) — if `RCC_FLAG_IWDG1RST` is
  set on boot, the previous reset was the watchdog (a prior fatal hang):
  assert the safe state immediately and bring `safetyTask` up **latched**
  in EMERGENCY instead of re-arming.

Rules basis: FS-Rules 2026 T15.3.2 (continuous monitoring), T15.3.4/5
(auto safe state = brakes engaged + open SDC), T11.9.2(d)/T11.9.4
(message-timeout detection ≤ 500 ms), T15.2.1 (power-loss → brake).

## 2. AS state machine (from `fix/17`, reconciled)

The live AS transition is the pure `as_next_state()` in
`Core/Inc/as_transition.hpp` (OFF/READY/DRIVING/EMERGENCY/FINISHED per
T14.8), fed each tick from `app_task.cpp` (the loop). `ebs_manager.cpp`
runs the EBS init sequence + ASB checks; `state_manager.cpp` now only
sources signals + telemetry (its old `updateState()` FSM is dead for AS
decisions — see `docs/STATE_MACHINE_INPUTS.md`). Also `hardware_io.c`,
`ros_globals.cpp`. Inputs come from the ADC/GPIO hardware lines (ASMS/TSMS,
EBS pressures) plus the CAN contract in `can_interface.cpp` (`0x503`
mission, `0x504` TS-active, `0x505` brake-over-limit verdict, `0x506` motor
rpm, `0x510`/`0x511` DV R2D request/confirm, `0x500` steering feedback,
`0x191` RES) and the DV pipeline over `/dv/status`. Note `res_go`/`res_estop`
come from the RES on `0x191`, and TS-active is sensed **locally** on the
board (TSMS A6 AND ASMS A3), not from CAN `0x504`.

**Reconciliations applied vs `fix/17`** (his branch was orphaned dead code
that didn't build):
- EBS polarity is **active-low, LOW = fire** throughout (owner-confirmed
  against the car): `hardware_io.c` `is_ebs_active()` reads a channel as
  active when it is LOW, and the write path (`ebs_manager`, `safety_monitor`,
  `main`, `freertos`, `force_ebs`) fires by driving D1/D2 LOW. LOW is the
  power-on/reset level, so the brake is fail-safe. (Supersedes the earlier
  fix/15 HIGH=fire assumption.)
- Watchdog unified onto the IWDG: `app_task` no longer drives the dead
  TIM3 timer (`htim3` was never configured); it beats `SAFETY_TASK_APP`.
- Drive trigger is the RES **GO** (`res_go` from `0x191`), not R2D: the live
  `as_next_state()` gates `READY→DRIVING` on `res_go && mission_valid &&
  (dv_ready || !mission_needs_pipeline)`. `g_can_r2d` is now the ECU R2D
  *confirm* (`0x511`) used to stop the `0x510` R2D-request loop while driving
  — it is no longer a state-transition input. (The legacy `state_manager.cpp`
  `updateState()` still references `signals_.r2d`, but that method is dead for
  AS decisions.)
- His `ros_interface.cpp` is **not** integrated (see §3).

---

## 3. ⚠️ OPEN ITEMS — stubbed / unverified (DO NOT MISS)

### O1 — Mission/actuation ROS layer is STUBBED 🔴 (functional blocker)
`Core/Src/ros_task_commands.c` is a **no-op stub** for the mission senders
(`send_set_mission_command`, `send_start_mission_command`, the cancels).
Alberto's real `ros_interface.cpp` depends on a custom **`dv_msgs` action
package that does not exist in the tree** and does not compile, so it was
left out.

Consequence: nothing sets `g_set_mission_ready` / `g_mission_going_cmd` /
`g_accel_cmd` / `g_steer_cmd`, so **the autonomous path does not advance
past AS Off via mission selection**, and no throttle/steer is streamed.
The EBS init sequence, ASSI emission, and all CAN/GPIO-driven FSM
transitions DO work.

**To close:** wire the `ros_globals` command atomics to a real ROS layer —
either author the `dv_msgs` SetMission/RuntimeControl actions + micro-ROS
`extra_packages` (rebuild the static lib), or express them as
`std_srvs`/`std_msgs` on the existing `cubemx_node`. Producers needed:
| atomic | set it from |
|---|---|
| `g_set_mission_ready` | SetMission result (or a `mission_ready` SetBool service) |
| `g_mission_going_cmd` | RuntimeControl active |
| `g_accel_cmd` / `g_steer_cmd` | `std_msgs/Float32` subscribers (`cmd/accel`, `cmd/steer`) |
| `g_finished_cmd` | `/slam/finished` relay (Bool sub or SetBool service) |

### O2 — Full `.elf` link NOT verified on a bleeding-edge toolchain 🟠
Every new/changed object compiles clean with `arm-none-eabi-gcc 15.2`, and
the host suites pass. The full link was **not** verified here because:
(a) the dev clone lacks the `micro_ros_stm32cubemx_utils` submodule, and
(b) GCC 15 turns the pre-existing micro-ROS `rmw_uros_*` implicit
declarations in `freertos.c` into hard errors (the team's older toolchain
treats them as warnings). **Build the full `.elf` with the team toolchain
before flashing.**

### O3 — SDC pin polarity needs EE sign-off 🟠 (safety)
- EBS: D1/D2 **LOW = fire** / HIGH = release (owner-confirmed, active-low,
  used throughout the code; LOW is the fail-safe power-on/reset level). This
  is settled — do not re-hedge to HIGH=fire.
- SDC: D4 **LOW = open** / HIGH = closed (matches dev's `hardware_io` and
  the power-on reset level). **Confirm D4 is the SDC-close line** — this is
  the remaining EE bench item.

### O4 — Per-vehicle constants are placeholders 🟠
- **Brakes-engaged is now the ECU's verdict, not a uDV threshold.** The old
  `g_brake_pressure_threshold` is gone: `brakes_engaged` /
  `checkBrakeLinePressure()` read `g_can_brake_over_limit` (CAN `0x505`, the
  ECU-owned `BrakeDvHardRaw` comparison). No uDV-side threshold to tune here.
- `ebs_manager.hpp` `ACTUATOR_STORAGE_THRESHOLD` (1.0) /
  `EMPTY_ACTUATOR_STORAGE_THRESHOLD` (0.1) and `ebs_manager.cpp` timeouts —
  confirm against the pneumatic system.
- `hardware_io.c` `adc_scale_factor` (1.0) — ADC raw → bar calibration.
- IWDG ~100 ms and the 100 ms monitor deadlines — tune vs T15.4.1's 200 ms
  EBS reaction if faster fault response is wanted.

### O5 — Telemetry publishers not exposed 🟢 (optional)
`app_task` writes the `g_telemetry_*` atomics, but nothing publishes them
(Alberto's `ros_interface` would have). Optionally add `telemetry/*`
`std_msgs` publishers to `cubemx_node` for debugging — no `dv_msgs` needed.

### O6 — Dead code 🟢
`hardware_io_watchdog_kick()` (now an `iwdg_refresh()` wrapper) has no
callers — the safety task is the sole IWDG owner. Kept as an available
hook; remove if undesired.

---

## 4. ASSI vs AMI — design boundary (DON'T conflate)

The uDV **drives the ASSI status lights** (the physical illumination logic
now lives in this firmware), but the `IFS08-DV_AMI` repo does **not** contain
ASSI light code. These are two different rules devices:

- **ASSI** (Autonomous System *Status* Indicator, T 14.9) — the yellow/blue
  status lights + emergency buzzer.
- **AMI** (Autonomous *Mission* Indicator, T 14.10) — the cockpit mission
  selector. That is what the `IFS08-DV_AMI` repo is: button/LCD mission
  pick that broadcasts the choice on CAN `0x503`. No status-light logic.

What uDV does for ASSI has two independent outputs, both computed from the AS
state the FSM produces:

1. **Physical WS2812 lights** — `app_task` publishes the AS mode via
   `assi_set_mode()`; `assi_task.c` (`StartAssiTask`, BelowNormal prio) maps
   the mode onto colours and **owns the flash timing itself**: yellow steady
   in READY, yellow flashing in DRIVING, blue flashing in EMERGENCY, blue
   steady in FINISHED, off in OFF. Flash rate is `ASSI_HALF_PERIOD_MS` = 150 ms
   half-period → 3.3 Hz at 50 % duty (T 14.9.1, mid-band). The strip is not
   driven directly (3.3 V-vs-5 V DIN issue): `ws2812.c` sends newline-framed
   1-char commands (`a`=off, `b`=yellow, `c`=blue) over **USART10** (PG12 TX,
   115200 8N1) to an Arduino Nano that drives the strip at 5 V. See
   `docs/ASSI_UART_BRIDGE.md`.
2. **A 1-byte status code on CAN `0x100`** (FDCAN3), emitted in parallel for
   any downstream node that mirrors the AS state.

- `assi_task.c` mode→colour + flash timing (`ASSI_HALF_PERIOD_MS`,
  `StartAssiTask`); `ws2812.c` UART bridge to the Arduino.
- `getAssiStatusCode()` — `state_manager.cpp:125` (ASState → byte:
  OFF 0x00 / EMERGENCY 0x01 / READY 0x02 / DRIVING 0x03 / FINISHED 0x04).
- `Can::sendAssiStatus()` — `can_interface.cpp:595`, FDCAN3 ID `0x100`,
  1-byte payload, sent on state change from `app_task.cpp:443`, and
  re-emitted as EMERGENCY by the safety monitor (`safety_monitor.c:147`,
  via `can_interface_send_assi_emergency_from_isr`).

The `0x100` codes are a **private bus protocol**, not a rules-mandated
encoding — keep them in sync with any consumer.

**Still not implemented / out of scope:**
- The **emergency buzzer** (T 14.9.5) is not driven by this firmware yet.
- Do **not** try to pull light/mission functions from `IFS08-DV_AMI` — the
  mission-selector repo has no status-light or buzzer logic. There is **no**
  separate on-board "mission colour" strip and no `amiTask` in this firmware;
  the WS2812 strip shows the **ASSI status** colours above.

---

## 5. Tests

Host suites (no HAL/ROS, run on a laptop):

```bash
cd tests/host && make        # builds + runs all suites
```

- `build_integrity` (`check_build_integrity.sh`) — **static build-config
  guard.** The logic suites link a hand-picked source subset, so they are
  blind to a mis-wired firmware build. This script catches that class:
  (1) Makefile ⇄ CMakeLists source lists are identical — the drift that
  silently dropped the whole watchdog/state-machine set from the CMake build
  and broke the link with `undefined reference to safety_* / Start*Task`;
  (2) every referenced source exists on disk; (3) no orphan `Core/Src`
  source left out of the build; (4) no duplicate entries; (5) `Start*Task`
  entry points defined in C++ are `extern "C"` (else the mangled name
  fails to link). **Add new sources to BOTH build files** or this fails.
- `test_safety_eval` — IWDG-monitor stall detection (1130 checks).
- `test_state_machine` — real `state_manager`/`ebs_manager` linked against
  a controllable `hardware_io` stub: ASSI codes, the full EBS init
  sequence (happy + timeout + pressure-failure + checks), and every AS
  transition incl. EMERGENCY/FINISHED/SDC-open branches (40 checks).

Bench validation still required (P1–P4 in `docs/PHYSICAL_TESTS.md` plus an
EBS-init dry run) once O1–O3 are closed.
