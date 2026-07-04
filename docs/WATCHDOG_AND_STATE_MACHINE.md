# Watchdog + AS state machine — `feat/15-iwdg-watchdog`

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

`Core/Src/state_manager.cpp` (AS Off/Ready/Driving/Emergency/Finished per
T14.8), `ebs_manager.cpp` (EBS init sequence + ASB checks), `app_task.cpp`
(the loop), `hardware_io.c`, `ros_globals.cpp`. Driven by CAN inputs
(`/0x503` mission, `0x504` TS, `0x505` brake, `0x506` SDC, `0x509` R2D —
ported into `rx_dispatch`) and the ADC/GPIO hardware lines.

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
- R2D reconnected: the FSM reads `g_can_r2d` (Alberto read a non-existent
  `g_can_go`).
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
- `g_brake_pressure_threshold` (`state_manager.cpp`, 1.0) — brakes-engaged
  threshold. TODO already marked in the header.
- `ebs_manager.cpp` `ACTUATOR_STORAGE_THRESHOLD` / `BRAKE_PRESSURE_THRESHOLD`
  / timeouts — confirm against the pneumatic system.
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

The uDV **does not drive the ASSI lights**, and the `IFS08-DV_AMI` repo does
**not** contain ASSI light code. These are two different rules devices:

- **ASSI** (Autonomous System *Status* Indicator, T 14.9) — the three
  yellow/blue status lights + emergency buzzer.
- **AMI** (Autonomous *Mission* Indicator, T 14.10) — the cockpit mission
  selector. That is what the `IFS08-DV_AMI` repo is: button/LCD mission
  pick that broadcasts the choice on CAN `0x503`. No status-light logic.

What uDV actually does for ASSI: the FSM computes the AS status, and uDV
**only emits a 1-byte status code on CAN**. A separate downstream **ASSI
peripheral** owns the physical illumination — the yellow/blue mapping, the
2–5 Hz / 50 % flashing in AS Driving & AS Emergency (T 14.9.1), and the
emergency buzzer (T 14.9.5). None of that lives in this firmware.

- `getAssiStatusCode()` — `state_manager.cpp:117` (ASState → byte:
  OFF 0x00 / EMERGENCY 0x01 / READY 0x02 / DRIVING 0x03 / FINISHED 0x04).
- `Can::sendAssiStatus()` — `can_interface.cpp:418`, FDCAN3 ID `0x100`,
  1-byte payload, sent on state change from `app_task.cpp:142`/`:177`, and
  re-emitted as EMERGENCY by the safety monitor (`safety_monitor.c:142`).

The `0x100` codes are a **private bus protocol** with the ASSI node, not a
rules-mandated encoding — keep them in sync with that node's firmware.

**Implications for upcoming work:**
- Do **not** add light-flashing / duty-cycle / buzzer logic to uDV, and do
  **not** try to pull such functions from `IFS08-DV_AMI` — they don't exist
  there. That logic belongs in the ASSI peripheral firmware.
- The on-board WS2812 strip (`ws2812_set_mission_color()`, driven by
  `amiTask`) is the **mission** color, unrelated to the ASSI status lights.
- Rules compliance for ASSI illumination/flashing/sound is owned by the
  ASSI node; from uDV's side, only the status-code mapping above is in scope.

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
