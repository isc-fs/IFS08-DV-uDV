# uDV branch freshness & comparison report

> **Audience: a future Claude (or engineer) triaging branches for *freshness*.**
> "Freshness" = *does this branch contain work not yet in `dev` that we might still
> want, or is it fully superseded and safe to delete?* This document gives you the
> `dev` baseline for every subsystem, then compares each open branch against it so
> you can answer that without re-reading every diff yourself.
>
> - **Repo:** `IFS08-DV-uDV` (STM32H733XG firmware, FreeRTOS + micro-ROS).
> - **Baseline:** `dev` @ `8af44e0` (this report's reference point).
> - **Generated:** 2026-07-07. **Regenerate** when `dev` moves — see [§7](#7-how-to-regenerate).
> - **Sibling docs:** [`MISSIONS.md`](MISSIONS.md), [`STATE_MACHINE_INPUTS.md`](STATE_MACHINE_INPUTS.md),
>   [`WATCHDOG_AND_STATE_MACHINE.md`](WATCHDOG_AND_STATE_MACHINE.md),
>   [`PIPELINE_INTERFACE.md`](PIPELINE_INTERFACE.md), [`fsm_visualizer.html`](fsm_visualizer.html).

---

## 0. TL;DR — freshness verdicts

| Branch | Ahead / Behind dev | Verdict | Fresh value? |
|---|---|---|---|
| `feat/23-mission-dispatch` | 0 / 3 | **Merged** (PR #88) | none — delete |
| `feat/23-steer-0x020-imu` | 0 / 13 | **Merged** (PR #90) | none — delete |
| `feat/18-ami-mission-ack` | 0 / 53 | **Merged** | none — delete |
| `fix/19-uart-assi` | 0 / 57 | **Merged** | none — delete |
| `fix/imu-topic-docs` | 0 / 26 | **Merged** | none — delete |
| `fix/steering-0x500-status-dlc` | 1 / 52 | **⭐ PARTLY FRESH** | steering-status **ROS observability** not in dev |
| `refactor/steer-feedback-can-id` | 1 / 25 | **Superseded** (already in dev) | none — the rename is in dev |
| `feat/22-ecu-comms` | 1 / 16 | **Superseded** (PR #90 + dev 0x512) | none of substance |
| `feat/18-ebs` | 3 / 128 | **Stale/superseded** | car-proven behaviour already extracted (#96) |
| `feat/17-inspection` | 5 / 128 | **Stale/superseded** | pre-refactor inspection work |
| `feat/17-ECU-inspection_redy` | 6 / 128 | **Stale/superseded** | pre-refactor inspection + ECU |
| `fix/17` | 5 / 167 | **Historical ancestor** | the original monolith — do not resurrect |
| `assi-dma` *(local-only)* | n/a (unpushed) | **Keep** | WS2812 DMA refactor, **never pushed** |

**One actionable item:** cherry-pick the steering-status observability from
`fix/steering-0x500-status-dlc` (details in [§4.1](#41-fixsteering-0x500-status-dlc--partly-fresh)).
Everything else is merged, superseded, or historical.

---

## 1. `dev` baseline — general functioning

The board is the **uDV** (micro Driverless Vehicle) autonomy controller for a
Formula Student Driverless car. It owns the **AS (Autonomous System) state
machine**; the DV pipeline (LattePanda / DVPC) only *reacts* to the state byte
the uDV publishes. FreeRTOS tasks (see `CLAUDE.md` for the full table):

- **`appTask`** ([`app_task.cpp`](../Core/Src/app_task.cpp)) — AS state machine + EBS init sequence + mission dispatch. **The heart of everything in this report.**
- **`canTask`** — drains RX queues (FDCAN1 RES, FDCAN3 AMI/steering), data-logger TX.
- **`defaultTask`** ([`ros_task.c`](../Core/Src/ros_task.c)) — micro-ROS node: all publishers/subscribers/services + `/debug` state dump.
- **`imuTask`**, **`assiTask`**, **`safetyTask`** — IMU 400 Hz, ASSI LED renderer, two-tier watchdog.

Control flow each `appTask` tick: read signals → build `AsInputs` → `as_next_state()` →
act on the resulting AS state (EBS actuation, steering lifecycle, mission dispatch, ASSI mode).

### 1.1 CAN bus layout (authoritative on `dev`)

| Bus | Role | RX | TX |
|---|---|---|---|
| **FDCAN1** | RES CANopen (node `0x11`) | `0x191` PDO (e-stop / go / radio quality) | NMT `0x000`, data-logger `0x500/0x501/0x502` |
| **FDCAN2** | ACU bus (ECU/VCU + AMS) | `0x504` TS-active, `0x505` brake-over-limit, `0x506` motor rpm, `0x511` DV-R2D confirm | `0x507` torque, `0x510` DV-R2D request, `0x512` IMU broadcast (50 Hz) |
| **FDCAN3** | AMI + steering (local DV peripherals) | `0x503` mission select, `0x2B0` steer sensor, `0x500` steer feedback | `0x50A` mission ACK, `0x100` ASSI, `0x010` steer-motor start, `0x020` steer angle |

> ⚠️ **`0x500` is two different frames on two buses**: RX on FDCAN3 = steering
> feedback (`CAN_ID_STEER_FEEDBACK`); TX on FDCAN1 = data-logger DYN1 (`CAN_ID_DL_DYN1`).
> `dev` already keeps them as distinct named constants — the `refactor/steer-feedback-can-id`
> branch's whole purpose ([§4.2](#42-refactorsteer-feedback-can-id--superseded)).

### 1.2 ROS topics (micro-ROS node `cubemx_node`, USB-CDC)

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/imu` | `sensor_msgs/Imu` | pub | 400 Hz, canonical both sides (no remap) |
| `/imu/status` | `std_msgs/Int32` | pub | |
| `/steering_angle` | `std_msgs/Float32` | pub | rad |
| `/motor_rpm` | `std_msgs/Float32` | pub | |
| `/steering/feedback` | `std_msgs/Float32MultiArray` | pub | **3 elements on dev**: [actual, target, motor] deg. ← see [§4.1](#41-fixsteering-0x500-status-dlc--partly-fresh) |
| `/res/status`, `/res/go` | `std_msgs/Int32` | pub | RES status codes |
| `/ami/mission` | `std_msgs/Int32` | pub | selected mission index |
| `/assi/state` | `std_msgs/UInt8` | pub | AS status byte, T14.9, 10 Hz wall-clock |
| `/assi/pub_gap_max_ms` | `std_msgs/Int32` | pub | heartbeat health |
| `/as_state` | `std_msgs/UInt8` | pub | raw AS state (OFF=0…FINISHED=4) |
| `/debug` | `std_msgs/String` | pub | **grouped one-line state snapshot** (AS/ASMS/TS/SDC/EBS/RES/EBSinit) — primary on-car diagnostic |
| `/dv/status` | `std_msgs/UInt8` | sub | DVPC → uDV pipeline state / heartbeat |
| `/ctrl/cmd` | `geometry_msgs/Twist` | sub | pipeline throttle+steer |
| `/cmd_test` | `std_msgs/Int32` | sub | bench test |
| `/activate_steering`, `/force_ebs` | `std_srvs/SetBool` | srv | manual steering enable / EBS fire |

---

## 2. `dev` baseline — the four focus subsystems

These are the reference definitions. Every branch section below is phrased as a
*delta* against this.

### 2.1 FSM — `Core/Inc/as_transition.hpp` + `app_task.cpp`

Pure, host-tested decision core `ASState as_next_state(prev, AsInputs)`.
States: **OFF=0, READY=1, DRIVING=2, EMERGENCY=3, FINISHED=4**.

**Fail-safe evaluation order (first match wins):**
1. `!asms_on` → **OFF** (always wins).
2. `prev==EMERGENCY` → **EMERGENCY** (latches until ASMS-off).
3. `prev==FINISHED` → **FINISHED** (latches until ASMS-off).
4. Emergency trigger → **EMERGENCY**: `res_estop` ∨ `steer_emergency` ∨ (`!ts_on` while R/D) ∨ (`dv_emergency` ∧ pipeline-mission while R/D) ∨ `dv_lost_driving`.
5. `prev==DRIVING` ∧ (`mission_complete` ∨ (pipeline ∧ `dv_finished`)) → **FINISHED**.
6. `prev==DRIVING` → **DRIVING** (sticky; GO is a trigger, not a level).
7. `res_go` ∧ `prev==READY` ∧ `mission_valid` ∧ (`dv_ready` ∨ standalone) → **DRIVING**.
8. `res_ok` ∧ `ts_on` ∧ `ebs_init_done` → **READY**.
9. else hold `prev`.

**Input sourcing** (`app_task.cpp`, `AsInputs`): `res_*` from
`can_c_get_res_status()` (0x191); `steer_emergency` from
`g_steer_motor_state == ESTADO_MOTOR_EMERGENCIA` (0x500 byte 5); `ebs_init_done`
from `ebs.getInitState()==Done`; `dv_*` from `/dv/status`; **`BENCH_STUB_DVPC`**
forces a fresh pipeline READY until a real `/dv/status` arrives.

**Safety invariants (do not regress):** `mission_valid` (unknown/SHUTDOWN never
drives), `active_mission` binding (run bound to the mission captured at GO),
continuous steer-start (re-send `0x010=1` every DRIVING tick). Interactive model:
[`fsm_visualizer.html`](fsm_visualizer.html).

### 2.2 EBS — `Core/Src/ebs_manager.cpp` + `bench_stubs.h`

- **Polarity (settled):** D1/D2 (PB4/PB5) **LOW = fire**, **HIGH = release**; LOW is the power-on/reset level → fail-safe braked.
- **Released only in AS DRIVING**; engaged in OFF/READY/EMERGENCY/FINISHED/manual. Stray releases from `reset()`/`force_ebs`/init-self-test are re-engaged within one `appTask` tick.
- **Init FSM:** `Start → WaitLow → CheckPressure → WaitTS` (closes AS SDC via D4) `→ CheckActuator1 → WaitInterActuatorCheck → CheckActuator2 → Done` (or `Failed`). `ebs_init_done` gates OFF→READY.
- **`BENCH_STUB_EBS_INIT=1`** (currently on `dev`, pre-production): jumps straight to `Done`, **skipping the SDC-close** — fine on a bench switch, not through car SDC wiring. Revert to `0` before racing.
- Storage pressures A4/A5; brake-line verdict via ECU `0x505`.

### 2.3 Steering — `can_interface.cpp` (FDCAN3, shared with the steering board's FDCAN1)

- **RX `0x500`** (`CAN_ID_STEER_FEEDBACK`, 20 Hz): `dlc<6` guard; bytes [2..4] = actual/target/motor angle (int8 × 0.5°); **byte [5] = motor state** → `g_steer_motor_state` (`ESTADO_MOTOR_OFF=0`, … `ESTADO_MOTOR_EMERGENCIA=-1`).
- **byte [5] is wired into the FSM**: `EMERGENCIA(-1)` → `steer_emergency` → unconditional **EMERGENCY**. This decode went *live only after the RX-dlc fix* (PR #92) — before, `dlc` was always 0 so the frame was dropped.
- **TX `0x010`** motor start (`1`=on, re-sent every DRIVING tick, car-proven PR #96), **`0x020`** angle (`deg×100` int32 LE).
- **Pipeline steering** (`/ctrl/cmd`): `norm[-1,1] × STEER_FULL_LOCK_DEG (65°)` on `0x020`, paced to the ECU 20 ms cycle; `0x508` retired. Full-lock/ratio/sign are commissioning item **#71**.
- **ROS:** `/steering_angle` (rad) + `/steering/feedback` (**3 elements** on dev). **dev has no ROS/CAN API for the steering *motor status*** other than the FSM-internal `g_steer_motor_state`.

### 2.4 AMI — `can_interface.cpp` (`0x503`/`0x50A`) + `mission_registry.cpp`

- **RX `0x503`** mission select, `dlc<1` guard: `g_can_mission_id = data[0]` (0-based AMI menu index).
- **TX `0x50A`** ACK echoes `data[0]` verbatim; the AMI retransmits `0x503` every 500 ms until it sees the echo. **The ACK is NOT an FSM prerequisite** — the FSM reads `g_can_mission_id`, which defaults to Inspection if never set.
- **Mission codes (settled):** Manual=0, Accel=1, Skidpad=2, Autocross=3, **Trackdrive=4, EBS-test=5, Inspection=6**, Shutdown=7. Pipeline missions = {1–4}; standalone = Inspection/EBS-test.
- **Dispatch:** `mission_for_code()` → `const Mission*` vtable (pure `MissionCtx → MissionCommand`); `nullptr` = invalid → `mission_valid=false` → GO refused. See [`MISSIONS.md`](MISSIONS.md).

---

## 3. Branch inventory (measured vs `origin/dev` @ `8af44e0`)

Merged branches (0 unique commits) — content is fully in `dev`, remotes may or may
not still exist. **Delete on sight; no freshness.**

| Branch | Behind | Was |
|---|---|---|
| `feat/23-mission-dispatch` | 3 | PR #88 (mission dispatch) |
| `feat/23-steer-0x020-imu` | 13 | PR #90 (pipeline→0x020, IMU 0x512) |
| `feat/18-ami-mission-ack` | 53 | AMI ACK handshake |
| `fix/19-uart-assi` | 57 | EBS polarity / UART ASSI |
| `fix/imu-topic-docs` | 26 | `/imu` topic standardisation |

---

## 4. Open branches with unique commits — detailed comparison

### 4.1 `fix/steering-0x500-status-dlc` — ⭐ PARTLY FRESH

- **Divergence:** 1 ahead / 52 behind. Single commit `6a63f58` (*"decode steering 0x500 motor-status byte + DLC guard"*, Raul Moran, 2026-07-04).
- **Files:** `can_globals.h/.cpp`, `can_interface.cpp`, `ros_task.c`.

**Steering — what it adds vs dev:**
- New atomics `g_steer_motor_status` + `g_steer_fb_last_rx_tick`, plus two C accessors:
  - `can_c_get_steer_motor_status()` — raw byte.
  - `can_c_get_steer_status(now, timeout)` — **freshness-aware**, mirroring `can_c_get_res_status`: `-2` never rx / `-1` silent(timeout) / `0` OFF / `1` ON / `2` EMERGENCIA.
- Publishes it as a **4th element on `/steering/feedback[3]`** (array grows 3→4, seed `-2`), with `STEER_FB_TIMEOUT_MS = 300`.

**ROS topics — delta:** `/steering/feedback` becomes **4-element** `[actual, target, motor, status]`.

**FSM/EBS — delta:** *none.* The commit message explicitly defers wiring
EMERGENCIA into the AS state machine as "a safety-FSM design decision for the DV
team." **dev went the other way** — it already wires `g_steer_motor_state`
straight into `steer_emergency`/EMERGENCY (§2.3), but does **not** expose the
freshness-aware status or the silent-board (`-1` timeout) distinction over ROS.

**AMI — delta:** none.

**Freshness verdict:** the FSM wiring is already in dev (via a different global,
`g_steer_motor_state`, using `CAN_ID_STEER_FEEDBACK`). The **genuinely fresh part
is the observability**: a freshness-aware `can_c_get_steer_status()` and the
`/steering/feedback[3]` publication — which is *exactly* the gap flagged during
the "car won't reach AS Ready" investigation (dev can't currently *show* a
steering EMERGENCIA/silent board over ROS; you infer it by elimination).

> **Recommended action:** cherry-pick only the observability slice. Reconcile
> naming to dev (`g_steer_motor_state` / `CAN_ID_STEER_FEEDBACK`, not this branch's
> `g_steer_motor_status` / `CAN_ID_DL_DYN1`), keep the `-1`/`-2` timeout semantics,
> and add the 4th `/steering/feedback` element **plus** a `steer:<status>` field to
> the `/debug` line. Small, safe, host-testable.

### 4.2 `refactor/steer-feedback-can-id` — Superseded

- **Divergence:** 1 ahead / 25 behind. Single commit `6a180ae`, pure rename: give the FDCAN3 `0x500` RX frame its own constant `CAN_ID_STEER_FEEDBACK` (was overloaded with `CAN_ID_DL_DYN1`). No behaviour change.
- **Delta vs dev:** **none — dev already has this.** `can_interface.cpp` on `8af44e0` defines `CAN_ID_STEER_FEEDBACK = 0x500u` (line 50) and uses it in `rx_dispatch` (line 327); `CAN_ID_DL_DYN1` is kept for the TX frame. The rename was already merged.
- **Freshness verdict:** **stale — delete.** Kept as a live remote only by oversight.

### 4.3 `feat/22-ecu-comms` — Superseded

- **Divergence:** 1 ahead / 16 behind. Single commit: *"pipeline steering to 0x020, retire 0x508, restore the IMU broadcast."*
- **Steering delta:** pipeline `/ctrl/cmd` → `0x020` at 65° full-lock, `0x508` retired. **Already in dev** via PR #90 (§2.3).
- **CAN comms delta:** restores an IMU→ECU broadcast — **but at `0x001`**, whereas **dev already broadcasts IMU at `0x512`** from `imu_task` (`CAN_ID_IMU = 0x512`, §1.1). This branch is the earlier `0x001` iteration, since superseded.
- **FSM/EBS/AMI:** touches `app_task.cpp` only for the steering plumbing above.
- **Freshness verdict:** **superseded — delete.** Nothing here is newer than dev; the `0x001` IMU id is actively *wrong* relative to the settled `0x512`.

### 4.4 `feat/18-ebs` — Stale / superseded (car-proven, already extracted)

- **Divergence:** 3 ahead / 128 behind. Commits: *"mision inspeccion"*, *"astart stop dir"*, *"añadir continus motor start"*.
- **Context:** the last-session car-proven branch. PR #94 was **closed without merging** (126+ behind); its one durable behaviour — continuous steering-motor start while DRIVING — was re-implemented cleanly on dev as **PR #96** (§2.3).
- **Steering delta:** continuous `0x010` start — **already in dev** (#96).
- **EBS delta:** *"activate ebs on finish"* semantics — dev already engages EBS in FINISHED (§2.2).
- **FSM/AMI delta:** pre-refactor inspection mission logic living directly in `app_task.cpp`; **superseded** by the mission-dispatch vtable (§2.4).
- **Freshness verdict:** **stale.** Keep the remote for provenance if desired, but nothing to cherry-pick — it's 128 commits behind the current architecture.

### 4.5 `feat/17-inspection` & `feat/17-ECU-inspection_redy` — Stale / superseded

- **Divergence:** 5 / 128 and 6 / 128 respectively (the `-ECU-` one is a superset).
- **Commits:** `mision inspeccion`, `astart stop dir`, `añadir continus motor start`, `unificacion de numero de mision inspecion`, `activate ebs on finish` (+ ECU inspection-ready on the second).
- **Deltas:** rewrites `app_task.cpp` (+132) and `can_interface.cpp` (+247) in the **pre-mission-dispatch monolith** style. The mission-numbering unification (`unificacion de numero de mision`) predates the settled AMI enum (Inspection=6, §2.4) — its numbering was the *wrong* `0/1/2` scheme.
- **Freshness verdict:** **stale/superseded.** Historical inspection-mission development; entirely replaced by PR #88 + the settled AMI codes. Do not resurrect the numbering.

### 4.6 `fix/17` — Historical ancestor (do not resurrect)

- **Divergence:** 5 ahead / **167 behind** — the most divergent branch.
- **Commits:** `ros interace standard comunmication`, `r2d ecu comunication`, `comunicacion res go y esperar al ecu`, `comienzo cambio watchdog`, plus a `fix/15` merge.
- **Deltas:** this is effectively the **original monolith** — it *adds* `state_manager.cpp` (+133), `ebs_manager.cpp` (+144), `app_task.cpp` (+328), `can_interface.cpp` (+262) as net-new, i.e. it predates the extraction of the pure FSM/EBS modules. It is the ancestor from which the current architecture descended (and the branch that "worked on the car" precisely because it *lacked* the defensive `dlc<1` guard — see PR #92 root-cause).
- **Freshness verdict:** **historical only.** Useful as a reference for "how the car behaved before the refactor," never as a merge source.

---

## 5. Local-only branches (no remote)

| Branch | Tip | Status |
|---|---|---|
| `assi-dma` | `2782671` | **Keep — unpushed work.** `refactor(assi): non-blocking WS2812 via TIM4_CH3 PWM+DMA; document buzzer gap`. Not on any remote; deleting loses it. Relates to the board `ASSI_DIN`/D5 path (currently dev drives ASSI over UART10→Arduino instead). Decide: push as `feat/N-assi-dma` or drop. |

Local pointers with a live upstream but no unique commits (`feat/23-mission-dispatch`,
`fix/19-uart-assi`, `fix/imu-topic-docs`, `refactor/steer-feedback-can-id`, `main`)
and the dead integration pointer `integration/fix19-into-dev` (tracks `origin/dev`,
116 behind, 0 ahead) are housekeeping, not freshness.

**Deleted this session** (stale locals whose remote was `[gone]`): `feat/16-stock-pipeline-interface`,
`feat/inspection-missions`, `fix/16-leds`, `fix/21-can-rx-dlc`, `fix/22-continuous-steer-start`,
`list`, `bench/stubs-no-ebs-no-dvpc` (all recoverable from reflog).

---

## 6. Freshness recommendations (ordered)

1. **Cherry-pick** the steering-status observability from `fix/steering-0x500-status-dlc` ([§4.1](#41-fixsteering-0x500-status-dlc--partly-fresh)) — reconciled to dev naming; add `/steering/feedback[3]` + a `/debug` `steer:` field. This is the only branch with net-new value, and it directly serves the open AS-Ready diagnosis.
2. **Decide `assi-dma`** — push or drop; it is the only unpushed original work.
3. **Delete as superseded:** `refactor/steer-feedback-can-id`, `feat/22-ecu-comms` (remote + local).
4. **Leave for provenance / delete at will:** `feat/18-ebs`, `feat/17-inspection`, `feat/17-ECU-inspection_redy`, `fix/17` — all ≥128 behind, zero cherry-pick value.
5. **Open commissioning items** unaffected by any branch above: steering full-lock/ratio/sign (**#71**), revert `BENCH_STUB_*` to `0` before racing, `r2d` direction (`STATE_MACHINE_INPUTS.md`).

---

## 7. How to regenerate

```bash
cd IFS08-DV-uDV && git fetch --prune origin
# divergence of every remote branch vs dev:
for b in $(git branch -r | grep -vE 'HEAD|origin/dev$'); do
  ab=$(git rev-list --left-right --count origin/dev...$b)
  echo "$b  behind/ahead: $ab"
done
# what a branch uniquely adds (freshness):
git log  --oneline origin/dev..origin/<branch>
git show origin/<branch>            # inspect the actual patch
```
Compare each unique commit's patch against the [§2](#2-dev-baseline--the-four-focus-subsystems)
baseline. A branch is **fresh** only if it changes FSM / EBS / steering / AMI / CAN /
ROS behaviour in a way `dev` does not already implement (often under different
names — always check semantics, not just constants). Update [§0](#0-tldr--freshness-verdicts)
and the baseline commit at the top when `dev` moves.
