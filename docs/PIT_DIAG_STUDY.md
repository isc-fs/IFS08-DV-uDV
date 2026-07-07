# Pit-diag study — what to stream on FDCAN2 that `/debug` does NOT carry

> Goal: a CAN-only observability stream (modelled on the ECU `0x700-0x707`/`0x7E0`
> and AMS `0x6C0-0x6C9` pit-diag) so the bench can read internal uDV state with a
> plain CAN tool — no ROS/USB/DVPC/agent. This study inventories every
> diagnostically-useful datum and marks what is **already** visible vs. what is
> **missing** and worth adding.

## 0. Baseline — what is already observable

**`/debug` (ROS `std_msgs/String`, ~2 Hz + on change)** — the primary on-car line:
`AS <state> || ASMS TS SDC EBS ABS || brakes mission R2D motion finished || RES:<name> || EBSinit:<name>`
→ 12 derived booleans + the RES status *name* + the EBS-init state *name*.

**Other ROS topics** (also USB-only, invisible on a bare CAN bench):
`/imu`, `/imu/status`, `/steering_angle`, `/steering/feedback[3]`, `/res/status`,
`/res/go`, `/ami/mission`, `/assi/state`, `/assi/pub_gap_max_ms`, `/as_state`,
`/motor_rpm`.

**The gap:** every line above needs the micro-ROS agent (DVPC/USB). On a bench
with just a CAN tool you see **nothing**. And even with ROS, several root-cause
values below are exposed **nowhere**.

---

## 1. 🔴 Would have short-cut *this week's* debugging (highest value)

These are the items whose absence directly cost bench time this session.

| Datum | Source | Exposed today | Why it matters |
|---|---|---|---|
| **Raw RES `0x191` `data[0]`** | ISR decode | ❌ nowhere | The RES→EMERGENCY hunt needed exactly this byte. `/debug` only shows the folded `RES:ESTOP` name; the raw byte tells polarity-bug vs. genuine e-stop at a glance. |
| **RES e-stop / GO / pre-alarm bits** (raw) | `g_res_estop`, `g_res_go_signal`, `g_res_pre_alarm` | partial (`/res/go`) | See the actual decoded bits, not just the folded status name. |
| **RES frame age (ms since last `0x191`)** | `g_res_last_rx_tick` | ❌ (folded into TIMEOUT/NONE) | "is it stale vs never vs live" as a number, not a 3-way name. |
| **fw build id / git short hash** | — (new) | ❌ nowhere | The stale-image confusion (pre-#92 DLC bug) would be a non-event: read the hash, know exactly what's flashed. |
| **free heap / min-ever heap** | `xPortGetFreeHeapSize` | ❌ nowhere | The 40 KB↔81 KB heap issue that risks micro-ROS init — invisible until it crashes. |
| **task-ran mask + stall/latch** | `safety_monitor` | ❌ nowhere | A dead/stalled task (IMU/CAN/APP) is currently only inferable by symptom. One byte shows liveness. |
| **reset cause (IWDG watchdog reset?)** | `RCC_FLAG_IWDG1RST` | ❌ nowhere | Distinguishes "booted normally" from "watchdog reset us" — a boot-loop tell. |
| **bench-stub mask (continuous)** | `BENCH_STUB_*` | boot banner only (once) | Which stubs are live, readable at any moment, not just at boot. |
| **steering motor state (`ESTADO_MOTOR`)** | `g_steer_motor_state` | ❌ nowhere | `OFF/ON/EMERGENCIA(-1)` — the input that trips `steer_emergency`→EMERGENCY. Invisible today. |

---

## 2. 🟠 State-machine inputs `/debug` folds or omits

`/debug` shows the *derived* booleans; these are the *raw* inputs behind them —
the difference between "TS:off" and "why is TS off".

| Datum | Source | Exposed | Note |
|---|---|---|---|
| Storage pressures A4/A5 (raw bar) | `hardware_io_read_actuator{1,2}_storage_pressure` | ❌ | `/debug` shows only `ABS:ok/fail`. Raw shows a marginal tank. |
| ASMS / TSMS raw (pre-boolean) | `hardware_io_read_asms_on/tsms_on` | derived only | a flaky switch reads here first. |
| EBS actuator pin readback | `hardware_io_is_ebs_active` | `EBS:on/off` | fine, keep. |
| `g_can_ts_active` (ECU `0x504`) | CAN | ❌ | the *CAN* TS vs the locally-sensed TS — divergence is a wiring tell. |
| `g_can_brake_over_limit` (ECU `0x505`) | CAN | folded into `brakes` | keep the raw verdict bit. |
| `g_can_sdc_res_open` | CAN | `SDC` | ⚠️ currently has **no source** (always false) — worth flagging in-frame. |
| `g_can_listen_go` (5 s READY gate) | app_task | ❌ | why GO isn't being accepted yet. |
| EBS-init **sub-state** + time-in-state | `EbsManager` | name only | which of the 9 init steps it is stuck on (`WaitTS`, `CheckActuator1`, …) + how long. |

---

## 3. 🟠 Pipeline / mission (invisible on a CAN bench)

| Datum | Source | Exposed | Note |
|---|---|---|---|
| `/dv/status` byte + age | `g_dv_status`, `g_dv_status_stamp_ms` | ROS-sub only | the pipeline heartbeat the FSM gates on — nowhere on CAN. |
| `/ctrl/cmd` accel / steer (latched) | `g_accel_cmd`, `g_steer_cmd` | ❌ | what the pipeline is *commanding* vs what we TX. |
| `/ctrl/cmd` age | `g_ctrl_cmd_stamp_ms` | ❌ | stale-command detection. |
| mission-setup handshake | `g_set_mission_in_progress`, `g_set_mission_ready`, `g_mission_going_cmd` | ❌ | where the mission start sequence is. |
| emergency/finished cmd flags | `g_emergency_cmd`, `g_finished_cmd` | ❌ | pipeline-raised abort/finish. |
| resolved mission id + `mission_valid` | app_task | `mission:set/unset` | the *code* (0-6) and whether it maps to a real mission. |
| IMU drop count | `g_imu_drop_count` | `/imu/status` | queue overflow / IMU health. |

---

## 4. 🟢 Steering / sensor detail (in `/steering/feedback`, not on CAN)

`g_steer_angle_actual/target/motor` (deg), LWS raw `g_steering_angle_raw` /
`g_steering_speed_raw` / `g_steering_status`, steering-feedback age
(`g_steering_last_rx_tick`), motor rpm (`g_can_motor_rpm`), IMU ax/ay/az/gyro.
Useful but lower priority for the "won't arm / emergency" class of bug.

---

## 5. Proposed frame grouping (FDCAN2, ~10 Hz, arm-gated `0x7DE` DEADBEEF)

IDs picked to avoid the ECU (`0x700-0x707`,`0x7E0`) and AMS (`0x6C0-0x6C9`)
ranges on the shared ACU bus. Low IDs = low arbitration priority (never preempts
safety frames).

| ID | Frame | Payload (draft) |
|---|---|---|
| `0x7A0` | **status** | AS state, signals bitmask (the 10 `/debug` booleans), mission id, `mission_valid`, EBS-init sub-state, stub mask |
| `0x7A1` | **RES + inputs** | raw `0x191 data[0]`, RES status code, estop/go/pre-alarm bits, radio quality, RES age-ms, brake-over-limit, `listen_go` |
| `0x7A2` | **pipeline** | `/dv/status` byte + age, accel_cmd, steer_cmd, ctrl age, setup-handshake bits |
| `0x7A3` | **health** | free heap, min heap, task-ran/stall mask, reset cause, uptime s, ASSI mode, safety-latched |
| `0x7A4` | **fw-info** | git short hash (build id), stub mask, heap size — emitted slowly (~1 Hz) |
| `0x7DE` (RX) | **arm** | magic `0xDEADBEEF` → sticky-enable the stream |

Notes:
- `initEcu` RX filter is `0x504-0x511` only → widen (or add a filter) to accept `0x7DE`.
- Optional **stream-always** compile toggle (bench) so no arm frame is needed;
  car default stays gated to keep the ACU bus quiet.
- Frame builders stay pure (state → bytes), paced from `can_task`, TX on `hfdcan2`
  — same shape as `sendDataLogger()`.

## 6. Recommendation (what to actually send)
Start with **§1 (all of it)** + **§2's raw pressures / EBS sub-state** + the
pipeline `/dv/status`+ages from §3. That set turns every hard-to-see failure
from this session (RES polarity, stale image, heap, dead task, steering
EMERGENCIA, "why not READY") into a byte you can read off the bus. §4 steering
detail and the rest of §3 can follow once the core frames prove out.
