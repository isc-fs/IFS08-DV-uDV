# uDV ↔ DV-pipeline interface (stock-typed)

Firmware side of the topic-based contract introduced in
`IFS08-DV-PIPELINE @ feat/7-udv-stock-interface`. The canonical source of
truth is the pipeline's `mission_control/interface_contract.py`; the
firmware mirror is [`Core/Inc/dv_interface.h`](../Core/Inc/dv_interface.h).

Nobody "calls" anybody. Both sides continuously broadcast a status byte and
react to the other's — the two old micro-ROS actions (`SetMission`,
`RuntimeControl`, never actually integrated on this firmware) are gone.

## Wire surface

| Dir | Topic / service | Type | QoS | uDV role |
|---|---|---|---|---|
| uDV→DVPC | `assi/state` | `std_msgs/UInt8` | best-effort, ~10 Hz | publish (heartbeat) |
| uDV→DVPC | `ami/mission` | `std_msgs/Int32` | best-effort, ~10 Hz | publish (raw AMI index) |
| DVPC→uDV | `dv/status` | `std_msgs/UInt8` | reliable (matches latched pub) | **subscribe** |
| DVPC→uDV | `ctrl/cmd` | `geometry_msgs/Twist` | **best-effort (required)** | **subscribe** |
| DVPC→uDV | `force_ebs` | `std_srvs/SetBool` | service | serve (emergency brake) |
| DVPC→uDV | `activate_steering` | `std_srvs/SetBool` | service | serve (enable steering motor) |

Byte values: AS `OFF=0 EMERGENCY=1 READY=2 DRIVING=3 FINISHED=4`;
DV `IDLE=0 PREPARING=1 READY=2 RUNNING=3 FINISHED=4 EMERGENCY=5 FAILED=6 STOPPING=7`.

Unknown / unhandled `dv/status` bytes are ignored by design: the firmware only
compares the byte for equality against the values it acts on, so an unrecognised
one changes no actuation and no AS state — it merely keeps the heartbeat fresh.
`RUNNING=3` has always relied on this (nothing matches it explicitly). That is
what lets either side add a byte without a lockstep flash.

`ctrl/cmd`: `linear.x` = throttle [-1,1], `angular.z` = steering [-1,1].

## Handshake (implemented in `app_task.cpp`)

1. AMI select → firmware publishes `ami/mission` (raw CAN `0x503` index).
2. Arm (ASMS + TS + RES ok) → `assi/state = READY`. DVPC configures,
   answers `dv/status = PREPARING → READY`.
3. **RES "go" is honoured only while `dv/status == READY` and fresh** →
   `assi/state = DRIVING`. This gate replaces waiting on the old
   `SetMission` result. (The pre-existing 5 s Ready dwell still applies.)
4. DVPC activates → `dv/status = RUNNING` and streams `ctrl/cmd`. The
   firmware actuates it **only while AS Driving** and zeroes it if the
   stream goes stale (`DV_CTRL_CMD_STALE_MS`).
5. *(optional)* `dv/status = STOPPING` → the **end-of-mission stop** (#176):
   the uDV fires the EBS and zeroes the torque **with the SDC left closed and
   the AS state left in Driving**. It is not an emergency — it exists only so
   the car can reach the standstill AS Finished requires. The pipeline follows
   with `FINISHED` once the car has actually stopped. Gated on a pipeline
   mission; `BENCH_STUB_DV_STOPPING=1` makes it a no-op for bench work.
   **This is not a service brake** — the EBS is binary, so it must only be used
   for the final stop, never to modulate speed during a run.
   **Debounced + latched:** the stop arms only after **`DV_STOPPING_MIN_STREAK`
   (3) consecutive** STOPPING messages (~200–300 ms at 10 Hz), so a single
   spurious byte can't stab the brakes; and once armed it is **sticky for the
   rest of the run** — reverting to `RUNNING` (byte 3) will NOT release it. To
   end the stop, send `FINISHED` (→ AS Finished) or let the link go stale
   (→ Emergency). The latch clears when the run leaves Driving; the next run
   must earn its own fresh streak.
6. `dv/status = FINISHED` → AS Finished (latches until ASMS off).
   `dv/status = EMERGENCY`/`FAILED`, RES e-stop, TS loss, or a **stale
   `dv/status` mid-run** → AS Emergency + EBS.

Liveness: `dv/status` older than `DV_STATUS_STALE_MS` (400 ms) while driving
is treated as a dead pipeline → safe state. The pipeline watches `assi/state`
on the same 400 ms window (`_ASSI_STALE_S` in `mission_control_node`). Both
sides publish at 10 Hz and time out at 4 missed cycles, so on a link loss each
independently drops to a safe state — and within the FS-Rules T11.9.4 500 ms
cap for detecting a lost safety-critical message.

## Implementation map

- [`Core/Inc/dv_interface.h`](../Core/Inc/dv_interface.h) — contract bytes,
  topic names, timings, C bridge decls.
- [`Core/Src/ros_task.c`](../Core/Src/ros_task.c) — `dv/status` +
  `ctrl/cmd` subscribers (note the best-effort QoS on `ctrl/cmd`),
  their callbacks, executor grown to 5 handles.
- [`Core/Src/ros_globals.cpp`](../Core/Src/ros_globals.cpp) — the C bridge
  (`ros_set_ctrl_cmd_norm`, `ros_set_dv_status`) + new atomics.
- [`Core/Src/app_task.cpp`](../Core/Src/app_task.cpp) — the READY→DRIVING
  gate, FINISHED/EMERGENCY transitions, `ctrl/cmd` actuation + staleness.

## ⚠️ Open items before an on-track run

- **IMU topic — RESOLVED (standardised on `/imu`, 2026-07-04).** The
  firmware publishes the IMU on **`imu`** (→ `/imu`, empty namespace), and
  the pipeline's car profile now subscribes `odometry_filter_node` /
  `slam_node` to **`/imu`** directly (the old `REMAP_IMU_CAR = /imu →
  /imu/data_raw` car remap was dropped in `bringup/topic_contract.py`). Both
  sides are canonical `/imu`, so the EKF gets IMU on the car. Bench-verify
  with `ros2 topic hz /imu` (expect ~400 Hz) once flashed.
- **`libmicroros.a` rebuild (`MAX_PUBLISHERS` / `MAX_SERVICES`).** The
  firmware creates **two** service servers (`activate_steering`, `force_ebs`)
  and **12** publishers, but the committed lib was built with
  `RMW_UXRCE_MAX_SERVICES=1` and `RMW_UXRCE_MAX_PUBLISHERS=10` — so the 2nd
  service (`force_ebs`) and the 11th/12th publishers **silently fail to
  register** on the currently-linked library. This is a *correctness* gap,
  **not a braking one** — `force_ebs` is a redundant/bench actuator hook; the
  emergency brake is driven by `dv/status = EMERGENCY` → AS Emergency →
  `as_actuation()`, a subscriber path unaffected by the service cap.
  `colcon.meta` is now bumped to `SERVICES=2` and
  `PUBLISHERS=20` (headroom for future publishers); **rebuild the static
  library** (see CLAUDE.md → "Rebuild micro-ROS static library") and reflash
  for it to take effect. `SUBSCRIPTIONS=5` is unchanged — the 3 subscribers
  fit the existing limit.
- **Actuation scaling `[G2/G3, SAFETY]`.** `ctrl/cmd` is normalised and
  clamped on receive. Throttle: `Can::sendAccel` → ECU `0x507` (int32 LE
  percent, confirmed against the ECU `.def`). Steering: routed to the
  steering controller on `0x521` as `norm × STEER_FULL_LOCK_DEG` (65°,
  under STEERING's 70° cutoff; the old consumer-less `0x508` is gone).
  Still to commission (#71): real full-lock angle, steering ratio, and the
  sign convention (ROS `+z` = CCW/left) on the car.
- **AMI index map `[G5]`.** `ami/mission` forwards the raw CAN `0x503` byte.
  Confirm the AMI board's index encoding matches the pipeline's
  `DEFAULT_AMI_TO_MISSION_ID` (4 = Track drive). (AMI board is a separate repo.)
- **Command latency.** `ctrl/cmd` callbacks are serviced when the executor
  spins (~10 Hz, in the slow-publish block of `ros_task.c`). Actuation to CAN
  runs every app tick from the latched value, but the *command update* is
  ~10 Hz — revisit the spin cadence if steering feels laggy at speed.
- **Sensor plane (`imu`/`motor_rpm`/`steering_angle`).** Unchanged by this
  work; see the pipeline's `docs/CAR_ADAPTATION.md` gaps G1/G3/G6.

## ⚠️ Rules-compliance items to verify (pre-existing — NOT introduced here)

These predate the pipeline-interface work (the AS transitions in
`app_task.cpp` were not rewritten, only gated) but must be checked against
the current season's FS Driverless rules before a track run. Listed here so
they aren't lost.

1. **5 s Ready dwell vs. the Driving transition.** The READY→DRIVING
   transition fires on the raw RES go bit (`in.res_go`, from
   `can_c_get_res_status()==2`; [as_transition.hpp:102](../Core/Inc/as_transition.hpp)),
   which is **not** gated by the firmware's 5 s Ready timer. `g_can_listen_go`
   is still set true after 5 s in Ready
   ([app_task.cpp:584](../Core/Src/app_task.cpp)) but is now **write-only**
   for gating — only pit-diag reads it — so the 5 s dwell currently gates
   nothing (the old `0x509` R2D frame it once guarded is gone; the DV R2D
   request is now the `0x510` `sendR2dRequest`, gated on the ECU's `0x511`
   confirm via `!g_can_r2d`). If the rules require ≥5 s in AS Ready before
   Driving and the RES does not enforce it itself, add
   `&& g_can_listen_go.load()` to the Driving transition.
   *Fix is one clause; the `dv_ready` gate already sits alongside it.*

2. **Mission-selected precondition for AS Ready.** The chain enters READY on
   `res_ok && ts_on` without explicitly checking a mission is selected; today
   this is masked by the default-to-Inspection fallback
   (`current_mission_id = 6`, [app_task.cpp](../Core/Src/app_task.cpp)).
   Confirm "no mission ⇒ no Ready" is the intended rule, or make it explicit.

3. **AS Finished braking.** The FINISHED case zeroes throttle/steering but
   does not itself command a brake-to-standstill — it relies on the pipeline
   having already stopped the car before it publishes `dv/status=FINISHED`.
   Confirm a controlled stop in AS Finished meets the rules (service brake vs.
   EBS-hold), or add an explicit brake in the FINISHED case.

4. **`/force_ebs` service registration (correctness, not safety-critical).**
   Until `libmicroros.a` is rebuilt with `MAX_SERVICES=2` the service is
   unregistered — but emergency **braking is unaffected**: the pipeline also
   drives `dv/status = EMERGENCY`, which latches AS Emergency + EBS via
   `as_actuation()` (a subscriber, not the service). `force_ebs` is a
   redundant/bench actuator path — its callback does a bare GPIO write that the
   ~1 ms `as_actuation()` loop overwrites, so it does **not** latch. Rebuild to
   restore the redundant channel (and `activate_steering` + the `/as_state`
   publisher), but it is not a braking dependency.
