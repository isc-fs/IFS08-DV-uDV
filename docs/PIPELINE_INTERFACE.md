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
| DVPC→uDV | `force_ebs` | `std_srvs/SetBool` | service | serve |

Byte values: AS `OFF=0 EMERGENCY=1 READY=2 DRIVING=3 FINISHED=4`;
DV `IDLE=0 PREPARING=1 READY=2 RUNNING=3 FINISHED=4 EMERGENCY=5 FAILED=6`.

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
5. `dv/status = FINISHED` → AS Finished (latches until ASMS off).
   `dv/status = EMERGENCY`/`FAILED`, RES e-stop, TS loss, or a **stale
   `dv/status` mid-run** → AS Emergency + EBS.

Liveness: `dv/status` older than `DV_STATUS_STALE_MS` (1 s) while driving is
treated as a dead pipeline → safe state. The pipeline's matching window on
`assi/state` is 1.5 s, so both sides independently agree who died.

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

- **`libmicroros.a` rebuild (`MAX_SERVICES`).** The firmware creates **two**
  service servers (`activate_steering`, `force_ebs`) but the committed lib
  was built with `RMW_UXRCE_MAX_SERVICES=1`, so `force_ebs` (the 2nd) may be
  **silently failing to register** on the currently-linked library — i.e.
  the pipeline's emergency-brake call would do nothing. `colcon.meta` is now
  bumped to `SERVICES=2`, `SUBSCRIPTIONS=6`; **rebuild the static library**
  (see CLAUDE.md → "Rebuild micro-ROS static library") and reflash. The two
  new *subscriptions* fit the existing limit (5) and need no rebuild.
- **Actuation scaling `[G2/G3, SAFETY]`.** `ctrl/cmd` is normalised and
  clamped on receive, then sent via `Can::sendAccel/sendSteer` as-is. Confirm
  the CONTROL_ACCEL / CONTROL_STEER CAN frames, units, sign and full-lock
  scaling against the vehicle DBC; clamp steering under STEERING's 70° cutoff.
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
   transition fires on the raw RES go bit (`can_c_get_res_status()==2`),
   which is **not** gated by the firmware's 5 s Ready timer. `g_can_listen_go`
   (set true after 5 s in Ready, [app_task.cpp:363](../Core/Src/app_task.cpp))
   only gates the separate `0x509` R2D frame
   ([can_interface.cpp:241](../Core/Src/can_interface.cpp)). If the rules
   require ≥5 s in AS Ready before Driving and the RES does not enforce it
   itself, add `&& g_can_listen_go.load()` to the Driving transition.
   *Fix is one clause; my `dv_ready` gate already sits alongside it.*

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

4. **`/force_ebs` service registration (safety-critical).** Covered above:
   until `libmicroros.a` is rebuilt with `MAX_SERVICES=2`, the pipeline's
   emergency-brake service may be unregistered — which would itself be a
   safety-rule failure. Close this before any track test.
