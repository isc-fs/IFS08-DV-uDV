# `prerun/` — rosbag replay ON the car (uDV side)

> ⚠️ **DO NOT DELETE this branch or this file.** `prerun/*` is a standing
> configuration branch, not a feature branch. It is never merged to `dev` and
> never deleted. If GitHub branch protection is set, deletion/force-push are
> blocked; keep it that way.

## What this branch is for

`prerun/` lets us **replay a recorded rosbag on the real car** (wheels off the
ground, on stands) to exercise the full autonomy + actuation stack **before**
putting the car on the ground. The IMU and LiDAR can't produce meaningful live
data in that state, so those two feeds come from the bag; **everything else
stays live** (EBS, steering, motor torque, RES, the DV handshake, ASSI).

This is the **uDV half**. The pipeline half lives on
`IFS08-DV-PIPELINE @ prerun/rosbag-onboard` (which launches with the Hesai
LiDAR driver disabled so the bag's `/lidar_points` is the only source).

## The one change vs `dev`

`Core/Inc/bench_stubs.h` sets **`#define BENCH_STUB_IMU_ROS 1`** (above the
guarded defaults). That is the entire functional divergence. The stub *hook*
itself (default 0) lives on `dev`, so this branch is a ~2-line divergence
(this `#define` + this file) that is trivial to keep fresh.

`BENCH_STUB_IMU_ROS=1` suppresses **only** the ROS `/imu` publish
(`ros_task.c`). It does **not** touch:

- IMU sampling (`imuTask`) or the complementary-filter attitude,
- the `0x512` CAN IMU broadcast to the ECU,
- the `/imu` publisher *entity* (still created → micro-ROS entity counts
  unchanged → **no `libmicroros.a` rebuild needed**).

All other bench stubs stay **0** (flight-clean): real EBS, real steering, real
DVPC handshake, real RES.

## Build & flash

```bash
git checkout prerun/rosbag-onboard
make            # BENCH_STUB_IMU_ROS is already 1 on this branch
# flash build/binaries/uDV.hex (STM32CubeProgrammer / OpenOCD)
```

At boot the firmware announces on `/debug`:
`BENCH STUBS COMPILED IN (... imu_ros=1)` — so a prerun image can never
masquerade as a flight build. The pit-diag stub mask carries bit `0x20`.

## Verify (bench, with the micro-ROS agent up)

- `ros2 topic hz /imu` → **no** firmware publisher (silent) until the bag plays.
- `/motor_rpm`, `/steering_angle`, `/assi/state` still publish normally.
- Play the bag → `ros2 topic info /imu` shows **exactly one** publisher (the
  replay). More than one = a collision / misconfig.

## Rosbag requirements (shared with the pipeline branch)

- The bag must contain **only** the replayed sensor feeds: `/imu` (here) and
  `/lidar_points` (pipeline side). If it also carries other uDV topics
  (`/motor_rpm`, `/steering_angle`, `/res/*`, `/assi/state`, …) they will
  collide with the live firmware — add stubs or strip those topics first.
- Replay is **re-stamped to "now"** (see the pipeline branch's re-stamp relay),
  because the live DV handshake and the 400 ms staleness watchdogs run on
  current time; old bag stamps would trip them.
- `/imu` frame_id must be `imu_link` (matches this firmware and the pipeline's
  static TF).

## Keeping this branch fresh

After the `BENCH_STUB_IMU_ROS` hook PR merges to `dev`, rebase this branch onto
`dev` periodically so it stays current. The rebase should collapse to exactly
this `#define` + this file. **Never** merge this branch into `dev`.
