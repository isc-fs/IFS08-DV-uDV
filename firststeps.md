# First Steps

End-to-end clone → flash → verify, in under five minutes on a clean
machine.

## Prerequisites

- **ARM GCC toolchain** (`arm-none-eabi-gcc`), tested with the
  [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  14.3.Rel1
- **ROS 2 Humble** on the host PC that will run the micro-ROS agent
- A unix-like shell on Linux or macOS (Windows works too via WSL or
  MSYS2/MinGW — the Makefile auto-detects)

> Note: you do **not** need Docker for a normal clone-and-build flow.
> The submodule lives on the org-owned fork
> [`isc-fs/micro_ros_stm32cubemx_utils`](https://github.com/isc-fs/micro_ros_stm32cubemx_utils)
> on the `humble-isc` branch, which ships the prebuilt `libmicroros.a`
> + headers ready to link.  Docker is only needed if you change the
> custom ROS interface package and need to rebuild libmicroros — see
> [Rebuilding libmicroros](#rebuilding-libmicroros) below.

## 1. Clone

```bash
git clone --recurse-submodules -b dev git@github.com:isc-fs/IFS08-DV-uDV.git
cd IFS08-DV-uDV
```

`--recurse-submodules` is mandatory.  Without it the
`micro_ros_stm32cubemx_utils/` directory will be empty and the build
will fail with `fatal error: rcl/rcl.h: No such file or directory`.
If you forgot, run `git submodule update --init --recursive`.

## 2. Build the firmware

```bash
make -j
```

If `arm-none-eabi-gcc` is on your `PATH`, that's all.  If it lives in a
non-standard location:

```bash
make GCC_PATH=/path/to/arm-none-eabi/bin -j
```

Outputs land in `build/binaries/`:

- `uDV.elf` — debug-friendly format (use with `gdb`, `arm-none-eabi-size`)
- `uDV.bin` — raw binary for ST-Link / DFU
- `uDV.hex` — Intel HEX for STM32CubeProgrammer

> **HIL bench note:** per the team's flashing convention, use
> STM32CubeProgrammer for ST-Link flashing.  Do **not** use MingoCAN
> ST-Link BL flashing — it produces a corrupt bootloader.

## 3. Flash and run

Flash `build/binaries/uDV.bin` (ST-Link) or `uDV.hex`
(STM32CubeProgrammer / DFU) to the STM32H733XG, then start the
micro-ROS agent on the host PC:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

On macOS the device path is `/dev/cu.usbmodem*` instead.

Keep the board stationary for ~10 s (gyro bias calibration), then
verify the IMU stream:

```bash
ros2 topic hz /imu/data_raw
# expected: average rate ~400 Hz
```

If you don't see the topic, see
[Troubleshooting](#troubleshooting) below.

## Rebuilding libmicroros

You only need this if you add or modify a custom ROS interface
package.  The prebuilt artifact baked into the submodule on
`humble-isc` already includes whatever schema is currently being
used; for most clone-and-flash work you can skip this section.

To rebuild from a local package source (the folder needs to live
under `micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/`):

```bash
docker pull microros/micro_ros_static_library_builder:humble
echo "y" | docker run --rm -i -v $(pwd):/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```

After the rebuild succeeds, the new `libmicroros.a` and `microros_include/`
overwrite the prebuilt ones in the submodule.  Commit those changes
inside the submodule, push to `humble-isc` on
[`isc-fs/micro_ros_stm32cubemx_utils`](https://github.com/isc-fs/micro_ros_stm32cubemx_utils),
and bump the parent's submodule pin.

## Troubleshooting

- **`fatal error: rcl/rcl.h: No such file or directory`**
  — you cloned without `--recurse-submodules`.  Run
  `git submodule update --init --recursive`.

- **`/bin/sh: -c: line 1: syntax error: unexpected end of file`**
  on the first `mkdir` line of the Makefile
  — your branch is missing the portable mkdir fix.  Rebase onto the
  latest `dev`.

- **Linker errors about `usleep` / `_gettimeofday` undefined**
  — your submodule pin is behind `humble-isc`.  Run
  `git submodule update --remote micro_ros_stm32cubemx_utils` and
  commit the bump.

- **No ROS topics published despite agent connecting**
  — the gyro calibration runs for the first 6 s of operation; the
  IMU task only starts publishing afterwards.  Wait, or check
  `/imu/status` for the BMI088 init code (-99 = pre-init, 0 = OK).

## Related docs

- [`changelog.md`](changelog.md) — full architecture documentation
- [`testing_guide_v0.1.md`](testing_guide_v0.1.md) — v0.1 release
  validation procedure
