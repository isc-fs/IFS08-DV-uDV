# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Firmware for the **uDV** (micro Driverless Vehicle) board — an STM32H733XG (Cortex-M7, 528 MHz) embedded controller for a Formula Student Driverless car. It runs FreeRTOS + micro-ROS over USB CDC and interfaces with IMU, FDCAN buses, and WS2812 LEDs.

## Build Commands

### Make (primary)
```bash
make          # Build all (ELF, HEX, BIN) → build/binaries/uDV.elf
make clean    # Remove build artifacts
```

### CMake (IDE / clangd)
```bash
cmake --preset "Configure preset using toolchain file"   # Configure → build/binaries/
cmake --build build/binaries                             # Build
```

CMake generates `build/binaries/compile_commands.json`, which clangd uses (see `.clangd`). The Make build is used for production; CMake is kept in sync for IDE support.

### Toolchain
Requires `arm-none-eabi-gcc`. The CMake toolchain ([cmake/arm-none-eabi.cmake](cmake/arm-none-eabi.cmake)) auto-detects it from common macOS install paths (`/Applications/ArmGNUToolchain/`, `/opt/homebrew/bin/`). Override with `GCC_PATH=<dir>` for Make or `-DTOOLCHAIN_PATH=<dir>` for CMake.

### Flash
Use STM32CubeProgrammer or OpenOCD with the built `build/binaries/uDV.hex` or `.bin`.

### Rebuild micro-ROS static library
Only needed when adding/changing ROS message types:
```bash
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -i -v $(pwd):/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```
Output lands in `micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/`.

### Run micro-ROS agent (host side, Ubuntu + ROS 2 Humble)
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
See [firststeps.md](firststeps.md) for full host setup.

## Architecture

### FreeRTOS Task Structure

All tasks and queues are **created** in [Core/Src/freertos.c](Core/Src/freertos.c) (`MX_FREERTOS_Init`), but each task **body** lives in its own file so the CubeMX-regenerated `freertos.c` stays thin. `StartDefaultTask` keeps only the CubeMX-managed `MX_USB_DEVICE_Init()` then calls `ros_task_run()`.

| Task | Priority | Stack | Role | Body |
|---|---|---|---|---|
| `defaultTask` | Normal | 12 KB | micro-ROS node: all publishers, subscriber executor, time sync | [ros_task.c](Core/Src/ros_task.c) (`ros_task_run`) |
| `imuTask` | AboveNormal | 8 KB | BMI088 read → attitude update → push to `imuQueueHandle` | [imu_task.c](Core/Src/imu_task.c) |
| `canTask` | AboveNormal | 4 KB | Drain `canRxQueueHandle` + `resRxQueueHandle`, data logger TX | [can_task.cpp](Core/Src/can_task.cpp) |
| `assiTask` | BelowNormal | 2 KB | ASSI LED renderer: AS mode → UART commands to the Arduino bridge | [assi_task.c](Core/Src/assi_task.c) |
| `safetyTask` | High | 2 KB | Two-tier watchdog / IWDG refresh | [safety_monitor.c](Core/Src/safety_monitor.c) |
| `appTask` | Normal | 4 KB | AS state machine + EBS init sequence | [app_task.cpp](Core/Src/app_task.cpp) |

The shared microsecond timestamp `dwt_micros()` (used by both the IMU and ROS tasks) lives in [dwt_time.c](Core/Src/dwt_time.c).

Inter-task communication uses FreeRTOS message queues:
- **`imuQueueHandle`** (depth 16): `imu_sample_t` structs from `imuTask` → `defaultTask`
- **`canRxQueueHandle`** (depth 32): `can_msg_t` from ISR → `canTask` (FDCAN3)
- **`resRxQueueHandle`** (depth 8): `can_msg_t` from ISR → `canTask` (FDCAN1)
- **`debugQueueHandle`** (depth 8, 128-byte strings): CAN service debug → `defaultTask` → `/debug` ROS topic
- **`imuSemHandle`**: TIM2 ISR → `imuTask` semaphore for 400 Hz deterministic sampling

### IMU Pipeline
TIM2 fires at 400 Hz → `HAL_TIM_PeriodElapsedCallback` releases `imuSemHandle` → `imuTask` reads BMI088 via I2C2 with software bitbang recovery ([Core/Src/i2c_utils.c](Core/Src/i2c_utils.c)) → computes roll/pitch via CORDIC hardware ([Core/Src/attitude.c](Core/Src/attitude.c), complementary filter) → pushes `imu_sample_t` with DWT cycle-counter timestamp → `defaultTask` publishes to `/imu` (canonical on both sides — see the IMU topic note below).

Timestamps use DWT cycle counter (sub-microsecond) with NTP-like sync via `rmw_uros_sync_session`. Re-sync happens every ~10 s (4000 samples).

### CAN Bus Layout
- **FDCAN1**: RES CANopen (Node-ID `0x11`). Receives PDO at `0x191` (e-stop, go-signal, radio quality). Transmits NMT `0x000` and data logger frames `0x500/0x501/0x502`.
- **FDCAN2**: ACU bus (ECU/VCU + AMS) — the ECU↔uDV contract, matching `IFS08-CE-ECU`'s `.def` files. Receives `0x504` TS-active, `0x505` brake-over-limit verdict, `0x506` motor rpm, `0x511` DV R2D confirm. Transmits `0x507` torque (int32 percent, 20 ms), `0x510` DV R2D request, and `0x001` IMU broadcast (ax/ay/az int16 mg + gx int16 0.1 dps, 50 Hz from imu_task). RX FIFO0 is routed to interrupt LINE1 (only `FDCAN2_IT1` is NVIC-enabled).
- **FDCAN3**: AMI + steering bus (local DV peripherals). Receives mission select `0x503`, steering sensor `0x2B0`, steering feedback `0x500`. Transmits mission ACK `0x50A`, ASSI `0x100` (local bus only — the ECU→AMS heartbeat also uses 0x100 but on the ACU bus), steering motor `0x010` and angle `0x020` (inspection sweep AND pipeline `/ctrl/cmd` steering — the old consumer-less `0x508` is gone).

ISR (`HAL_FDCAN_RxFifo0Callback`) drains all pending frames into the correct queue via `can_isr_push_rx`. The `HAL_FDCAN_ErrorCallback` override re-enables RX notifications after overrun (HAL silently disables them on overrun).

### micro-ROS Node (`defaultTask`)
ROS 2 node `cubemx_node` exposes:

| Topic | Type | QoS | Hz |
|---|---|---|---|
| `/imu` ⚠️ | `sensor_msgs/Imu` | best-effort | 400 |
| `/imu/status` | `std_msgs/Int32` | reliable | ~0.1 |
| `/steering_angle` (rad) | `std_msgs/Float32` | best-effort | ~10 |
| `/motor_rpm` | `std_msgs/Float32` | best-effort | ~10 |
| `/steering/feedback` | `std_msgs/Float32MultiArray` | best-effort | 20 |
| `/res/status` | `std_msgs/Int32` | best-effort | ~10 |
| `/res/go` | `std_msgs/Int32` | best-effort | ~10 |
| `/ami/mission` | `std_msgs/Int32` | best-effort | ~10 |
| `/assi/state` (heartbeat) | `std_msgs/UInt8` | best-effort | 10 (wall-clock, IMU-decoupled) |
| `/assi/pub_gap_max_ms` | `std_msgs/Int32` | best-effort | 10 |
| `/as_state` | `std_msgs/UInt8` | best-effort | ~10 |
| `/debug` | `std_msgs/String` | reliable | ~10 |
| `/dv/status` (sub) | `std_msgs/UInt8` | reliable | — |
| `/ctrl/cmd` (sub) | `geometry_msgs/Twist` | best-effort | — |
| `/cmd_test` (sub) | `std_msgs/Int32` | reliable | — |
| `/activate_steering`, `/force_ebs` (srv) | `std_srvs/SetBool` | — | — |

**IMU topic**: canonical `/imu` on both sides. The firmware publishes `/imu`
and the pipeline's car profile subscribes odometry/slam to `/imu` directly
(no remap). Standardised on `/imu` 2026-07-04 (the pipeline dropped its old
`/imu/data_raw` car remap).

Transport: USB CDC with HDLC framing via `micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c`. Memory: custom FreeRTOS-heap allocators in `microros_allocators.c`.

### ASSI LEDs (UART → Arduino bridge)
The STM32 does not drive the WS2812 strip directly (3.3 V-vs-5 V DIN level problem). [ws2812.c](Core/Src/ws2812.c) sends 2-byte commands (`a`=off, `b`=yellow, `c`=blue, newline-framed) over **USART10** (PG12 TX, 115200 8N1) to an Arduino Nano that drives the strip at 5 V. [assi_task.c](Core/Src/assi_task.c) maps the AS mode set by `appTask` (`assi_set_mode`) onto colours and owns the flash timing (150 ms half-period → 3.3 Hz, 50 % duty, T14.9.1). See [docs/ASSI_UART_BRIDGE.md](docs/ASSI_UART_BRIDGE.md).

## STM32CubeMX Integration

`uDV.ioc` is the CubeMX project file. CubeMX generates peripheral init code into `Core/Src/` and `Core/Inc/`. **Only edit code inside `/* USER CODE BEGIN/END */` blocks** to survive regeneration. USART10 (ASSI Arduino bridge, PG11 RX / PG12 TX, AF11) is a fully CubeMX-managed peripheral: enabled in `uDV.ioc`, with `MX_USART10_UART_Init`/`HAL_UART_MspInit` generated into [Core/Src/usart.c](Core/Src/usart.c). It must stay in the `.ioc` — if it is removed there, regeneration drops `HAL_UART_MODULE_ENABLED` from `stm32h7xx_hal_conf.h` and the build breaks. (SPI1 was fully removed with the old direct WS2812 driver.) **The clock tree in `uDV.ioc` is 528 MHz (VOS0, PLLN=44, HPRE=DIV2)** — do not import a different team `.ioc`'s clock config without regenerating `tim.c`/`fdcan.c`/`i2c.c`/`adc.c` consistently; TIM2's 400 Hz tick and the FDCAN bit timing depend on it.

## Key Constraints

- **RAM_D1 BSS**: The startup script only zeros DTCMRAM `.bss`. `main.c` manually zeros the `RAM_D1` BSS section at startup before peripheral init.
- **FreeRTOS task stack**: `defaultTask` needs ≥ 3000 words (12 KB) for micro-ROS. Reducing it causes stack overflow during `rclc_support_init`.
- **micro-ROS static library**: Pre-built Cortex-M7 hard-float. Do not change compiler flags (MCU/FPU) without rebuilding it with Docker.
- **`STM32_THREAD_SAFE_STRATEGY=4`**: Required define for thread-safe newlib with FreeRTOS; implemented in `newlib_lock_glue.c`.
