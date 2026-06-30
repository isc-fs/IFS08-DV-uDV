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

All tasks and queues are created in [Core/Src/freertos.c](Core/Src/freertos.c):

| Task | Priority | Stack | Role |
|---|---|---|---|
| `defaultTask` | Normal | 12 KB | micro-ROS node: all publishers, subscriber executor, time sync |
| `imuTask` | AboveNormal | 8 KB | BMI088 read → attitude update → push to `imuQueueHandle` |
| `canTask` | AboveNormal | 4 KB | Drain `canRxQueueHandle` + `resRxQueueHandle`, data logger TX |
| `amiTask` | BelowNormal | 2 KB | WS2812 LED color based on mission index |

Inter-task communication uses FreeRTOS message queues:
- **`imuQueueHandle`** (depth 16): `imu_sample_t` structs from `imuTask` → `defaultTask`
- **`canRxQueueHandle`** (depth 32): `can_msg_t` from ISR → `canTask` (FDCAN3)
- **`resRxQueueHandle`** (depth 8): `can_msg_t` from ISR → `canTask` (FDCAN1)
- **`debugQueueHandle`** (depth 8, 128-byte strings): CAN service debug → `defaultTask` → `/debug` ROS topic
- **`imuSemHandle`**: TIM2 ISR → `imuTask` semaphore for 400 Hz deterministic sampling

### IMU Pipeline
TIM2 fires at 400 Hz → `HAL_TIM_PeriodElapsedCallback` releases `imuSemHandle` → `imuTask` reads BMI088 via I2C2 with software bitbang recovery ([Core/Src/i2c_utils.c](Core/Src/i2c_utils.c)) → computes roll/pitch via CORDIC hardware ([Core/Src/attitude.c](Core/Src/attitude.c), complementary filter) → pushes `imu_sample_t` with DWT cycle-counter timestamp → `defaultTask` publishes to `/imu/data_raw`.

Timestamps use DWT cycle counter (sub-microsecond) with NTP-like sync via `rmw_uros_sync_session`. Re-sync happens every ~10 s (4000 samples).

### CAN Bus Layout
- **FDCAN1**: RES CANopen (Node-ID `0x11`). Receives PDO at `0x191` (e-stop, go-signal, radio quality). Transmits NMT `0x000` and data logger frames `0x500/0x501/0x502`.
- **FDCAN3**: AMI + steering bus. Receives mission select `0x503` and steering sensor `0x2B0`. Transmits steering motor `0x010` and angle `0x020`.

ISR (`HAL_FDCAN_RxFifo0Callback`) drains all pending frames into the correct queue via `can_isr_push_rx`. The `HAL_FDCAN_ErrorCallback` override re-enables RX notifications after overrun (HAL silently disables them on overrun).

### micro-ROS Node (`defaultTask`)
ROS 2 node `cubemx_node` exposes:

| Topic | Type | QoS | Hz |
|---|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | best-effort | 400 |
| `/imu/status` | `std_msgs/Int32` | reliable | ~0.1 |
| `/steering/data` | `std_msgs/Float32` | best-effort | ~10 |
| `/res/status` | `std_msgs/Int32` | best-effort | ~10 |
| `/ami/mission` | `std_msgs/Int32` | best-effort | ~10 |
| `/debug` | `std_msgs/String` | reliable | ~10 |
| `/cmd_test` (sub) | `std_msgs/Int32` | reliable | — |

Transport: USB CDC with HDLC framing via `micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c`. Memory: custom FreeRTOS-heap allocators in `microros_allocators.c`.

### WS2812 LED Driver
8 LEDs driven via SPI1 MOSI (bit-banged encoding: `0xE0`=high bit, `0x80`=low bit at ~2 MHz). `ws2812_set_mission_color(index)` maps mission index 0–9 to colors. Updated by `amiTask` on mission changes.

## STM32CubeMX Integration

`uDV.ioc` is the CubeMX project file. CubeMX generates peripheral init code into `Core/Src/` and `Core/Inc/`. **Only edit code inside `/* USER CODE BEGIN/END */` blocks** to survive regeneration. SPI1 (WS2812 driver, PA7 = `SPI1_MOSI`, transmit-only master) is a fully CubeMX-managed peripheral: enabled in `uDV.ioc`, with `MX_SPI1_Init`/`HAL_SPI_MspInit` generated into [Core/Src/spi.c](Core/Src/spi.c). It must stay in the `.ioc` — if SPI1 is removed there, regeneration drops `HAL_SPI_MODULE_ENABLED` from `stm32h7xx_hal_conf.h` and the build breaks with `unknown type name 'SPI_HandleTypeDef'`.

## Key Constraints

- **RAM_D1 BSS**: The startup script only zeros DTCMRAM `.bss`. `main.c` manually zeros the `RAM_D1` BSS section at startup before peripheral init.
- **FreeRTOS task stack**: `defaultTask` needs ≥ 3000 words (12 KB) for micro-ROS. Reducing it causes stack overflow during `rclc_support_init`.
- **micro-ROS static library**: Pre-built Cortex-M7 hard-float. Do not change compiler flags (MCU/FPU) without rebuilding it with Docker.
- **`STM32_THREAD_SAFE_STRATEGY=4`**: Required define for thread-safe newlib with FreeRTOS; implemented in `newlib_lock_glue.c`.
