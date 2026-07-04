# Changelog

## Architecture

### FreeRTOS Tasks

| Task | Priority | Stack | Function |
|---|---|---|---|
| defaultTask | Normal | 12 KB | USB CDC init, micro-ROS node, IMU + debug publishers |
| imuTask | AboveNormal | 8 KB | BMI088 I2C read at 400Hz (TIM2 + semaphore driven) |
| canTask | AboveNormal | 4 KB | FDCAN3 CAN RX message processing via queue |
| amiTask | BelowNormal | 2 KB | WS2812 mission LED updates via task notifications |

### Data Flow

```
[TIM2 ISR 400Hz] --sem--> [imuTask] --queue(16)--> [defaultTask] --> /imu
                           BMI088 I2C               micro-ROS pub    sensor_msgs/Imu

[FDCAN3 ISR] --queue(32)--> [canTask] --g_mission_index + notify--> [amiTask]
                             dispatch                                WS2812 LEDs
```

### Memory Layout

| Region | Size | Usage | Contents |
|---|---|---|---|
| FLASH | 1024 KB | ~130 KB (13%) | Code + constants |
| DTCMRAM | 128 KB | ~77 KB (60%) | .bss, FreeRTOS heap (40KB), stacks |
| RAM_D1 | 320 KB | ~40 KB (13%) | micro-ROS custom memory manager heap |

## micro-ROS Agent Setup

### Prerequisites

- Ubuntu 22.04 (Jammy) or compatible Linux distribution
- ROS 2 Humble installed and sourced
- USB cable connected to the STM32 board

### 1. Create the micro-ROS Agent Workspace

```bash
mkdir -p microros_agent_ws/src
cd microros_agent_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs
cd ..
```

### 2. Build the Agent

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Connect the STM32 Board

Plug in the STM32 via USB. Verify the CDC device appears:

```bash
ls /dev/ttyACM*
```

If you get permission errors, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

### 4. Run the Agent

```bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

On macOS the device path is `/dev/cu.usbmodem*`.

## Topics

| Topic | Type | QoS | Rate | Description |
|---|---|---|---|---|
| `/imu` | `sensor_msgs/msg/Imu` | Best effort | 400 Hz | BMI088 accelerometer and gyroscope data |
| `/imu/status` | `std_msgs/msg/Int32` | Reliable | ~1 Hz | IMU driver status code |

### IMU Data

```bash
ros2 topic echo /imu --qos-profile sensor_data
ros2 topic hz /imu
```

- `linear_acceleration`: m/s^2 (converted from BMI088 g output)
- `angular_velocity`: rad/s (converted from BMI088 dps output)
- `orientation`: not populated (covariance[0] = -1)

### IMU Status Codes

| Value | Meaning |
|---|---|
| `0` | BMI088_OK |
| `-1` | BMI088_ERR_PARAM |
| `-2` | BMI088_ERR_I2C |
| `-3` | BMI088_ERR_ID |
| `-99` | imuTask not started (insufficient FreeRTOS heap) |

## Timestamps

Messages carry epoch-synchronized timestamps with sub-microsecond precision:
- Clock synchronized with the agent via NTP-like `rmw_uros_sync_session` protocol
- Timestamps captured at the exact TIM2 interrupt moment using the DWT cycle counter (528 MHz)
- Re-synced every ~10 seconds to correct crystal drift
- Deterministic 2.5ms intervals between samples for SLAM pre-integration
- DWT overflow handled with 64-bit extension (safe beyond 8.13s wrap period)

## Startup Sequence

After flashing and connecting, the board goes through:
1. **USB CDC enumeration** (~2 seconds)
2. **micro-ROS agent connection** (~1 second)
3. **Clock synchronization** (retries until success)
4. **Gyro bias calibration** (~6 seconds, board must be stationary)
5. **Publishing begins** at 400 Hz

Total startup time: ~10 seconds. Keep the board still during this period.

## Covariance Values

From BMI088 datasheet noise specifications:
- Accelerometer: 8.25e-4 (m/s^2)^2 per axis (175 ug/sqrt(Hz) @ 280Hz BW)
- Gyroscope: 1.37e-5 (rad/s)^2 per axis (0.014 deg/s/sqrt(Hz) @ 230Hz BW)
- Orientation: covariance[0] = -1 (not available)

## CubeMX-Managed Peripherals

| Peripheral | Pin(s) | Function |
|---|---|---|
| ADC3 | PC2_C, PF7-10 | Analog inputs A1-A7 |
| CORDIC | - | Hardware math accelerator for attitude filter |
| FDCAN1 | PD0/PD1 | CAN bus |
| FDCAN2 | PB12/PB13 | CAN bus |
| FDCAN3 | PG9/PG10 | CAN RX for AMI mission state |
| I2C2 | PF0 (SDA) / PF1 (SCL) | BMI088 IMU |
| SPI1 | PA7 (MOSI) | WS2812 LED strip |
| TIM2 | - | 400Hz IMU sampling interrupt |
| USB OTG HS | PA11/PA12 | CDC for micro-ROS transport |

## Build System

Both Makefile and CMake are maintained. When CubeMX regenerates code, it only updates the Makefile. CMakeLists.txt must be updated manually for new source files.

### Build with Make (default, Cmd+Shift+B)

```bash
make GCC_PATH=/Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi/bin -j8
```

### Build with CMake

```bash
cmake --preset "Configure preset using toolchain file"
cmake --build build/binaries -j8
```

### Rebuild micro-ROS Static Library

```bash
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -i -v $(pwd):/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No `/dev/ttyACM*` device | USB not enumerating | Check USB cable, verify firmware is flashed |
| Agent runs but no session created | Framing mismatch or timing | Reset the STM32 board after starting the agent |
| Agent shows `create_client` then disconnects | Stack overflow or transport error | Verify FreeRTOS task stack >= 3000 words (12 KB) |
| Permission denied on `/dev/ttyACM0` | User not in dialout group | Run `sudo usermod -aG dialout $USER` and re-login |
| CMake build fails with undefined reference | Missing source in CMakeLists.txt | Check Makefile for new files after CubeMX regeneration |
| Wrong micro-ROS include path | CMakeLists.txt has `_ide` path | Use `microros_static_library/libmicroros/microros_include` (NOT `_ide`) |
