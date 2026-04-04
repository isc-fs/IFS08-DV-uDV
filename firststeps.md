# First Steps: micro-ROS Agent Setup

This guide walks through setting up the micro-ROS agent on the target computer to communicate with the STM32H733 over USB CDC.

## Prerequisites

- Ubuntu 22.04 (Jammy) or compatible Linux distribution
- ROS 2 Humble installed and sourced
- USB cable connected to the STM32 board

## 1. Create the micro-ROS Agent Workspace

```bash
mkdir -p microros_agent_ws/src
cd microros_agent_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git micro_ros_msgs
cd ..
```

## 2. Build the Agent

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Connect the STM32 Board

Plug in the STM32 via USB. Verify the CDC device appears:

```bash
ls /dev/ttyACM*
```

You should see something like `/dev/ttyACM0`. If the device doesn't appear, check that the board is flashed with the micro-ROS firmware and USB is properly configured.

### Permissions

If you get permission errors, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for the change to take effect.

## 4. Run the Agent

```bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the actual device path from step 3.

On successful connection you should see output similar to:

```
[...] info | TermiosAgentLinux.cpp | init                  | running...          | fd: 3
[...] info | Root.cpp             | create_client         | create              | client_key: 0x...
[...] info | SessionManager.hpp   | establish_session     | session established | client_key: 0x...
[...] info | ProxyClient.cpp      | create_participant    | participant created | client_key: 0x...
[...] info | ProxyClient.cpp      | create_topic          | topic created       | ...
[...] info | ProxyClient.cpp      | create_publisher      | publisher created   | ...
[...] info | ProxyClient.cpp      | create_datawriter     | datawriter created  | ...
```

## 5. Verify Communication

In a new terminal, check the topics are available and data is flowing:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

### Available Topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 400 Hz | BMI088 accelerometer and gyroscope data |
| `/imu/status` | `std_msgs/msg/Int32` | ~1 Hz | IMU driver status code (0 = OK, see below) |

### Check IMU data

```bash
ros2 topic echo /imu/data_raw
```

You should see `linear_acceleration` (m/s^2) and `angular_velocity` (rad/s) values updating at 400 Hz. The `orientation` field is not populated (covariance[0] = -1).

The IMU publisher uses **best-effort QoS** for maximum throughput. To subscribe, match the QoS:

```bash
ros2 topic echo /imu/data_raw --qos-profile sensor_data
```

```bash
ros2 topic hz /imu/data_raw
```

Should report ~400 Hz with minimal jitter (sampling is driven by a hardware timer interrupt via TIM2 + semaphore).

### Timestamps

Messages carry epoch-synchronized timestamps with sub-microsecond precision:
- Clock synchronized with the agent via NTP-like `rmw_uros_sync_session` protocol
- Timestamps captured at the exact TIM2 interrupt moment using the DWT cycle counter (528 MHz)
- Re-synced every ~10 seconds to correct crystal drift
- Deterministic 2.5ms intervals between samples for SLAM pre-integration

### Startup Sequence

After flashing and connecting, the board goes through:
1. **USB CDC enumeration** (~2 seconds)
2. **micro-ROS agent connection** (~1 second)
3. **Clock synchronization** (retries until success)
4. **Gyro bias calibration** (~6 seconds, board must be stationary)
5. **Publishing begins** at 400 Hz

Total startup time: ~10 seconds. Keep the board still during this period.

### Covariance Values

The IMU message includes covariance matrices populated from the BMI088 datasheet:
- Accelerometer: 8.25e-4 (m/s^2)^2 per axis (175 ug/sqrt(Hz) noise density)
- Gyroscope: 1.37e-5 (rad/s)^2 per axis (0.014 deg/s/sqrt(Hz) noise density)
- Orientation: covariance[0] = -1 (not available)

### Check IMU status

```bash
ros2 topic echo /imu/status
```

| Value | Meaning |
|---|---|
| `0` | `BMI088_OK` — sensor reads working normally |
| `-1` | `BMI088_ERR_PARAM` — initialization parameter error |
| `-2` | `BMI088_ERR_I2C` — I2C bus communication failure |
| `-3` | `BMI088_ERR_ID` — unexpected chip ID (not a BMI088) |
| `-99` | imuTask has not started (likely insufficient FreeRTOS heap) |

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No `/dev/ttyACM*` device | USB not enumerating | Check USB cable, verify firmware is flashed |
| Agent runs but no session created | Framing mismatch or timing | Reset the STM32 board after starting the agent |
| Agent shows `create_client` then disconnects | Stack overflow or transport error | Verify FreeRTOS task stack >= 3000 words (12 KB) |
| Permission denied on `/dev/ttyACM0` | User not in dialout group | Run `sudo usermod -aG dialout $USER` and re-login |

## Rebuilding the micro-ROS Static Library

If you need to regenerate the micro-ROS static library (e.g., after changing ROS message types), run from the project root:

```bash
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -i -v $(pwd):/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```

This reads compiler flags from the `Makefile` (`print_cflags` target) and cross-compiles the micro-ROS libraries for the Cortex-M7. The output is placed in `micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/`.
