# First Steps

## Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Docker
- ROS 2 Humble (on the target PC)

## 1. Clone

```bash
git clone --recurse-submodules git@github.com:isc-fs/IFS08-DV-uDV.git
cd IFS08-DV-uDV
```

## 2. Build micro-ROS Static Library

Required on first clone and when adding new ROS message types.

```bash
docker pull microros/micro_ros_static_library_builder:humble
echo "y" | docker run --rm -i -v $(pwd):/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```

## 3. Build Firmware

```bash
make GCC_PATH=/path/to/arm-none-eabi/bin -j8
```

Output: `build/binaries/uDV.elf`, `.hex`, `.bin`

## 4. Flash and Run

Flash `build/binaries/uDV.bin` to the STM32, then start the micro-ROS agent on the target PC:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Keep the board stationary for ~10 seconds (gyro calibration), then verify:

```bash
ros2 topic hz /imu/data_raw
```

See [changelog.md](changelog.md) for full documentation.
