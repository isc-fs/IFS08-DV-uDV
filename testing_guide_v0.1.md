# uDV v0.1 Testing Guide

Quick guide to verify the 4 communication interfaces implemented in `feat/7-release-v0.1`.

## Prerequisites

- STM32H733 board flashed with `build/binaries/uDV.bin`
- USB-C cable (micro-ROS transport)
- CAN bus analyzer or transceiver for FDCAN1 (PD0/PD1) and FDCAN3 (PG9/PG10)
- Host PC with ROS 2 Humble and micro-ROS agent installed
- All CAN buses at **500 kbps**, classic CAN

## Build & Flash

```bash
make GCC_PATH=/Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi/bin -j8
# Flash build/binaries/uDV.bin via ST-Link or DFU
```

## Startup Sequence

1. Board powers on, LEDs show dim white (idle)
2. USB CDC enumerates (~2s)
3. micro-ROS agent connects, clock syncs (~1s)
4. Gyro calibration runs (~6s, board must be stationary)
5. IMU publishing begins at 400 Hz

---

## Test 1: micro-ROS Communication

**Bus:** USB CDC (`/dev/ttyACM0`)

### Launch agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v 6
```

### Verify topics

```bash
ros2 topic list
```

Expected topics:
- `/imu` (sensor_msgs/Imu, 400 Hz)
- `/imu/status` (std_msgs/Int32, ~0.1 Hz)
- `/steering/data` (std_msgs/Float32, ~10 Hz)
- `/res/status` (std_msgs/Int32, ~10 Hz)
- `/ami/mission` (std_msgs/Int32, ~10 Hz)
- `/cmd_test` (std_msgs/Int32, subscriber)

### Test subscriber

```bash
# Toggle OK_STATUS LED (PD14) on each publish
ros2 topic pub --once /cmd_test std_msgs/msg/Int32 "{data: 1}"
```

LED PD14 should toggle with each message received.

### Monitor data

```bash
ros2 topic echo /imu --qos-profile sensor_data
ros2 topic hz /imu          # expect ~395-400 Hz
ros2 topic echo /steering/data
ros2 topic echo /res/status           # -1 if RES not connected
ros2 topic echo /ami/mission          # 255 if no mission selected
```

---

## Test 2: CAN AMI + ASSI (FDCAN3)

**Bus:** FDCAN3 (PG9 TX / PG10 RX), 500 kbps

### Send mission select

From a CAN analyzer, transmit:

| CAN ID | DLC | Data | Expected Result |
|--------|-----|------|-----------------|
| `0x503` | 1 | `00` | LEDs green (Manual) |
| `0x503` | 1 | `01` | LEDs yellow (Acceleration) |
| `0x503` | 1 | `02` | LEDs blue (Skidpad) |
| `0x503` | 1 | `03` | LEDs red (Autocross) |
| `0x503` | 1 | `04` | LEDs purple (Track Drive) |
| `0x503` | 1 | `05` | LEDs white (EBS Test) |
| `0x503` | 1 | `06` | LEDs cyan (Inspection) |
| `0x503` | 1 | `FF` | LEDs dim white (idle) |

Also verify `/ami/mission` topic updates on each message.

---

## Test 3: CAN Steering (FDCAN3)

**Bus:** FDCAN3 (shared with AMI), 500 kbps

### Receive steering angle (LWS sensor feedback)

Send from CAN analyzer:

| CAN ID | DLC | Data (hex) | Meaning |
|--------|-----|------------|---------|
| `0x2B0` | 8 | `E8 03 0A 01 00 00 00 00` | Angle: 100.0 deg, Speed: 40 deg/s, Status: OK |
| `0x2B0` | 8 | `18 FC 00 01 00 00 00 00` | Angle: -100.0 deg, Speed: 0, Status: OK |

Byte breakdown:
- Bytes 0-1: angle (int16 LE, 0.1 deg/bit) — `0x03E8` = 1000 = 100.0 deg
- Byte 2: speed (4 deg/s per bit) — `0x0A` = 10 = 40 deg/s
- Byte 3: status — bit 0=OK, bit 1=CAL, bit 2=TRIM

Verify with:
```bash
ros2 topic echo /steering/data    # should show 100.0 or -100.0
```

### Send steering commands (uDV to steering actuator)

These functions are available for the control pipeline (not called automatically yet):

| CAN ID | DLC | Data | Meaning |
|--------|-----|------|---------|
| `0x010` | 1 | `01` | Motor start |
| `0x010` | 1 | `00` | Motor stop |
| `0x020` | 4 | int32 LE, scale 1/100 deg | Desired angle |

Example: to command 45.0 deg → `0x020` data = `0x94 0x11 0x00 0x00` (4500 in LE)

---

## Test 4: CAN RES (FDCAN1)

**Bus:** FDCAN1 (PD0 RX / PD1 TX), 500 kbps

### With real RES hardware

1. Connect RES receiver CAN to FDCAN1
2. Set RES Node-ID to `0x11` (DIP switches 1+5 ON)
3. Set RES baud rate to 500 kbps (DIP switches 7+8 = `1 1`)
4. Power on RES — it sends boot-up on `0x711`
5. uDV automatically sends NMT operational: `0x000` data `[0x01, 0x11]`
6. RES begins sending PDOs on `0x191` every 30 ms

Verify:
```bash
ros2 topic echo /res/status
# 0  = OK (PDOs arriving, no E-Stop)
# 1  = E-Stop active
# -1 = Timeout (no PDO received for >150 ms)
```

### Without RES hardware (CAN analyzer)

Simulate RES boot-up:
```
TX: ID=0x711  DLC=1  Data=[0x00]
```
uDV should respond with NMT on `0x000`.

Simulate RES PDO (repeat every 30 ms):
```
TX: ID=0x191  DLC=8  Data=[0x00 0x00 0x00 0x00 0x00 0x00 0x64 0x00]
```
- Byte 0 bit 0 = 0 → no E-Stop
- Byte 6 = 0x64 → radio quality 100%

To simulate E-Stop:
```
TX: ID=0x191  DLC=8  Data=[0x01 0x00 0x00 0x00 0x00 0x00 0x64 0x00]
```
`/res/status` should change to `1`.

---

## Test 5: Data Logger TX (FDCAN1)

**Bus:** FDCAN1 (same as RES), 500 kbps

Monitor FDCAN1 with a CAN analyzer. The following messages should appear every **100 ms**:

| CAN ID | DLC | Content |
|--------|-----|---------|
| `0x500` | 8 | DV dynamics 1: speed, steering angle, brake, motor |
| `0x501` | 6 | DV dynamics 2: longitudinal/lateral accel, yaw rate |
| `0x502` | 5 | DV system status: AS state, EBS, AMI, steering |

Refer to FS Driverless Specification 2026 (DS 2.2) for full field definitions.

---

## Troubleshooting

| Symptom | Check |
|---------|-------|
| No `/dev/ttyACM0` | USB cable, firmware flashed? Reset board |
| Agent connects then drops | Check defaultTask stack (12 KB), reset after agent starts |
| `/res/status` always -1 | RES not connected or FDCAN1 wiring/termination issue |
| LEDs don't change color | Check SPI1 MOSI (PA7) wiring to WS2812 strip |
| No DL messages on FDCAN1 | Verify 120 ohm termination on both ends of CAN bus |
| Steering angle always 0 | No 0x2B0 messages on FDCAN3, check LWS sensor connection |
