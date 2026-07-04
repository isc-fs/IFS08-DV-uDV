# Next Steps — uDV Firmware Roadmap

This document tracks what has been done and what remains to converge the uDV firmware toward the target architecture (based on [IFS08-DV `micro` branch](https://github.com/isc-fs/IFS08-DV/tree/micro)).

---

## Current State (branch `fix/2-fdcan3-ws2812`)

### Task Map

| Task | Priority | Stack | Trigger | Status |
|------|----------|-------|---------|--------|
| imuTask | AboveNormal | 8 KB | TIM2 semaphore (400 Hz) | Working |
| canTask | AboveNormal | 4 KB | canRxQueue (event-driven) | New — needs HW test |
| defaultTask | Normal | 12 KB | imuQueue (event-driven) | Working |
| amiTask | BelowNormal | 2 KB | xTaskNotifyGive from canTask | New — needs HW test |

### Architecture Diagram

```
FDCAN3 ISR → canRxQueue → canTask → dispatch(0x503) → notify → amiTask → WS2812
                                                                           (SPI1/PA7)
TIM2 ISR → imuSem → imuTask → imuQueue → defaultTask → micro-ROS publish
                                                          (USB CDC)
```

### What's Been Done

- [x] FDCAN3 runtime bring-up (filter, start, RX FIFO0 notification)
- [x] CAN RX ISR-to-queue bridge (`can_isr_push_rx`)
- [x] CAN message dispatcher with `switch` on ID (`can_rx_dispatch`)
- [x] CAN ID 0x503 (mission select) parsed and stored in `g_mission_index`
- [x] WS2812 SPI driver (8 LEDs, GRB, SPI1 MOSI on PA7)
- [x] Mission color table (10 colors indexed 0–9)
- [x] amiTask with notification-driven wake and LED update
- [x] SPI1 HAL init + MSP (GPIO PA7 AF5, ~2 MHz clock)
- [x] HAL SPI module enabled in `stm32h7xx_hal_conf.h`
- [x] micro-ROS static library rebuilt with hard-float VFP flags
- [x] USB CDC transport fixed (FS → HS references)
- [x] CMake toolchain detection updated
- [x] Firmware compiles and links cleanly (105 KB text)

---

## Phase 1 — Hardware Validation (Immediate)

Priority: **Critical** — must be done before any further development.

### 1.1 Flash and Test FDCAN3 RX

- Flash `fix/2-fdcan3-ws2812` to the STM32H733 board
- Connect a CAN bus tool (PCAN, CANalyzer, or another STM32 node) to FDCAN3 pins (PG9 TX / PG10 RX)
- Send CAN frame: ID=0x503, DLC=1, Data[0]=0x01
- Verify OK_STATUS LED (PD14) lights up — confirms canTask received the message
- Send different mission indices (0x00–0x09, 0xFF) and verify dispatch works

### 1.2 Test WS2812 LED Strip

- Connect 8 WS2812 LEDs to PA7 (SPI1 MOSI), with appropriate 5V power and logic level shifting if needed
- On boot, LEDs should show dim white (20, 20, 20) — idle state
- Send CAN 0x503 with Data[0]=0x01 → LEDs should turn yellow (Acceleration)
- Send Data[0]=0x02 → blue (Skidpad)
- Send Data[0]=0xFF → red (invalid index)
- Verify color accuracy and LED update latency

### 1.3 Regression Test Existing Features

- Confirm imuTask still publishes IMU data at ~400 Hz over micro-ROS
- Confirm micro-ROS agent connects via USB CDC
- Monitor `imu/data_raw` and `imu/status` topics — data should be unchanged
- Check FreeRTOS heap usage (`xPortGetFreeHeapSize`) — ensure the two new tasks don't exhaust memory

### 1.4 SPI Timing Verification

- Use a logic analyzer on PA7 to verify SPI bit timing
- WS2812 expects ~800 kHz data rate (1.25 µs per bit)
- Current SPI prescaler = 64 → ~2.06 MHz SPI clock → each SPI byte ≈ 3.9 µs
- Each WS2812 bit = 1 SPI byte → bit period ≈ 3.9 µs (should be ~1.25 µs)
- **This may need prescaler adjustment** — if LEDs don't respond, try prescaler 16 (~8.25 MHz → 1 byte ≈ 0.97 µs) or prescaler 32 (~4.125 MHz → 1 byte ≈ 1.94 µs)
- Adjust `SPI_BAUDRATEPRESCALER_*` in `MX_SPI1_Init()` accordingly

---

## Phase 2 — Additional CAN Message Parsing

Priority: **High** — required for vehicle integration.

### 2.1 Steering Sensor (CAN ID 0x2B0)

The MicroDV project already parsed this. Add a case in `can_rx_dispatch()`:

```
case 0x2B0:
    // Bytes 0-1: steering angle (int16_t LE, 0.1 deg/bit)
    // Byte 2: angular speed (uint8_t, 4 deg/s per bit)
    // Byte 3: status flags (bit0=OK, bit1=CAL, bit2=TRIM)
```

- Store parsed values in shared globals (volatile or atomic)
- Decide whether to publish over micro-ROS (new topic) or just make available to AppTask

### 2.2 Define Full CAN ID Table

Work with the CE (Central Electronics) team to define the complete CAN ID table for the AMI bus on FDCAN3. At minimum:

| CAN ID | Source | Data | Priority |
|--------|--------|------|----------|
| 0x503 | AMI | Mission select (byte 0) | Done |
| 0x2B0 | LWS sensor | Steering angle, speed, status | Phase 2 |
| 0x??? | ECU | Vehicle speed / standstill | Needed for state machine |
| 0x??? | ECU | Tractive system status | Needed for state machine |
| 0x??? | ECU | ASMS switch state | Needed for state machine |
| 0x??? | ECU | R2D (ready to drive) signal | Needed for state machine |

### 2.3 CAN TX Capability

Currently canTask is receive-only. The `micro` branch also sends:
- Control commands (accel + steer floats packed into 8 bytes)
- R2D (ready to drive) signal

Add any new CAN transmit helpers to `Core/Src/can_interface.cpp` so the active firmware keeps a single CAN interface layer.

---

## Phase 3 — AppTask and State Machine

Priority: **High** — core vehicle autonomy logic.

This is the biggest missing piece compared to the `micro` branch. The AppTask is the top-level vehicle supervisor.

### 3.1 Create AppTask

- Priority: Highest (above imuTask and canTask)
- Stack: 4 KB
- Runs continuously (no delay in main loop)
- Reads shared state from canTask (atomics/volatiles) and ROS (future)
- Drives the AS (Autonomous System) state machine

### 3.2 AS State Machine

Implement the state transitions from the `micro` branch:

```
OFF → READY → DRIVING → FINISHED
                ↓
            EMERGENCY
```

**Inputs to the state machine:**
- `asms_on` — ASMS main switch (GPIO read)
- `ts_active` — Tractive system active (GPIO or CAN)
- `ebs_activated` — EBS engaged (internal flag)
- `abs_checks_ok` — EBS init sequence complete
- `brakes_engaged` — Brake pressure present (ADC)
- `mission_selected` — `g_mission_index != 0xFF`
- `r2d` — Ready to drive signal (from CAN)
- `vehicle_standstill` — IMU velocity near zero (from imuTask or CAN)
- `mission_finished` — From ROS action result

### 3.3 EBS Manager

Port the `EbsManager` from the `micro` branch (non-blocking init sequence):

1. Wait for SDC ready
2. Wait for SDC toggle (operator confirmation)
3. Check storage pressures (ADC3 channels)
4. Close AS SDC relay
5. Wait for tractive system
6. Check actuator 1 (brake pressure)
7. Wait inter-actuator period
8. Check actuator 2
9. Done / Failed

Requires: ADC3 pressure readings, GPIO for SDC relay and actuator control.

### 3.4 HardwareIO Abstraction

Create a `hardware_io.h/.c` module (like the `micro` branch's `HardwareIO` class, but in C):

```c
bool     hw_read_asms_on(void);
bool     hw_read_ts_active(void);
bool     hw_read_sdc_ready(void);
bool     hw_read_sdc_res_open(void);
float    hw_read_main_pressure(void);
float    hw_read_actuator1_pressure(void);
float    hw_read_actuator2_pressure(void);
float    hw_read_brake_pressure(void);
void     hw_set_as_close_sdc(bool on);
void     hw_enable_ebs_actuator1(bool en);
void     hw_enable_ebs_actuator2(bool en);
void     hw_toggle_watchdog(void);
uint32_t hw_now_ms(void);
```

Map each function to the actual GPIO/ADC pins on the uDV board. This requires the hardware schematic.

### 3.5 Watchdog

AppTask must toggle a watchdog GPIO every loop iteration. If it stalls >50 ms, the vehicle ECU cuts power. This is a safety requirement — the watchdog toggle must never be blocked by SPI, I2C, or queue operations.

---

## Phase 4 — Evolve defaultTask into RosTask

Priority: **Medium** — required for full autonomy stack.

### 4.1 Add Service Client: SetMission

The `micro` branch uses a `set_mission` ROS2 service to tell the remote planner which mission is selected. Add:

```c
rcl_client_t set_mission_client;
// Service type: ros2_interface/srv/SetMission
```

This requires the `ros2_interface` custom message package to be included in the micro-ROS static library build. Update `micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages/` with the custom package.

### 4.2 Add Action Client: StartMission

The `micro` branch uses a `start_mission` ROS2 action to execute missions. The action server runs on the remote PC and streams feedback (acceleration, steering commands) back to the STM32.

**Feedback fields consumed by the firmware:**
- `acceleration` (float) → forwarded to CAN TX as control command
- `steering` (float) → forwarded to CAN TX as control command
- `finished` (bool) → triggers state transition to FINISHED
- `emergency` (bool) → triggers state transition to EMERGENCY

### 4.3 Command Queues

Following the `micro` branch pattern, add two command queues:

```
AppTask → g_ros_cmd_queue (4 items) → RosTask (defaultTask)
AppTask → g_can_cmd_queue (8 items) → canTask
```

Commands:
- `ROS_CMD_SET_MISSION(id)` — call set_mission service
- `ROS_CMD_START_MISSION(id)` — send start_mission action goal
- `ROS_CMD_CANCEL_MISSION` — cancel active goal
- `CAN_CMD_SEND_CONTROL(accel, steer)` — TX control frame on FDCAN

### 4.4 Shared State via Atomics/Volatiles

The `micro` branch uses C++ `std::atomic`. Since we're in C, use `volatile` for single-writer shared state:

```c
// Written by RosTask (future), read by AppTask
volatile float    g_accel_cmd;
volatile float    g_steer_cmd;
volatile bool     g_finished_cmd;
volatile bool     g_emergency_cmd;
volatile bool     g_mission_going_cmd;

// Written by canTask, read by AppTask
volatile bool     g_can_r2d;
volatile bool     g_can_vehicle_standstill;
volatile uint8_t  g_mission_index;  // already exists

// Written by AppTask, read by canTask
volatile bool     g_can_listen_go;
```

---

## Phase 5 — FDCAN1 and FDCAN2 Bring-Up

Priority: **Medium** — depends on vehicle wiring.

### 5.1 Determine Bus Assignments

The board has three FDCAN peripherals. Current assignment:

| FDCAN | Pins | Current Use | Potential Use |
|-------|------|-------------|---------------|
| FDCAN1 | PD0/PD1 | Init only | Vehicle ECU bus? |
| FDCAN2 | PB12/PB13 | Init only | Accumulator bus? |
| FDCAN3 | PG9/PG10 | Active (AMI bus) | AMI bus |

Work with the CE team to confirm which CAN bus connects to which FDCAN peripheral.

### 5.2 Multi-Bus Dispatch

Extend `can_isr_push_rx()` to tag messages with their source bus:

```c
typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
    uint8_t  bus;    // 0=FDCAN1, 1=FDCAN2, 2=FDCAN3
    uint8_t  _pad[2];
} can_msg_t;
```

And extend `can_rx_dispatch()` to switch on `bus` first, then `id` (like IFS08-CE-ECU pattern).

---

## Phase 6 — IMU Data on CAN

Priority: **Low** — nice to have for other nodes.

### 6.1 Publish IMU Over CAN

The `micro` branch's CanTask also transmits IMU data over CAN for other vehicle nodes. Add a periodic CAN TX in canTask:

- Pack accelerometer + gyroscope into CAN frames
- Derive vehicle standstill flag from IMU velocity
- Set `g_can_vehicle_standstill` for the state machine

### 6.2 Vehicle Standstill Detection

The state machine needs to know if the vehicle is stopped (for FINISHED state transition). Options:
- Integrate accelerometer data over time to estimate velocity
- Use wheel speed from CAN if available
- Simple threshold on gyroscope readings

---

## Phase 7 — Cleanup and Hardening

Priority: **Low** — before competition.

### 7.1 Remove MicroDV Directory from Dev

The bad merge (commit `a305ad1`) added the entire `MicroDV/` project to `dev`. This needs to be reverted:

```
git revert a305ad1  # or rebase dev onto 93f2dfb
```

### 7.2 Error Handling

- Add CAN bus-off recovery (`HAL_FDCAN_GetError`, auto-restart)
- Add queue-full detection and logging
- Add SPI timeout handling for WS2812
- Add heartbeat LED blink pattern on OK_STATUS to indicate system health

### 7.3 Memory Budget

Current heap: 40 KB (FreeRTOSConfig.h). Verify with `xPortGetFreeHeapSize()` and `xPortGetMinimumEverFreeHeapSize()` that the 4 tasks + queues + micro-ROS fit comfortably.

### 7.4 Power Consumption

WS2812 LEDs at full white draw ~60 mA per LED (480 mA for 8). Ensure the power supply can handle this alongside the STM32 and CAN transceivers.

### 7.5 CAN Bus Termination

Verify 120 ohm termination resistors are present on both ends of each CAN bus. Missing termination causes unreliable communication.

---

## Architecture Target (Converged)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     STM32H733 — FreeRTOS (1 kHz tick, 528 MHz)          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐  │
│  │   AppTask    │ │   imuTask    │ │   canTask    │ │  defaultTask │  │
│  │   Prio: High │ │   Prio: AbN  │ │   Prio: AbN  │ │  Prio: Norm  │  │
│  │              │ │              │ │              │ │              │  │
│  │ State machine│ │ BMI088 400Hz │ │ FDCAN RX/TX  │ │ micro-ROS    │  │
│  │ EBS manager  │ │ Gyro calib   │ │ Multi-bus    │ │ IMU publish  │  │
│  │ Watchdog     │ │ Attitude est │ │ Dispatch     │ │ Srv + Action │  │
│  │ Ctrl dispatch│ │              │ │              │ │ Feedback     │  │
│  └──────┬───────┘ └──────┬───────┘ └──────┬───────┘ └──────┬───────┘  │
│         │                │                │                │           │
│  ┌──────────────┐        │                │                │           │
│  │   amiTask    │        │                │                │           │
│  │   Prio: BelN │        │                │                │           │
│  │   WS2812 LED │        │                │                │           │
│  └──────────────┘        │                │                │           │
│         │                │                │                │           │
│   ┌─────┴────────────────┴────────────────┴────────────────┘           │
│   │  Atomics:  g_accel_cmd, g_steer_cmd, g_mission_index, ...          │
│   │  Queues:   imuQueue, canRxQueue, rosCmd Queue, canCmd Queue        │
│   │  Sema:     imuSem    Notify: amiTask                              │
│   │  Singletons: StateManager, EbsManager                             │
│   └────────────────────────────────────────────────────────────────────│
│                                                                         │
├─────────────────────────────────────────────────────────────────────────┤
│  FDCAN1    FDCAN2    FDCAN3    USB HS    I2C2    SPI1    TIM2    ADC3  │
│  (ECU)     (ACU?)    (AMI)     (uROS)   (IMU)   (LED)   (400Hz) (P)  │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Dependencies and Blockers

| Item | Blocked By | Owner |
|------|-----------|-------|
| CAN ID table | CE team CAN protocol spec | CE team |
| FDCAN1/2 bus assignment | Hardware schematic / wiring | HW team |
| HardwareIO pin mapping | Board schematic | HW team |
| EBS pressure thresholds | Mechanical team specs | Mech team |
| ros2_interface package | Custom msg definitions | DV team |
| Watchdog GPIO pin | Board schematic | HW team |
| WS2812 logic level | 3.3V→5V level shifter needed? | HW team |
