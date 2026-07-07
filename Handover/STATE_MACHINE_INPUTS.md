# State-machine input sourcing (uDV)

Where `StateManager` / `EbsManager` get each input, the physical board signal
(per `06_MicroDV/PCB DV.kicad_sch`, MCU header **J8**), and what still needs work.

Board input map (J8 → STM32 pin → ADC ch → schematic net):

| Pad | STM32 | ADC ch | Schematic net | Meaning |
|-----|-------|--------|---------------|---------|
| A1  | PF7   | INP3   | `RES_1_IN`    | RES channel-1 SDC-loop feedback (also K2 contact) |
| A2  | PF8   | INP7   | `GO_RES`      | **RES GO signal** (go-to-drive) |
| A3  | PF9   | INP2   | `24V_DIR`(ASMS) | ASMS master switch |
| A4  | PF10  | INP6   | `PRES_2_IN`   | EBS actuator-2 storage (air-tank) pressure |
| A5  | PC0   | INP10  | `PRES_1_IN`   | EBS actuator-1 storage (air-tank) pressure |
| A6  | PC1   | INP11  | `24V_EBS`(TSMS) | TSMS master switch |
| D1  | PB4   | —      | `EBS_1`       | EBS actuator-1 drive (out) |
| D2  | PB5   | —      | `EBS_2`       | EBS actuator-2 drive (out) |
| D4  | PB7   | —      | `EBS_SDC`     | SDC drive (out) |
| D5  | PB8   | —      | `ASSI_DIN`    | ASSI status-LED data (out, via SN74AHCT125) |

`RES_2_IN` exists on the board but is **not** routed to any MCU pin. D3/D6 are spare.

## ✅ Correctly sourced (board-local)

| Signal | Source | Notes |
|--------|--------|-------|
| `asms_on` | `hardware_io_read_asms_on()` → A3 | |
| `ts_active` | `asms_on && hardware_io_read_tsms_on()` → A3 && A6 | **Fixed 2026-07-01.** Was CAN `0x504` (never transmitted → TS stuck off, EBS hung at `WaitTS`). Now sensed locally in both `StateManager::updateSignals()` and `EbsManager::WaitTS`. |
| `abs_checks_ok` | `EbsManager` storage-pressure checks → A4/A5 | |
| `ebs_activated` | `hardware_io_is_ebs_active()` → D1/D2 readback | |

## ⏳ Deferred — DO NOT LOSE

### `r2d` — legacy `StateManager` input, left as-is (the live FSM no longer uses it)
- **Current:** `signals_.r2d = g_can_r2d.load()` (ECU R2D **confirm** on CAN `0x511`;
  see `can_globals.h`). The old `updateState()` in `state_manager.cpp` still gates its
  `OFF→DRIVING` branch on `signals_.r2d`, but that method is now dead for AS decisions:
  the live transition runs through the pure `as_next_state()` (`as_transition.hpp`),
  which is driven by the RES **GO** (`res_go`), not `r2d`. `StateManager` is kept only
  to source signals + telemetry. See the TODO at `state_manager.cpp:51`.
- **Important distinction:** `R2D` ≠ `GO`. The **GO** signal comes from the **RES**
  (via CAN; also hardwired on **A2 = `GO_RES`**). The **R2D _request_** is what the uDV
  asserts _to_ the ECU (0x510), and 0x511 is the ECU's R2D _confirm_ back to the uDV
  (`app_task.cpp` gates the 0x510 request loop on `!g_can_r2d`).
- **Consequence to revisit:** the drive trigger in the live FSM is already the RES
  **GO** (`res_go` + `mission_valid` + DV-ready). Only the vestigial `updateState()`
  still references `signals_.r2d`; remove it or repurpose when `StateManager` is
  trimmed. Do not assume `updateState()` reflects the running behaviour.

## ✅ Sourced via CAN by design — settled, no change

| Signal | Source | Why CAN (not board-local) |
|--------|--------|---------------------------|
| `sdc_res_open` | `g_can_sdc_res_open` | "RES opened the SDC" (EMERGENCY-vs-FINISHED discriminator at mission end). **Currently has NO CAN source** — the old `0x506` mapping now carries motor rpm (`can_globals.h`), so this atomic stays `false` until a source exists. Derivation from A1=`RES_1_IN` was considered but rejected — only RES channel-1 reaches the MCU (`RES_2_IN` doesn't). Consumed only by the vestigial `StateManager::updateState()`, not the live `as_next_state()`. |
| `brakes_engaged` | `g_can_brake_over_limit` (CAN `0x505`) | The ECU's **binary verdict** (`VCU_brake_over_limit`, byte0 bool): brake_raw above the hard-braking limit (`BrakeDvHardRaw`, ECU-owned — no uDV-side threshold to tune). Replaced the old float32-bar reading + local threshold, which never matched what the ECU transmits. Not derivable on the uDV — A4/A5 are EBS **air-tank storage** pressures, not the brake line. |
| GO (feeds `r2d` today) | CAN (RES) | RES go-to-drive signal. Also hardwired on A2=`GO_RES`, but consumed from CAN. |

## CAN/ROS by design (not board signals)
`mission_selected` (CAN mission-id + ROS set-ready), `mission_finished` (ROS),
`vehicle_standstill` (IMU).

## Cleanup noted
`hardware_io_read_sdc_is_ready()` (A1) and `hardware_io_read_sdc_res_open()` (A2) are
**dead** and **mislabeled** (A1=`RES_1_IN`, A2=`GO_RES`). Repurpose for the RES/GO/SDC
work above or remove; fix the comments either way.
