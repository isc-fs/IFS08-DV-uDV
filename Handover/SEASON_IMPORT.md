# uDV Firmware — Season Import & Handover Guide

**Purpose.** Everything the next-season team needs to import this repository into a fresh
season repo and carry the driverless firmware forward: what to take, what to drop, the
cross-repo contracts you inherit, the open coordination work in flight, and the traps that
will silently break the car if you miss them.

> This complements the existing `Handover/` docs (architecture, FSM, pipeline interface,
> missions, ASSI bridge). Read [`Handover/README.md`](README.md) first for getting-started;
> this doc is the migration layer on top.

Snapshot at handover:
- **`main` = `c3f8350`**, **`dev` = `f6ca95b`** — ⚠️ **dev is 4 commits ahead of main** (the
  0x7AA AS-Emergency-cause telemetry, #192). **Promote `dev` → `main` before importing** so
  `main` is the clean import point.
- **Submodule** `micro_ros_stm32cubemx_utils` @ `3539782` (fork, branch `humble-isc`).
- Board: **STM32H733XG** (Cortex-M7, 528 MHz), FreeRTOS + micro-ROS over USB CDC.

---

## 1. What to import (and what not to)

### Bring these
| Item | Notes |
|---|---|
| **`main`** (after promoting dev→main) | The canonical firmware. |
| **`dev`** | Active integration branch — all work lands here first, then promotes to `main`. |
| **`prerun/rosbag-onboard`** | **Permanent branch, always in the repo alongside main/dev.** Loads SW on the car *lifted off the ground* to test against a recorded rosbag (`BENCH_STUB_IMU_ROS=1` so the bag's `/imu` is the only source). Re-create it in the new repo. |
| **Submodule** `micro_ros_stm32cubemx_utils` (fork `humble-isc`) | Tracks a **prebuilt `libmicroros.a`** so a plain `--recurse-submodules` clone builds with no Docker. See §6. |
| **All docs** | `CLAUDE.md` (the authoritative architecture brief), `docs/*`, `Handover/*`, `firststeps.md`. |
| **`uDV.ioc`** | The CubeMX project — see §7 gotchas. |

### Reference only — do NOT carry over / do NOT implement
- **PR #162** (`feat/imu-yaw-align`) and **PR #170** (`feat/emergency-steering-centering`) —
  kept in *this* repo as reference; not merged, not carried to the new repo.

### Drop entirely
- `feat/18-ebs`, `feat/17-ECU-inspection_redy` — superseded, no value.
- Any remaining short-lived `fix/*` / `feat/*` branches already merged into `main`.

### Already handled this cycle (context, no action)
- Deleted: `feat/asb-pressure-emergency` (content in main), `feat/service-brake-finish`
  (superseded by DV_STOPPING), `feat/71-steering-ratio` (kinematics in main; its one unique
  bit — the 0x7AA emergency-cause telemetry — was salvaged into #192), `feat/pd15-iwdg-notok-led`.

### How to actually move it — use a git transfer, NOT a "copy from local"

**Everything that matters is version-controlled**, including the full test suite (source,
mocks, stubs, Makefile — see §8). Only *build artifacts* are gitignored (`build/`, the compiled
host-test binaries, `*.o`, `*.dSYM`), and those regenerate. So a git-native transfer carries the
complete, correct tree — and a "commit my local folder" copy would be **worse**: it risks
dragging the ignored build junk into the new repo.

Recommended, in order of preference:

1. **Mirror push (keeps full history + all branches + tags).** In an empty new repo:
   ```bash
   git clone --bare git@github.com:isc-fs/IFS08-DV-uDV.git
   cd IFS08-DV-uDV.git
   git push --mirror git@github.com:isc-fs/<new-repo>.git
   ```
   Then in a fresh working clone of the new repo, re-point the **submodule** at whatever
   `micro_ros_stm32cubemx_utils` fork the new season uses (`.gitmodules` URL) and
   `git submodule update --init --recursive`. Prune the branches you don't want (§1/§3).
2. **Snapshot import (fresh history, cleaner, loses PR history).** Import only `main`'s tree as
   an initial commit in the new repo. Do it with `git archive` / `git worktree`, **not** a
   Finder copy, so `.gitignore` is honoured and no build junk comes along:
   ```bash
   git archive --format=tar main | (mkdir ../new && cd ../new && tar xf -)
   ```
   Add the submodule fresh in the new repo.

**Do NOT** `cp -r` the working directory or "upload from local" — it pulls in `build/`, the
compiled `test_*` binaries, `.o`/`.dSYM`, and any local scratch, none of which belong in git.

Either way the **submodule** is the one thing a plain push/archive won't fully carry (it's a
pointer, not content) — re-init it in the new repo per §6.

---

## 2. Architecture in one screen

Full detail in `CLAUDE.md`. The load-bearing shape:

- **FreeRTOS tasks**, each body in its own file (CubeMX regenerates a thin `freertos.c`):
  `defaultTask` (micro-ROS node), `imuTask` (BMI088 @ 400 Hz), `canTask`, `assiTask`,
  `safetyTask` (IWDG watchdog), `appTask` (**the AS state machine + EBS init + mission dispatch**).
- **The AS state machine is PURE and host-tested** — this is the safety core and the thing to
  protect during any refactor:
  - `Core/Inc/as_transition.hpp` — `as_next_state(prev, AsInputs)` decides the next AS state.
  - `Core/Inc/as_actuation.hpp` — `as_actuation(state, asms, dv_stopping)` decides EBS/SDC.
  - `Core/Inc/as_stop_latch.hpp` — the DV_STOPPING debounce+latch.
  - Exhaustively swept in `tests/host/` (~149,637 checks). `app_task.cpp` only wires signals in
    and applies the results — no decision logic lives in the loop.
- **Missions** are pure (`mission_*.cpp`, dispatched by `mission_registry.cpp`); `appTask` owns
  all CAN actuation. See `Handover/MISSIONS.md`.
- **Three FDCAN buses** with distinct roles (§4) + a **CAN-only pit-diag observability stream**
  on FDCAN2 (`0x7A0`–`0x7AA`) that MingoCAN decodes.

---

## 3. Branch & repo state at handover

Live remote branches (post-cleanup):
- **`main`, `dev`** — the two trunks.
- **`prerun/rosbag-onboard`** — permanent (carry over).
- **Merged into main** (safe to archive): `feat/ts-active-from-can`, `feat/176-dv-stopping`,
  `feat/asb-finish-at-standstill`, `fix/steering-motor-100hz`, `fix/adc-sample-time-crosstalk`,
  `fix/24-terminal-state-sdc`, `docs/force-ebs-not-brake-path`, `chore/gitignore-*`.
- **On dev, not yet main**: `feat/pitdiag-emergency-cause` (#192, the 0x7AA telemetry).
- **Reference / drop**: see §1.

---

## 4. Cross-repo contracts you inherit ⚠️

This firmware is one node in a multi-board system. **These interfaces are the real dependency
surface** — the new repo must keep them in lockstep with the peer repos. Bus roles:

### FDCAN1 — RES CANopen + on-board data logger
| ID | Dir | Meaning |
|---|---|---|
| `0x191` | RX | RES PDO: e-stop, go-signal, radio quality (**e-stop is ACTIVE-LOW** — undocumented deploy quirk) |
| `0x000` | TX | NMT set-operational (re-send gates on boot-up `data[0]==0x00`) |
| `0x500/0x501/0x502` | TX | Data-logger frames (`DV_driving_dynamics_1/2`, `DV_system_status`) — see `docs/IFS08_FSG26_datalogger.dbc` |

### FDCAN2 — ACU bus (**shared with `IFS08-CE-ECU` + AMS**) — the ECU contract
| ID | Dir | Meaning |
|---|---|---|
| `0x504` | ECU→uDV | **TS active** (VCU_ts_active, 100 ms) — TS is sourced from CAN, not the TSMS pin |
| `0x505` | ECU→uDV | Brake over-limit verdict (binary) |
| `0x506` | ECU→uDV | Motor rpm (int32 LE) — SLAM/standstill |
| `0x511` | ECU→uDV | DV R2D confirm |
| `0x507` | uDV→ECU | Torque command (int32 LE %, 20 ms) |
| `0x510` | uDV→ECU | DV R2D request |
| `0x512` | uDV→ECU | IMU broadcast (ax/ay/az + gx, 50 Hz) |
| **`0x513`** | uDV→ECU | **PROPOSED, not yet built** — AS-status byte for the AS-Emergency buzzer (ECU#142) |
| `0x7A0–0x7AA` | uDV→ECU | pit-diag observability stream (see §telemetry) |
| `0x7DE / 0x7DF` | ECU→uDV | pit-diag arm / steering-calib trigger |

**Contract source of truth:** `IFS08-DBCinator` (ECU-generated DBCs). Peer file: the ECU's
`.def` messages under `Core/Inc/can/messages/`. **Keep both sides in sync.**

### FDCAN3 — AMI + steering (local DV peripherals)
| ID | Dir | Peer repo |
|---|---|---|
| `0x503` | RX | mission select — `IFS08-DV_AMI` (⚠️ only TX'd on *confirm*, not dial position) |
| `0x50A` | TX | mission ACK → AMI |
| `0x100` | TX | ASSI status byte → ASSI node (Arduino bridge) |
| `0x2B0` | RX | steering sensor (LWS) |
| `0x520/0x521/0x522` | TX | steering motor / angle / calib — `IFS08-DV-STEERING` |
| `0x528/0x529` | RX | steering feedback / calib status |

The `0x520-0x529` block sits **above** the AMI IDs so steering always loses arbitration to the
AMI — **both boards must be flashed together** if renumbered.

### micro-ROS (USB CDC) — the `IFS08-DV-PIPELINE` contract
Mirror in `Core/Inc/dv_interface.h`; full detail in `Handover/PIPELINE_INTERFACE.md`.
- **Publish:** `/imu` (400 Hz, canonical on both sides), `/as_state`, `/assi/state` (heartbeat),
  `/ami/mission`, `/steering_angle`, `/motor_rpm`, `/res/status`, `/res/go`, `/debug`.
- **Subscribe:** `/dv/status` (pipeline lifecycle byte, incl. `STOPPING=7`), `/ctrl/cmd` (Twist).
- **Serve:** `/force_ebs`, `/activate_steering`.
- Both status bytes are mutual heartbeats; a stale one drops each side to a safe state within
  the FS-Rules T11.9.4 500 ms bound.

### Telemetry — `can-flasher` / **MingoCAN**
The pit-diag decoder & viewer is developed in **`isc-fs/can-flasher`** (MingoCAN VS Code
extension). `isc-fs/iskapps` is **releases only**. pit-diag frames: `0x7A0` status, `0x7A1` RES,
`0x7A2` pipe, `0x7A3` health, `0x7A4` fwinfo, `0x7A5` CAN-health, `0x7A6` calib, `0x7A7` steer,
`0x7A8` calib-dbg, `0x7A9` EBS pressures, **`0x7AA` AS-Emergency cause (new, #192)**.

---

## 5. Open work to carry forward

### Cross-repo coordination (open issues on peer repos)
- **`IFS08-CE-ECU#127`** — TS-active `0x504`: semantics confirmed; **joint bench still owed**
  (nobody has eyeballed `0x504` on a real bus).
- **`IFS08-CE-ECU#142`** — AS-Emergency buzzer via the RTDS buzzer. **uDV side (`0x513`) is ON
  HOLD** until the ECU blesses the ID. Spec: 3.3 Hz / 50 % / 10 s, edge-triggered, coincident
  with the blue flash. The trigger backend already exists (the 0x7AA emergency-reason latch).
- **`IFS08-CE-ECU#132`** — priority TX path for the safety cyclics.
- **`can-flasher#490`** — MingoCAN: decode the new telemetry (stub bits, `dv/status=STOPPING`,
  and the `0x7AA` emergency reason — the `0x7AA` frame is already emitting).

### Pre-track / before an on-car run (from `Handover/PIPELINE_INTERFACE.md`)
- **Rebuild `libmicroros.a`** — the committed lib was built with `PUBLISHERS=10 / SERVICES=1`;
  `colcon.meta` is bumped to `20 / 2` but the lib must be rebuilt + reflashed or the 2nd service
  (`force_ebs`) and 11th/12th publishers silently don't register. *(Correctness, not braking —
  the emergency path is the `dv/status=EMERGENCY` subscriber, unaffected.)*
- **Steering commissioning (#71)** — real full-lock angle, ratio, sign on the car.
- **Rules checks** — 5 s Ready dwell vs Driving transition; mission-selected precondition; AS
  Finished braking. All flagged in `Handover/PIPELINE_INTERFACE.md`.
- **DV_STOPPING scrutineering** — the end-of-mission EBS stop (`/dv/status=7`) needs a
  scrutineering sign-off before it drives; inert until the pipeline emits byte 7, and
  `BENCH_STUB_DV_STOPPING=1` makes it a no-op.

---

## 6. Build & flash

```bash
# Clone WITH submodules (required — libmicroros lives there)
git clone --recurse-submodules -b dev git@github.com:isc-fs/<new-repo>.git

make                 # FLIGHT build (all stubs 0) -> build/binaries/uDV.{elf,hex,bin}
make BENCH="-DBENCH_STUB_TS=1 -DBENCH_STUB_DVPC=1"   # bench build via -D overrides (no source edit)
make clean
```

- Toolchain: `arm-none-eabi-gcc` (auto-detected on macOS; override `GCC_PATH=`).
- **CMake** is kept in sync for IDE/clangd (`build/binaries/compile_commands.json`); Make is
  production.
- **Flash** `build/binaries/uDV.hex` (or `.bin`) with STM32CubeProgrammer / OpenOCD.
- **Bench stubs**: every `BENCH_STUB_*` defaults to 0 (flight-clean) in `Core/Inc/bench_stubs.h`;
  a `-D` wins. Any stub on announces itself on `/debug` at boot and in the pit-diag stub mask, so
  a stubbed image can never pass as flight. New stubs this cycle: `BENCH_STUB_TS` (0x40),
  `BENCH_STUB_DV_STOPPING` (0x80).

---

## 7. Testing — read this, it's the safety backbone

The whole architecture is shaped around **making the safety-critical logic testable off-target**.
The AS state machine, the EBS/SDC actuation, the DV_STOPPING latch, the mission bodies, and the
watchdog are all extracted into **pure functions with no HAL / RTOS / ROS dependency**, so they
run and are exhaustively swept on a laptop with no board. **Treat the test suite as part of the
firmware, not an optional extra** — a change that breaks a host test is a change that breaks the
car.

### The suite: `tests/host/` — `make test`

```bash
cd tests/host && make test     # builds + runs all 10 suites; exits non-zero on any failure
```

Host-native (plain `c++`/`cc`, no ARM, no HAL, no scheduler — RTOS/HAL headers are shadowed by
`tests/host/stubs/` + `tests/host/mocks/`). **~149,637 checks, 0 failures** at handover.

| Suite | Covers |
|---|---|
| `build_integrity` | Makefile↔CMake source parity, orphan/missing sources, `extern "C"` task entry points — catches link-breaking build drift the logic suites can't see |
| `test_safety_eval` | IWDG two-tier watchdog stall detection (`safety_eval.c`) |
| `test_state_machine` | AS state machine + EBS init sequence — links the **real** `state_manager.cpp`/`ebs_manager.cpp` against a controllable `hardware_io` stub |
| `test_as_transition` | **The exhaustive AS-transition sweep** (`as_transition.hpp`) — every reachable combination of inputs × prev-state. ~147k of the total checks. **This is the safety core's guard.** |
| `test_as_actuation` | pure EBS-release / SDC-open decision per AS state (`as_actuation.hpp`) |
| `test_as_stop_latch` | DV_STOPPING debounce (3 msgs) + sticky latch (`as_stop_latch.hpp`) |
| `test_missions` | mission registry + each mission body (inspection sweep, pipeline relay, EBS-test) |
| `test_rules_compliance` | FS-Rules timing-budget `static_assert`s |
| `test_attitude` / `test_bmi088_convert` | complementary-filter roll/pitch + BMI088 raw→scaled |

### The rule for adding safety logic
Any new **emergency trigger** or **actuation rule** must land as (a) a named term in the pure
function, (b) a case in the exhaustive sweep, and (c) — if it's an emergency — an
`AsEmergencyReason` enum value + `0x7AA` telemetry entry (or the cause is a mystery on the bench).
This is how the ASB trip, DV_STOPPING, finish-at-standstill, and the emergency-cause telemetry
were all added this cycle.

### CI runs the host tests (added this cycle) — but make it a *required* check
`.github/workflows/host-tests.yml` runs `make -C tests/host test` on every push (main/dev/feat/fix)
and PR (ubuntu, no ARM toolchain, ~seconds). It was added this cycle to close the gap where
`build.yml` only *built* the firmware and the ~149k-check safety suite never ran in CI.
**One step remains: promote `host-tests / host unit tests (make test)` to a REQUIRED status check**
in the new repo's branch protection for `dev` and `main`, so a red suite actually *blocks* the
merge instead of just showing red. The workflow + job name are deliberately distinct so this is a
one-line branch-protection edit. Carry `host-tests.yml` over with the rest of `.github/`.

Tests are **fully version-controlled** (all 10 sources + `mocks/` + `stubs/` + `Makefile`); only
the compiled binaries are gitignored — so they travel with any git transfer (§1).

---

## 8. ⚠️ Gotchas that silently break the car

1. **CubeMX regen re-applies must-fix values.** After any `.ioc` regeneration:
   - **FDCAN message-RAM offsets** (`Core/Src/fdcan.c`) MUST be `FDCAN1=0 / FDCAN2=640 /
     FDCAN3=1280`. CubeMX resets all three to 0, which overlaps their FIFOs — starting FDCAN2
     then clobbers FDCAN1's RX FIFO0 and the uDV goes **RES-deaf** (no e-stop/GO). Verify every
     time.
   - **Clock tree stays 528 MHz** (VOS0, PLLN=44, HPRE=DIV2). TIM2's 400 Hz IMU tick and the
     FDCAN bit timing depend on it.
   - **USART10** (ASSI Arduino bridge, PG11/PG12) must stay in the `.ioc` or the build loses
     `HAL_UART_MODULE_ENABLED`.
   - Only edit inside `/* USER CODE BEGIN/END */` blocks.
2. **Clone with `--recurse-submodules`** or the build has no `libmicroros.a`.
3. **RAM_D1 BSS** is manually zeroed in `main.c` (startup only zeros DTCMRAM). Don't remove it.
4. **`defaultTask` needs ≥ 12 KB stack** for micro-ROS `rclc_support_init`.
5. **Rebuild libmicroros** (Docker) only when ROS message types change — and reflash.
6. **RES e-stop is ACTIVE-LOW** (undocumented on the box).

---

## 9. Validation state at handover

| | State |
|---|---|
| Host test suite (10 suites) | ✅ ~149,637 checks, 0 failures |
| Flight ARM build | ✅ clean |
| Bench-stub ARM builds | ✅ clean (both branches always compile) |
| **On-car: TS-from-CAN `0x504`** | ⏳ ECU semantics confirmed; **joint bench owed** |
| **On-car: AS-Emergency buzzer** | ⏳ ECU#142 open; uDV `0x513` on hold |
| **On-car: DV_STOPPING** | ⏳ inert until pipeline emits byte 7; **scrutineering owed** |
| **libmicroros caps** | ⏳ rebuild+reflash owed (2nd service / >10 publishers) |

**Nothing here is flashed-and-proven on the car for the new-this-cycle features** — treat the
⏳ rows as the pre-track checklist.

---

## 10. Key decisions & rationale (season memory)

Carry these — they're the "why" behind the code and easy to accidentally undo:

- **The EBS is NOT a service brake.** `DV_STOPPING` (`/dv/status=7`) may only brake to a
  standstill at end-of-mission so AS Finished is reachable — never to modulate speed. The EBS is
  binary. Debounced (3 consecutive msgs) + sticky-latched (never releases mid-run).
- **TS-active comes from the ECU (`0x504`), not the TSMS pin.** The pin measured switch position,
  not tractive-system state — an AMS-initiated AIR-open mid-run was invisible. Read via
  `can_ts_active_fresh()`; stale/never-seen ⇒ off (fail-safe).
- **ASB stored energy is a live emergency input.** Either air tank < 3 bar while armed ⇒ AS
  Emergency (debounced 100 ms). A completed mission that finishes at standstill still reports
  FINISHED even with low tanks (finish wins over the ASB trip at standstill).
- **The AS FSM stays pure & exhaustively host-tested.** Any new emergency trigger goes in
  `as_next_state` as a named OR-term (see the `armed` / `mission_finishing` locals) AND gets the
  reason enum + `0x7AA` telemetry entry (`AsEmergencyReason`), or an Emergency becomes a mystery
  on the bench.
- **Flight-clean discipline.** dev never carries a stub on; bench = `-D` override or a throwaway
  `bench/*` branch.

---

*Prepared at season handover. Questions on any contract → the peer repo's open issues linked in
§4–5, or the `Handover/` technical docs.*
