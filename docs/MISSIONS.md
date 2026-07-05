# Mission dispatch (uDV)

How the firmware turns the AMI-selected mission into actuation, and **how to add
a new mission**. The mission layer sits between the AS state machine
([app_task.cpp](../Core/Src/app_task.cpp)) and the CAN actuators; the state
machine itself ([as_transition.hpp](../Core/Inc/as_transition.hpp)) is
mission-agnostic.

## Model

The AMI board transmits the 0-based mission menu index on CAN `0x503` byte[0]
(see `missions[]` in the `IFS08-DV_AMI` repo). `app_task` resolves that code to a
`const Mission*` via `mission_for_code()` **once per tick** and drives it through
a small vtable ([mission.h](../Core/Inc/mission.h)). Each mission's control law
lives in its own translation unit; the registry
([mission_registry.cpp](../Core/Src/mission_registry.cpp)) is the single place
that maps a code to a mission.

A mission is **pure**, the same discipline as `as_transition.hpp`: its `on_tick`
consumes an immutable `MissionCtx` snapshot and **returns** a `MissionCommand`
describing what to actuate. `app_task` builds the snapshot and applies the
command over CAN — a mission never touches the CAN driver, the RTOS, or the ROS
globals. That is what makes every mission host-unit-testable with **no** HAL /
CAN / RTOS stubs (see [tests/host/test_missions.cpp](../tests/host/test_missions.cpp)).

### The vtable

```c
struct Mission {
    const char*    name;
    bool           needs_pipeline;                       // data, not a call (see below)
    void           (*on_enter)(const MissionCtx*);        // GO edge; may be nullptr
    MissionCommand (*on_tick )(const MissionCtx*);        // every DRIVING tick; may be nullptr
    bool           (*is_complete)(const MissionCtx*);     // polled each tick; may be nullptr
    void           (*on_exit )(const MissionCtx*);        // ->FINISHED edge; may be nullptr
};
```

| Type | Fields |
|------|--------|
| `MissionCtx` (in) | `now_ms`, `mission_elapsed_ms` (since the GO edge), `ctrl_cmd_fresh`, `ctrl_accel`, `ctrl_steer` (latest normalised `/ctrl/cmd`) |
| `MissionCommand` (out) | `send_steer_angle` + `steer_angle_deg` (0x020 angle), **or** `send_drive` + `accel_norm` + `steer_norm` (normalised accel/steer). Default-init = emit nothing. A mission uses one channel; never both. |

`needs_pipeline` is a **field, not a call**, because the AS transition reads it in
READY to gate GO and to arm the lost-heartbeat rule — it must be known *before*
DRIVING, so it cannot be an `on_tick`-time query.

## Standalone vs pipeline missions

| Kind | `needs_pipeline` | GO gated on | Ends when | Examples |
|------|------------------|-------------|-----------|----------|
| **Pipeline** | `true` | a fresh `dv/status == READY` | pipeline reports `dv/status == FINISHED` (FSM rule; `is_complete` stays `nullptr`) | accel, skidpad, autocross, trackdrive |
| **Standalone** | `false` | RES GO alone (no pipeline) | its own `is_complete` returns true, or an external safe-state exit | inspection, EBS test |

A stale `/dv/status` while driving trips EMERGENCY **only** for a pipeline
mission — a standalone mission has no heartbeat to lose. See
[PIPELINE_INTERFACE.md](PIPELINE_INTERFACE.md) for the `dv/status` contract.

## How the FSM drives a mission

Per tick, `app_task`:

1. resolves `current_mission_id` (defaults to Inspection when none received) and
   `const Mission* mission = mission_for_code(id)`;
2. feeds the transition two mission facts — `mission_needs_pipeline` and
   **`mission_valid`** (`mission != nullptr`);
3. on the **READY→DRIVING (GO) edge**: captures `active_mission = mission`, sends
   the steering-motor start (0x010), (re)starts the mission clock, and calls
   `on_enter`;
4. in **DRIVING**: builds a `MissionCtx`, calls `active_mission->on_tick` and
   applies the returned `MissionCommand`, then polls `is_complete` (which latches
   `mission_complete → FINISHED` next tick);
5. on the **→FINISHED edge**: sends the steering-motor stop (0x010=0) and calls
   `on_exit`.

Two safety properties are enforced here, not in the missions:

- **`mission_valid` gate.** An unknown / SHUTDOWN code (7-9, or ≥10) resolves to
  `nullptr` and **cannot enter DRIVING** — otherwise the car would release the
  brakes (DRIVING is the only state with brakes off) with no mission body
  running. Such a code can still arm to READY, a safe braked state.
- **Run binding (`active_mission`).** The run is bound to the mission captured at
  the GO edge and used for `on_enter`/`on_tick`/`is_complete`/`on_exit` for its
  whole duration. A mid-run change of the selected code (a stray/corrupt `0x503`
  — the AMI latches one mission per power cycle, so it cannot happen normally)
  can neither swap the running mission's actuation nor null it mid-drive.

## Code → mission map

Defined in [mission_registry.cpp](../Core/Src/mission_registry.cpp), indexed by
the `AmiMission` enum ([mission.h](../Core/Inc/mission.h)); order matches the
AMI `missions[]` menu.

| Code | AMI menu | Mission | Kind |
|------|----------|---------|------|
| 0 | manual | `mission_manual` (no-op) | standalone |
| 1 | acceleration | `mission_pipeline` | pipeline |
| 2 | skidpad | `mission_pipeline` | pipeline |
| 3 | autocross | `mission_pipeline` | pipeline |
| 4 | track drive | `mission_pipeline` | pipeline |
| 5 | ebs test | `mission_ebstest` (stub) | standalone |
| 6 | inspection | `mission_inspection` | standalone |
| 7 | shutdown | `nullptr` | — (never drives) |
| 8 / 9 | aux1 / aux2 | `nullptr` | — (never drives) |

The four pipeline missions share one body (`mission_pipeline`) — they differ only
in the menu entry; the uDV-side actuation (relay `/ctrl/cmd`) is identical.

## How to add a new mission

Say you are adding mission code **N** (must match the AMI `missions[]` index).

1. **Write the mission body** — a new `Core/Src/mission_<name>.cpp` (or reuse
   `mission_pipeline` if it is a pipeline mission with identical actuation).
   Implement only the callbacks you need; the rest stay `nullptr`. Keep it pure:
   `#include "mission.h"` and nothing hardware-specific. Define the object with
   **external linkage** so the registry can reference it (a namespace-scope
   `const` is internal by default in C++):

   ```c
   #include "mission.h"

   namespace {
   MissionCommand my_on_tick(const MissionCtx* ctx) {
       MissionCommand cmd = {};                 // default: emit nothing
       // ... set cmd.send_steer_angle / cmd.send_drive + values ...
       return cmd;
   }
   bool my_is_complete(const MissionCtx* ctx) { return ctx->mission_elapsed_ms >= 12000u; }
   } // namespace

   extern const Mission mission_my = {          // extern => external linkage
       "my_mission",  /* name           */
       false,         /* needs_pipeline */       // true only if driven by /ctrl/cmd
       nullptr,       /* on_enter       */
       my_on_tick,    /* on_tick        */
       my_is_complete,/* is_complete    */       // nullptr => never self-finishes
       nullptr,       /* on_exit        */
   };
   ```

   Rules of thumb: a standalone mission that must stop itself needs an
   `is_complete`; a pipeline mission leaves `is_complete == nullptr` (it finishes
   via `dv/status FINISHED`); reset any internal state in `on_enter` (it is
   called on every GO edge). Use exactly one actuation channel per tick.

2. **Add the enum value** in [mission.h](../Core/Inc/mission.h) `AmiMission` if the
   code is newly named.

3. **Register it** in [mission_registry.cpp](../Core/Src/mission_registry.cpp):
   add an `extern const Mission mission_my;` declaration and point `k_by_code[N]`
   at `&mission_my`. (Trivial no-op missions may be defined inline there instead
   of in their own file, like `mission_manual`.)

4. **Add the source to BOTH build manifests** — [Makefile](../Makefile)
   `CPP_SOURCES` and [CMakeLists.txt](../CMakeLists.txt) `set(CPP_SOURCES ...)`.
   They must stay identical; the host suite's `check_build_integrity.sh` fails the
   build if they drift or a `Core/Src/*.cpp` is orphaned. (There is **no**
   CubeIDE `.cproject` in this repo — only these two lists.) `mission.h` is
   header-only and takes no manifest entry.

5. **Test it** — add cases to
   [tests/host/test_missions.cpp](../tests/host/test_missions.cpp) (the mission
   links directly, no stubs) and, if the mission changes the FSM gating, to
   [tests/host/test_as_transition.cpp](../tests/host/test_as_transition.cpp). Run
   `cd tests/host && make` (all suites) and `make` at the repo root (ARM build).

The AS state machine ([as_transition.hpp](../Core/Inc/as_transition.hpp)) and
`app_task`'s dispatch do **not** change to add a mission — only the registry, the
new TU, the two manifests, and the tests.

## Missions today

- **Inspection** (6) — open-loop steering sweep: ±90° at 0.3 Hz, one 0x020 angle
  command per 200 ms, self-finishes after 30 s. The 90° amplitude intentionally
  saturates the steering firmware's ±60° clamp. The first tick of a run always
  commands (independent of the tick-counter value).
- **Pipeline** (1-4, incl. **trackdrive**) — relays the pipeline's latest
  normalised `/ctrl/cmd`; zeros both channels when `/ctrl/cmd` goes stale so a
  dropped link never latches the last command. The mission's `on_tick` runs every
  loop tick, but `app_task` paces the actual torque TX (0x507 to the ECU) to the
  ECU's 20 ms cyclic contract (`TORQUE_TX_PERIOD_MS`); while DRIVING a pipeline
  mission, `app_task` also drives the DV ready-to-drive handshake (0x510 → 0x511)
  until the ECU confirms.
- **EBS test** (5) — **stub**: holds the wheels straight. The real test (drive to
  speed, trigger the EBS, verify deceleration) is an on-car TODO.

### Open items (on-car)

- Trackdrive actuation: the `0x507` torque contract is confirmed against the ECU
  `.def` (int32 percent, paced 20 ms); the `0x508` normalised-steer frame has no
  ECU consumer on the current map (steering is commanded via `0x020`) — confirm
  who reads it or retire it (G3). The `[-1, 1]` values are clamped on receive.
- EBS-test mission body is a hold-straight stub.
