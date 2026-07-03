# Host-side Unit Tests

Pure-logic tests that run on your dev machine (not the STM32). They link
against the actual production `.c` files in `Core/Src/`, with the STM32 HAL
stubbed out by lightweight mocks under `tests/mocks/`. No Docker, no ARM
toolchain, no flashing required.

## What is covered

| Test | Source under test | Scope |
|------|-------------------|-------|
| `test_bmi088_convert` | `Core/Src/bmi088.c` | `bmi088_convert_scaled` scaling math, `bmi088_bind` parameter validation |
| `test_attitude` | `Core/Src/attitude.c` | `attitude_init`, `attitude_set_gyro_bias_dps`, `attitude_roll_deg`/`attitude_pitch_deg` |
| `test_ws2812` | `Core/Src/ws2812.c` | Bit encoding (MSB-first, GRB order), buffer length, mission color lookup, out-of-bounds guards |

What is **not** covered:

- `attitude_update_cordic` — depends on the STM32 CORDIC peripheral for
  hardware atan2. Would need an emulator or a CPU fallback to test on host.
- `bmi088_init_minimal`, `bmi088_read_*`, `i2c_utils_*` — require a richer
  I2C mock (return scripted register reads). Out of scope for the first pass.
- `can_service`, `imu_service` integration paths — same reasoning.
- `freertos.c`, micro-ROS plumbing, USB CDC transport — RTOS / network /
  hardware dependent.

## Build and run

```bash
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

All three test binaries land in `tests/build/`. You can also run them
directly for verbose output:

```bash
./tests/build/test_bmi088_convert
./tests/build/test_attitude
./tests/build/test_ws2812
```

Each prints `PASS`/`FAIL` per test case and exits non-zero if anything fails.

## Mock layout

```
tests/mocks/
├── main.h            — empty shim (Core/Inc/main.h pulls in pin defines we don't need)
├── stm32h7xx_hal.h   — minimal HAL types, function prototypes, mock-state externs
└── hal_stubs.c       — function bodies; HAL_SPI_Transmit captures the buffer for inspection
```

Tests reset mock state via `hal_mock_reset()` before each scenario and
inspect captured side effects through globals such as `hal_mock_spi_buf`,
`hal_mock_spi_len`, `hal_mock_tick`, and `hal_mock_i2c_state`.

## Adding a new test

1. Drop a `test_<name>.c` file in `tests/` with a `main()` that uses the
   helpers in `test.h` (`TEST_BEGIN`, `TEST_CASE`, `TEST_END`, `ASSERT_*`).
2. Add the corresponding `add_executable` + `add_test` block to
   `tests/CMakeLists.txt`. List the production `.c` files it needs.
3. If the production file references a HAL symbol that isn't stubbed yet,
   add a prototype to `tests/mocks/stm32h7xx_hal.h` and a no-op (or
   instrumented) implementation in `tests/mocks/hal_stubs.c`.

## CI

These tests are designed to run anywhere a host C compiler + CMake exists,
so they fit nicely into a GitHub Actions workflow without needing the ARM
toolchain or Docker. Not wired up yet — left as a follow-up.
