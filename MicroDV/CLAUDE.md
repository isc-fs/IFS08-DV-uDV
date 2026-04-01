# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MicroDV is embedded firmware for an **STM32H733ZGT6** microcontroller (ARM Cortex-M7 @ 384 MHz), part of a driverless vehicle project. It handles CAN bus communication, USB CDC (virtual COM), SPI radio (NRF24L01), I2C sensors, ADC, and addressable LEDs (WS2812).

## Build Commands

Requires `arm-none-eabi-gcc` (STM32 GNU Tools v14.3.1+st.2) and CMake 3.22+ with Ninja.

```bash
# Configure
cmake --preset Debug
cmake --preset Release

# Build
cmake --build build/Debug
cmake --build build/Release
```

Output: `build/Debug/MicroDV.elf`

Flash via STM32CubeProgrammer (USB) or OpenOCD (SWD on PA13/PA14).

There is no test framework or lint toolchain configured.

## Architecture

### Code Ownership Boundaries

| Path | Owner | Editable? |
|---|---|---|
| `Core/Src/`, `Core/Inc/` | User application code | Yes |
| `USB_DEVICE/App/` | CubeMX-generated, user callbacks | Yes (in USER CODE blocks) |
| `Drivers/`, `Middlewares/`, `USB_DEVICE/Target/` | ST HAL + middleware | No — auto-generated |
| `cmake/stm32cubemx/CMakeLists.txt` | CubeMX-generated build config | No |
| `startup_stm32h733xx.s`, `STM32H733XG_FLASH.ld` | CubeMX-generated | No |

### User Application Files

- **`Core/Src/main.c`** — Peripheral init (via HAL), main loop, `_write()` redirect (printf → USB CDC)
- **`Core/Src/i2c_lcd.c`** / **`Core/Inc/i2c_lcd.h`** — Custom I2C LCD driver (`lcd_init`, `lcd_puts`, `lcd_gotoxy`, etc.)
- **`Core/Src/ws2812.c`** — WS2812 RGB LED driver via SPI1 DMA (`WS2812_Init`, `WS2812_SetLED`, `WS2812_Show`)
- **`Core/Src/stm32h7xx_it.c`** — Interrupt handlers

### STM32CubeMX Regeneration

The `.ioc` file (`MicroDV.ioc`) is the hardware configuration source. When regenerating code from CubeMX:
- All code **outside** `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` blocks is overwritten.
- Custom source files (`ws2812.c`, etc.) must be **manually re-added** to `CMakeLists.txt` after regeneration.

### Key Peripherals

| Peripheral | Pins | Purpose |
|---|---|---|
| FDCAN1/2/3 | — | CAN bus @ 500 kbps |
| SPI1 | PA5/PA6/PA7 | NRF24L01 radio |
| I2C2 | PF0/PF1 | IMU sensor |
| ADC3 | PF7–10, PC0–2 | Analog inputs A1–A7 |
| USB OTG HS | — | Virtual COM port (CDC) |
| TIM16 | — | Periodic interrupt ~36 Hz |
| CORDIC | — | Hardware math accelerator |

### Clock Summary

- HSE: 24 MHz → CPU: 384 MHz, AHB/AXI: 192 MHz, APB1/2/3/4: 96 MHz
- FDCAN clocked from HSE (24 MHz); USB from HSI48 (48 MHz)

### CMake Structure

- `CMakeLists.txt` — Top-level, user-editable; add custom sources here
- `CMakePresets.json` — Debug/Release presets using Ninja
- `cmake/gcc-arm-none-eabi.cmake` — Toolchain file; sets Cortex-M7 flags (`-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard`)
- `cmake/stm32cubemx/CMakeLists.txt` — Auto-generated; defines `STM32_Drivers` and `USB_Device_Library` object libraries

### clangd / IntelliSense

Configured via `.clangd` and `.vscode/c_cpp_properties.json` using `build/Debug/compile_commands.json`. The VSCode extension uses `stm32cube-ide-clangd` as the language server.
