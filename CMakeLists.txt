cmake_minimum_required(VERSION 3.22)

# Project
project(uDV C CXX ASM)

# MCU flags
set(MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H733XG_FLASH.ld)

# Compile definitions
add_compile_definitions(
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H733xx
    STM32_THREAD_SAFE_STRATEGY=4
)

# C sources
set(C_SOURCES
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Core/Src/adc.c
    Core/Src/cordic.c
    Core/Src/fdcan.c
    Core/Src/i2c.c
    Core/Src/usart.c
    Core/Src/bmi088.c
    Core/Src/i2c_utils.c
    Core/Src/imu_service.c
    Core/Src/imu_task.c
    Core/Src/ws2812.c
    Core/Src/assi_task.c
    Core/Src/dwt_time.c
    Core/Src/attitude.c
    Core/Src/tim.c
    Core/Src/comunicacion_direccion.c
    # Safety / watchdog + AS state-machine support (keep in sync with Makefile)
    Core/Src/iwdg.c
    Core/Src/safety_eval.c
    Core/Src/safety_monitor.c
    Core/Src/hardware_io.c
    Core/Src/ros_task_commands.c
    Core/Src/ros_task.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/system_stm32h7xx.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cordic.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    # micro-ROS sources
    micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
    micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c
)

# C++ sources
set(CPP_SOURCES
    Core/Src/can_interface.cpp
    Core/Src/can_globals.cpp
    Core/Src/pit_diag.cpp
    Core/Src/can_task.cpp
    # AS state machine (keep in sync with Makefile)
    Core/Src/app_task.cpp
    Core/Src/state_manager.cpp
    Core/Src/ebs_manager.cpp
    Core/Src/ros_globals.cpp
    # Mission dispatch (registry + per-mission bodies; keep in sync with Makefile)
    Core/Src/mission_registry.cpp
    Core/Src/mission_inspection.cpp
    Core/Src/mission_pipeline.cpp
    Core/Src/mission_ebstest.cpp
)

# ASM sources
set(ASM_SOURCES
    startup_stm32h733xx.s
)

# Include directories
set(INCLUDE_DIRS
    Core/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/Include
    # micro-ROS includes (CORREGIDO)
    micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include
)

# Executable
add_executable(${PROJECT_NAME}.elf ${C_SOURCES} ${CPP_SOURCES} ${ASM_SOURCES})
target_include_directories(${PROJECT_NAME}.elf PRIVATE ${INCLUDE_DIRS})

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Og -Wall -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS} -Og -std=c++11 -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections")

# Debug flags
set(CMAKE_C_FLAGS_DEBUG "-g -gdwarf-2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -gdwarf-2")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -specs=nano.specs -T${LINKER_SCRIPT} -lc -lm -Wl,-Map=\"${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map\",--cref -Wl,--gc-sections")

# micro-ROS static library (CORREGIDO)
target_link_libraries(${PROJECT_NAME}.elf
    ${CMAKE_SOURCE_DIR}/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
)

# Generate HEX and BIN
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating HEX and BIN files"
)