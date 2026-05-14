#
# Host-side unit tests for IFS08-DV-uDV
#
# Builds with the host compiler (NOT arm-none-eabi). HAL is stubbed in
# tests/mocks/. Run with: cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
#
cmake_minimum_required(VERSION 3.16)
project(uDV-tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Strict-ish warnings for the test build only (we don't change firmware flags).
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# Source root sits one directory up.
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}                    # test.h
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks               # stm32h7xx_hal.h, main.h shims
  ${REPO_ROOT}/Core/Inc                           # production headers
)

# Shared HAL stub library — linked into every test executable.
add_library(hal_mock STATIC
  mocks/hal_stubs.c
  mocks/i2c_utils_stubs.c)
target_include_directories(hal_mock PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/mocks
  ${REPO_ROOT}/Core/Inc)

enable_testing()

#
# test_bmi088_convert
#
add_executable(test_bmi088_convert
  test_bmi088_convert.c
  ${REPO_ROOT}/Core/Src/bmi088.c)
target_link_libraries(test_bmi088_convert PRIVATE hal_mock)
add_test(NAME bmi088_convert COMMAND test_bmi088_convert)

#
# test_attitude
#
add_executable(test_attitude
  test_attitude.c
  ${REPO_ROOT}/Core/Src/attitude.c)
target_link_libraries(test_attitude PRIVATE hal_mock m)
add_test(NAME attitude COMMAND test_attitude)

#
# test_ws2812
#
add_executable(test_ws2812
  test_ws2812.c
  ${REPO_ROOT}/Core/Src/ws2812.c)
target_link_libraries(test_ws2812 PRIVATE hal_mock)
add_test(NAME ws2812 COMMAND test_ws2812)
