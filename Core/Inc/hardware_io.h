/**
 * @file    hardware_io.h
 * @brief   Hardware I/O abstraction layer
 * @details Wraps STM32 HAL calls for GPIO, ADC, and watchdog
 */

#ifndef INC_HARDWARE_IO_H_
#define INC_HARDWARE_IO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Digital outputs
 */
void hardware_io_set_as_close_sdc(bool on);
void hardware_io_enable_ebs_actuator_1(bool enable);
void hardware_io_enable_ebs_actuator_2(bool enable);

/*
 * Readbacks for outputs (useful when app stalled):
 * - `hardware_io_is_ebs_active()` returns true when any EBS actuator indicates braking active
 */
bool hardware_io_is_ebs_active(void);

/**
 * @brief Digital inputs (sensors/signals)
 */
bool hardware_io_read_sdc_is_ready(void);
bool hardware_io_read_asms_on(void);
bool hardware_io_read_tsms_on(void);
bool hardware_io_read_sdc_res_open(void);

/**
 * @brief Analog inputs (pressure sensors via ADC)
 * @return Pressure in bar
 */
float hardware_io_read_actuator1_storage_pressure(void);
float hardware_io_read_actuator2_storage_pressure(void);

/**
 * @brief Time helper
 * @return Milliseconds since boot (from HAL_GetTick)
 */
uint32_t hardware_io_now_ms(void);

/**
 * @brief Watchdog kick/refresh (call regularly to prevent watchdog timeout)
 *        Resets the hardware timer countdown to prevent ISR from triggering
 */
void hardware_io_watchdog_kick(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_HARDWARE_IO_H_ */
