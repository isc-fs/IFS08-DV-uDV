/**
 * @file    hardware_io.c
 * @brief   Hardware I/O abstraction implementation
 */

#include "hardware_io.h"
#include "gpio.h"
#include "adc.h"
#include "main.h"

/* ADC conversion buffer and calibration constants */
static float adc_scale_factor = 1.0f;  // Can be calibrated based on ADC reference

void hardware_io_init(void)
{
    // GPIO already initialized by MX_GPIO_Init()
    // ADC already initialized by MX_ADC1_Init()
    // Watchdog can be initialized here if needed
}

/* Digital Outputs */

void hardware_io_set_as_close_sdc(bool on)
{
    // Example: AS_CLOSE_SDC pin control
    // Adjust GPIO port/pin based on your STM32 project
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hardware_io_enable_ebs_actuator_1(bool enable)
{
    // EBS Actuator 1 control
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hardware_io_enable_ebs_actuator_2(bool enable)
{
    // EBS Actuator 2 control
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hardware_io_toggle_watchdog(void)
{
    // Watchdog toggle (independent watchdog or external circuit)
    // Example: Toggle a GPIO pin
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
}

/* Digital Inputs */

bool hardware_io_read_sdc_is_ready(void)
{
    // SDC (Shutdown Circuit) ready signal
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    return state == GPIO_PIN_SET;
}

bool hardware_io_read_ts_activated(void)
{
    // Tractive System active signal
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    return state == GPIO_PIN_SET;
}

bool hardware_io_read_asms_on(void)
{
    // Autonomous System Management System on signal
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    return state == GPIO_PIN_SET;
}

bool hardware_io_read_sdc_res_open(void)
{
    // SDC Residual signal (resistor network open/closed detection)
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    return state == GPIO_PIN_SET;
}

/* Analog Inputs (Pressure Sensors via ADC) */

float hardware_io_read_main_storage_pressure(void)
{
    // Read main storage pressure from ADC
    // Scaling: ADC value to bar conversion based on sensor calibration
    uint32_t adc_val = HAL_ADC_GetValue(&hadc3);
    return (float)adc_val * adc_scale_factor;
}

float hardware_io_read_actuator1_storage_pressure(void)
{
    // Read actuator 1 storage pressure
    uint32_t adc_val = HAL_ADC_GetValue(&hadc3);
    return (float)adc_val * adc_scale_factor;
}

float hardware_io_read_actuator2_storage_pressure(void)
{
    // Read actuator 2 storage pressure
    uint32_t adc_val = HAL_ADC_GetValue(&hadc3);
    return (float)adc_val * adc_scale_factor;
}

float hardware_io_read_brake_pressure(void)
{
    // Read brake line pressure
    uint32_t adc_val = HAL_ADC_GetValue(&hadc3);
    return (float)adc_val * adc_scale_factor;
}

/* Time Helpers */

uint32_t hardware_io_now_ms(void)
{
    return HAL_GetTick();
}
