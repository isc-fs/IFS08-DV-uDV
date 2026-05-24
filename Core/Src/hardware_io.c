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
static const uint32_t adc_digital_threshold = 2048U;

static uint32_t hardware_io_read_adc_raw(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;

    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_Start(&hadc3) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc3, 10U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc3);
        return 0U;
    }

    uint32_t adc_value = HAL_ADC_GetValue(&hadc3);
    (void)HAL_ADC_Stop(&hadc3);
    return adc_value;
}

static bool hardware_io_read_adc_level(uint32_t channel)
{
    return hardware_io_read_adc_raw(channel) >= adc_digital_threshold;
}

/* Digital Outputs */

void hardware_io_set_as_close_sdc(bool on)
{
    HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hardware_io_enable_ebs_actuator_1(bool enable)
{
    HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void hardware_io_enable_ebs_actuator_2(bool enable)
{
    HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Digital Inputs */

bool hardware_io_read_sdc_is_ready(void)
{
    // SDC ready is wired to A1 -> ADC3_INP3 -> ADC channel 3
    return hardware_io_read_adc_level(ADC_CHANNEL_3);
}

bool hardware_io_read_asms_on(void)
{
    // ASMS main switch wired to A3 -> ADC3_INP2 -> ADC channel 2
    return hardware_io_read_adc_level(ADC_CHANNEL_2);
}

bool hardware_io_read_sdc_res_open(void)
{
    /* TODO: No local pin is assigned for sdc_res_open yet. */
    return false;
}

bool hardware_io_read_go_res(void)
{
    // GO_RES wired to A2 -> ADC3_INP7 -> ADC channel 7
    return hardware_io_read_adc_level(ADC_CHANNEL_7);
}

/* Analog Inputs (Pressure Sensors via ADC) */

float hardware_io_read_actuator1_storage_pressure(void)
{
    // Actuator1 -> A5 -> ADC3_INP10 -> ADC channel 10
    uint32_t adc_val = hardware_io_read_adc_raw(ADC_CHANNEL_10);
    return (float)adc_val * adc_scale_factor;
}

float hardware_io_read_actuator2_storage_pressure(void)
{
    // Actuator2 -> A4 -> ADC3_INP6 -> ADC channel 6
    uint32_t adc_val = hardware_io_read_adc_raw(ADC_CHANNEL_6);
    return (float)adc_val * adc_scale_factor;
}

/* Time Helpers */

uint32_t hardware_io_now_ms(void)
{
    return HAL_GetTick();
}

// Timer handle (provided by CubeMX-generated tim.c)
extern TIM_HandleTypeDef htim3;  // TIM3 configured as watchdog timer

void hardware_io_watchdog_kick(void)
{
    // Reset the timer counter to start from ARR again
    // This prevents the timer from expiring
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

bool hardware_io_is_ebs_active(void)
{
    GPIO_PinState p1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);
    GPIO_PinState p2 = HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin);
    /* EBS actuators are driven low to activate braking */
    return (p1 == GPIO_PIN_RESET) || (p2 == GPIO_PIN_RESET);
}
