/**
 * @file    hardware_io.c
 * @brief   Hardware I/O abstraction implementation
 */

#include "hardware_io.h"
#include "gpio.h"
#include "adc.h"
#include "main.h"
#include "iwdg.h"

/* ADC conversion constants */
static const uint32_t adc_digital_threshold = 2048U;

/* --- Festo SPAN air-tank pressure sensors (PRES_1/2_IN -> A5 ch10 / A4 ch6) ---
 * Output configured 1-5 V over 0-10 bar, linear: 1 V = 0 bar, 5 V = 10 bar
 * (SPAN datasheet, docs/SPAN_operating-instr_*.pdf). The 1-5 V swing is above
 * the 3.3 V ADC rail, so PRES_x_IN is divided on the PCB before the ADC pin.
 * PRES_DIVIDER = Vsensor / Vadc (the inverse divider gain). Per
 * 06_MicroDV/PCB DV.kicad_sch: R1/R3 = 10k series from the sensor to the ADC
 * tap, R30/R31 = 1k tap-to-GND (was 5k) => Vadc = Vsensor·1k/(10k+1k) =
 * Vsensor/11, so PRES_DIVIDER = (10k+1k)/1k = 11. (5 V full-scale -> 0.45 V at
 * the pin; the 1-5 V sensor now uses only ~0.09-0.45 V of the ADC range.) */
static const float ADC_VREF_V     = 3.3f;     /* ADC3 reference                */
static const float ADC_FULL_SCALE = 4095.0f;  /* 12-bit                        */
static const float PRES_DIVIDER   = 11.0f;    /* 10k/1k divider: Vadc = Vsensor/11 */
static const float SPAN_V_AT_ZERO = 1.0f;     /* sensor output at 0 bar       */
static const float SPAN_BAR_PER_V = 2.5f;     /* 10 bar / (5 V - 1 V)         */

static uint32_t hardware_io_read_adc_raw(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    /* Max sample time (640.5 cyc). ADC3 runs at the full kernel clock
     * (ADC_CLOCK_ASYNC_DIV1), and every channel here is high-impedance: the
     * pressure taps sit behind a 10k/1k divider (~0.9k Thevenin), the digital
     * levels behind their own dividers. The old 2.5-cycle sample was far too
     * short to charge the sample cap through that, so each conversion kept
     * residual charge from the PREVIOUS channel -> ch10/ch6 read low/high for
     * the SAME pressure (0.5 vs 6.7 bar at a true 5 bar). All these signals are
     * DC, so a long sample costs nothing. */
    sConfig.SamplingTime = ADC3_SAMPLETIME_640CYCLES_5;
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
    // NOTE: A1 -> ADC3_INP3 (ch3) physically carries RES_1_IN on the PCB, not an
    // "SDC ready" signal. Currently unused. See docs/STATE_MACHINE_INPUTS.md
    return hardware_io_read_adc_level(ADC_CHANNEL_3);
}

bool hardware_io_read_asms_on(void)
{
    // ASMS main switch wired to A3 -> ADC3_INP2 -> ADC channel 2
    return hardware_io_read_adc_level(ADC_CHANNEL_2);
}

bool hardware_io_read_tsms_on(void)
{
    // TSMS (Tractive System Master Switch) wired to A6 -> ADC3_INP11 -> ADC channel 11
    return hardware_io_read_adc_level(ADC_CHANNEL_11);
}

bool hardware_io_read_sdc_res_open(void)
{
    // NOTE: A2 -> ADC3_INP7 (ch7) physically carries GO_RES (the RES go signal) on
    // the PCB, not "SDC_RES_OPEN". Currently unused. See docs/STATE_MACHINE_INPUTS.md
    return hardware_io_read_adc_level(ADC_CHANNEL_7);
}

/* Analog Inputs (Pressure Sensors via ADC) */

/* Raw ADC count -> tank pressure (bar) for the SPAN 1-5 V / 0-10 bar sensor
 * behind the PCB divider. Sub-1 V (open / short / empty line) yields a negative
 * result and is clamped to 0 bar, so a disconnected sensor reads 0 and correctly
 * FAILS the >1 bar CheckPressure gate (the old scale=1.0 passed on any noise). */
static float hardware_io_adc_to_bar(uint32_t raw)
{
    const float v_adc    = ((float)raw / ADC_FULL_SCALE) * ADC_VREF_V;
    const float v_sensor = v_adc * PRES_DIVIDER;
    const float bar      = (v_sensor - SPAN_V_AT_ZERO) * SPAN_BAR_PER_V;
    return (bar < 0.0f) ? 0.0f : bar;
}

float hardware_io_read_actuator1_storage_pressure(void)
{
    // Actuator1 -> A5 -> ADC3_INP10 -> ADC channel 10
    return hardware_io_adc_to_bar(hardware_io_read_adc_raw(ADC_CHANNEL_10));
}

float hardware_io_read_actuator2_storage_pressure(void)
{
    // Actuator2 -> A4 -> ADC3_INP6 -> ADC channel 6
    return hardware_io_adc_to_bar(hardware_io_read_adc_raw(ADC_CHANNEL_6));
}

/* Time Helpers */

uint32_t hardware_io_now_ms(void)
{
    return HAL_GetTick();
}

void hardware_io_watchdog_kick(void)
{
    /* The watchdog is the hardware IWDG (see iwdg.c / safety_monitor.c),
     * NOT the old TIM3 software timer. Refresh the IWDG so app_task's
     * loop counts as a liveness source. (The safety task is the primary
     * refresher; this is harmless belt-and-suspenders.) */
    iwdg_refresh();
}

bool hardware_io_is_ebs_active(void)
{
    GPIO_PinState p1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);
    GPIO_PinState p2 = HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin);
    /* EBS polarity (confirmed): actuators are driven LOW to FIRE the brake,
     * HIGH to release it. Fail-safe — LOW is the power-on/reset level
     * (gpio.c inits D1/D2 LOW), so a reset or power loss lands braked;
     * matches ebs_manager's activate(LOW)/deactivate(HIGH). The brake is
     * "active" when EITHER channel reads LOW. */
    return (p1 == GPIO_PIN_RESET) || (p2 == GPIO_PIN_RESET);
}
