/**
 ******************************************************************************
 * @file    assi.c
 * @brief   WS2812 ASSI driver on D5/PB8 (bit-banged, DWT-timed).
 *
 * See assi.h. Timing targets WS2812B: T0H 400ns, T1H 800ns, bit period
 * 1250ns, reset >50us. Cycle counts are derived from the real SystemCoreClock
 * at init so the driver survives a clock-tree change.
 ******************************************************************************
 */
#include "assi.h"
#include <string.h>

#define ASSI_PORT   D5_GPIO_Port
#define ASSI_PIN    D5_Pin

/* GRB pixel buffer (WS2812 wire order). */
static uint8_t pixels[ASSI_NUM_LEDS][3];

/* Bit timings in CPU cycles (filled in assi_init from SystemCoreClock). */
static uint32_t t0h_cyc;      /* '0' high time  (~400 ns) */
static uint32_t t1h_cyc;      /* '1' high time  (~800 ns) */
static uint32_t period_cyc;   /* full bit period (~1250 ns) */
static uint32_t reset_cyc;    /* latch/reset low (~60 us) */

static inline uint32_t ns_to_cyc(uint32_t ns)
{
    return (uint32_t)(((uint64_t)SystemCoreClock * ns) / 1000000000ull);
}

static inline void delay_from(uint32_t start, uint32_t cycles)
{
    while ((DWT->CYCCNT - start) < cycles) { /* busy-wait */ }
}

void assi_init(void)
{
    /* DWT cycle counter (also used by dwt_micros; enable defensively). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* CubeMX generates D5/PB8 at LOW speed (shared init with the EBS relay
     * lines D1/D2, which must stay slow). Re-init only this pin at high speed
     * for clean WS2812 edges; does not touch the .ioc or the shared block. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = ASSI_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ASSI_PORT, &gpio);

    t0h_cyc    = ns_to_cyc(400);
    t1h_cyc    = ns_to_cyc(800);
    period_cyc = ns_to_cyc(1250);
    reset_cyc  = ns_to_cyc(60000);

    memset(pixels, 0, sizeof(pixels));
    HAL_GPIO_WritePin(ASSI_PORT, ASSI_PIN, GPIO_PIN_RESET);
    assi_show();
}

void assi_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < ASSI_NUM_LEDS; i++)
    {
        pixels[i][0] = g;
        pixels[i][1] = r;
        pixels[i][2] = b;
    }
}

void assi_show(void)
{
    const uint32_t set_mask   = ASSI_PIN;                 /* BSRR: drive high */
    const uint32_t reset_mask = (uint32_t)ASSI_PIN << 16; /* BSRR: drive low  */

    /* The WS2812 protocol has no clock — a gap longer than the reset window
     * latches the frame, so the whole chain must clock out without preemption. */
    __disable_irq();
    for (uint16_t led = 0; led < ASSI_NUM_LEDS; led++)
    {
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            uint8_t byte = pixels[led][ch];
            for (int8_t bit = 7; bit >= 0; bit--)
            {
                uint32_t high = (byte & (1u << bit)) ? t1h_cyc : t0h_cyc;
                uint32_t start = DWT->CYCCNT;
                ASSI_PORT->BSRR = set_mask;
                delay_from(start, high);
                ASSI_PORT->BSRR = reset_mask;
                delay_from(start, period_cyc);
            }
        }
    }
    __enable_irq();

    /* Reset/latch: hold low for >50 us. */
    ASSI_PORT->BSRR = reset_mask;
    delay_from(DWT->CYCCNT, reset_cyc);
}

void assi_off(void)
{
    assi_set_all(0, 0, 0);
    assi_show();
}
