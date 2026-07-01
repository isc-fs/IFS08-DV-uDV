#include "ws2812.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * WS2812 bit encoding via SPI MOSI
 * SPI kernel = PLL1Q 96 MHz, prescaler 16 → SPI = 6.0 MHz.
 * 1 SPI bit = 167 ns, 1 SPI byte = 1.33 µs = exactly one WS2812 bit period.
 * Each color bit is encoded as one SPI byte (MSB first):
 *   '1' bit → 0xF8 (11111000): T1H = 5×167 ≈ 833 ns, T1L = 3×167 ≈ 500 ns
 *   '0' bit → 0xC0 (11000000): T0H = 2×167 ≈ 333 ns, T0L = 6×167 ≈ 1000 ns
 * Both high times sit centered in WS2812B spec (T1H 0.8 µs, T0H 0.4 µs, ±150 ns).
 * 40 LEDs × 24 bits = 960 bytes data + 64 bytes reset (64×1.33 µs ≈ 85 µs > 50 µs latch)
 * --------------------------------------------------------------------------- */
#define BITS_PER_LED   24
#define RESET_BYTES    64
#define BUF_SIZE       (WS2812_NUM_LEDS * BITS_PER_LED + RESET_BYTES)

#define WS_BIT_HIGH    0xF8
#define WS_BIT_LOW     0xC0

static SPI_HandleTypeDef *ws_hspi;
static uint8_t ws_buf[BUF_SIZE];

/* GRB pixel buffer */
static uint8_t pixels[WS2812_NUM_LEDS][3];  /* [i][0]=G, [i][1]=R, [i][2]=B */

/* Mission color table (R, G, B) — indexed 0–9 */
static const uint8_t mission_colors[10][3] = {
    {   0, 255,   0 },  /* 0: Manual       — green  */
    { 255, 255,   0 },  /* 1: Acceleration  — yellow */
    {   0,   0, 255 },  /* 2: Skidpad       — blue   */
    { 255,   0,   0 },  /* 3: Autocross     — red    */
    { 128,   0, 128 },  /* 4: Track drive   — purple */
    { 255, 255, 255 },  /* 5: EVS test      — white  */
    {   0, 255, 255 },  /* 6: Inspection    — cyan   */
    { 255, 165,   0 },  /* 7: Shutdown      — orange */
    {   0, 100,   0 },  /* 8: Aux1          — dk grn */
    { 128, 128,   0 },  /* 9: Aux2          — olive  */
};

void ws2812_init(SPI_HandleTypeDef *hspi)
{
    ws_hspi = hspi;
    memset(ws_buf, 0, sizeof(ws_buf));
    memset(pixels, 0, sizeof(pixels));
    ws2812_show();
}

void ws2812_set_led(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_NUM_LEDS) return;
    pixels[index][0] = g;
    pixels[index][1] = r;
    pixels[index][2] = b;
}

void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++)
    {
        pixels[i][0] = g;
        pixels[i][1] = r;
        pixels[i][2] = b;
    }
}

void ws2812_show(void)
{
    uint16_t pos = 0;
    for (uint8_t led = 0; led < WS2812_NUM_LEDS; led++)
    {
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            uint8_t byte = pixels[led][ch];
            for (int8_t bit = 7; bit >= 0; bit--)
            {
                ws_buf[pos++] = (byte & (1 << bit)) ? WS_BIT_HIGH : WS_BIT_LOW;
            }
        }
    }
    /* Reset pulse: keep MOSI low for >50 µs */
    memset(&ws_buf[pos], 0, RESET_BYTES);

    /* Ensure the SPI transfer is not interrupted — WS2812 requires a
     * continuous bitstream. Disable interrupts briefly for the duration
     * of the transfer (order of 0.5–1.5 ms for 8 LEDs). */
    __disable_irq();
    HAL_SPI_Transmit(ws_hspi, ws_buf, BUF_SIZE, HAL_MAX_DELAY);
    __enable_irq();
}

void ws2812_clear(void)
{
    memset(pixels, 0, sizeof(pixels));
    ws2812_show();
}

void ws2812_set_mission_color(uint8_t mission_index)
{
    if (mission_index < 10)
    {
        ws2812_set_all(
            mission_colors[mission_index][0],
            mission_colors[mission_index][1],
            mission_colors[mission_index][2]);
    }
    else
    {
        ws2812_set_all(255, 0, 0);  /* invalid → red */
    }
    ws2812_show();
}
