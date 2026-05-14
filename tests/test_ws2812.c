/*
 * Tests for ws2812.c
 *
 * Strategy: the encoded SPI buffer is static inside ws2812.c, so we observe
 * it via the HAL_SPI_Transmit mock (hal_mock_spi_buf). ws2812_show pushes
 * the encoded buffer to SPI, and we inspect what got captured.
 *
 * Encoding rules (from ws2812.c):
 *   '1' color bit → SPI byte 0xE0
 *   '0' color bit → SPI byte 0x80
 *   Order per LED: G byte, R byte, B byte; MSB first
 *   After all LED data: 64 reset bytes of 0x00
 */
#include "test.h"
#include "ws2812.h"
#include "stm32h7xx_hal.h"

#define BITS_PER_LED  24
#define RESET_BYTES   64
#define EXPECTED_LEN  (WS2812_NUM_LEDS * BITS_PER_LED + RESET_BYTES)

#define WS_HIGH 0xE0
#define WS_LOW  0x80

static SPI_HandleTypeDef hspi_dummy;

/* Re-init the WS2812 driver to start from a known state for each test.
   ws2812_init clears pixels[], encodes them, and calls ws2812_show — so after
   init the captured buffer is all zeros + reset. */
static void reinit(void) {
  hal_mock_reset();
  ws2812_init(&hspi_dummy);
}

/* Verify a slice of the SPI buffer is all `expected` bytes. */
static void assert_slice_all(uint16_t start, uint16_t len, uint8_t expected)
{
  for (uint16_t i = 0; i < len; i++) {
    if (hal_mock_spi_buf[start + i] != expected) {
      TEST_FAIL("byte[%u]: expected 0x%02X, got 0x%02X",
                start + i, expected, hal_mock_spi_buf[start + i]);
      return;
    }
  }
}

static void test_buffer_length(void)
{
  reinit();
  ASSERT_EQ_INT(hal_mock_spi_len, EXPECTED_LEN);
}

static void test_init_clears_all_pixels(void)
{
  reinit();
  /* All LED data bytes should encode '0' (0x80). */
  assert_slice_all(0, WS2812_NUM_LEDS * BITS_PER_LED, WS_LOW);
  /* Reset trailer is zero. */
  assert_slice_all(WS2812_NUM_LEDS * BITS_PER_LED, RESET_BYTES, 0x00);
}

static void test_clear_after_set(void)
{
  reinit();
  ws2812_set_all(255, 255, 255);
  ws2812_show();
  /* Sanity: all 0xE0 in the LED region */
  assert_slice_all(0, WS2812_NUM_LEDS * BITS_PER_LED, WS_HIGH);

  ws2812_clear();
  assert_slice_all(0, WS2812_NUM_LEDS * BITS_PER_LED, WS_LOW);
}

static void test_set_led_red(void)
{
  reinit();
  /* Red on LED 0 only: r=255 g=0 b=0
     Expected first 24 bytes: G=0 (8x 0x80), R=255 (8x 0xE0), B=0 (8x 0x80). */
  ws2812_set_led(0, 255, 0, 0);
  ws2812_show();

  assert_slice_all(0,  8, WS_LOW);   /* G */
  assert_slice_all(8,  8, WS_HIGH);  /* R */
  assert_slice_all(16, 8, WS_LOW);   /* B */

  /* LEDs 1..7 still off */
  assert_slice_all(24, (WS2812_NUM_LEDS - 1) * BITS_PER_LED, WS_LOW);
}

static void test_set_led_green(void)
{
  reinit();
  ws2812_set_led(1, 0, 255, 0);
  ws2812_show();

  /* LED 1 starts at byte 24. G=255, R=0, B=0 */
  assert_slice_all(24, 8, WS_HIGH);  /* G */
  assert_slice_all(32, 8, WS_LOW);   /* R */
  assert_slice_all(40, 8, WS_LOW);   /* B */
}

static void test_set_led_blue(void)
{
  reinit();
  ws2812_set_led(7, 0, 0, 255);
  ws2812_show();

  /* LED 7 starts at byte 7*24 = 168. G=0, R=0, B=255 */
  assert_slice_all(168, 8, WS_LOW);
  assert_slice_all(176, 8, WS_LOW);
  assert_slice_all(184, 8, WS_HIGH);
}

static void test_set_led_msb_first(void)
{
  reinit();
  /* G channel = 0x81 = 10000001b
     With MSB-first encoding the byte slot becomes:
     [HIGH, LOW, LOW, LOW, LOW, LOW, LOW, HIGH]                            */
  ws2812_set_led(0, 0, 0x81, 0);
  ws2812_show();

  ASSERT_EQ_BYTE(hal_mock_spi_buf[0], WS_HIGH);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[1], WS_LOW);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[2], WS_LOW);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[3], WS_LOW);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[4], WS_LOW);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[5], WS_LOW);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[6], WS_LOW);
  ASSERT_EQ_BYTE(hal_mock_spi_buf[7], WS_HIGH);
}

static void test_set_led_out_of_bounds(void)
{
  reinit();
  /* Should be a no-op — buffer must remain all-zero */
  ws2812_set_led(WS2812_NUM_LEDS, 255, 255, 255);
  ws2812_set_led(200, 255, 255, 255);
  ws2812_show();
  assert_slice_all(0, WS2812_NUM_LEDS * BITS_PER_LED, WS_LOW);
}

static void test_set_all_white(void)
{
  reinit();
  ws2812_set_all(255, 255, 255);
  ws2812_show();
  assert_slice_all(0, WS2812_NUM_LEDS * BITS_PER_LED, WS_HIGH);
  assert_slice_all(WS2812_NUM_LEDS * BITS_PER_LED, RESET_BYTES, 0x00);
}

/* mission_colors[3] = {255, 0, 0} (Autocross = red) */
static void test_mission_color_autocross_is_red(void)
{
  reinit();
  ws2812_set_mission_color(3);
  /* Every LED: G=0, R=255, B=0 */
  for (int led = 0; led < WS2812_NUM_LEDS; led++) {
    uint16_t base = led * BITS_PER_LED;
    assert_slice_all(base,      8, WS_LOW);   /* G */
    assert_slice_all(base + 8,  8, WS_HIGH);  /* R */
    assert_slice_all(base + 16, 8, WS_LOW);   /* B */
  }
}

/* mission_colors[0] = {0, 255, 0} (Manual = green) */
static void test_mission_color_manual_is_green(void)
{
  reinit();
  ws2812_set_mission_color(0);
  for (int led = 0; led < WS2812_NUM_LEDS; led++) {
    uint16_t base = led * BITS_PER_LED;
    assert_slice_all(base,      8, WS_HIGH);  /* G */
    assert_slice_all(base + 8,  8, WS_LOW);   /* R */
    assert_slice_all(base + 16, 8, WS_LOW);   /* B */
  }
}

/* mission_colors[2] = {0, 0, 255} (Skidpad = blue) */
static void test_mission_color_skidpad_is_blue(void)
{
  reinit();
  ws2812_set_mission_color(2);
  for (int led = 0; led < WS2812_NUM_LEDS; led++) {
    uint16_t base = led * BITS_PER_LED;
    assert_slice_all(base,      8, WS_LOW);   /* G */
    assert_slice_all(base + 8,  8, WS_LOW);   /* R */
    assert_slice_all(base + 16, 8, WS_HIGH);  /* B */
  }
}

/* Invalid mission index falls back to red (255, 0, 0). */
static void test_mission_color_invalid_fallback(void)
{
  reinit();
  ws2812_set_mission_color(99);
  for (int led = 0; led < WS2812_NUM_LEDS; led++) {
    uint16_t base = led * BITS_PER_LED;
    assert_slice_all(base,      8, WS_LOW);
    assert_slice_all(base + 8,  8, WS_HIGH);
    assert_slice_all(base + 16, 8, WS_LOW);
  }
}

int main(void)
{
  TEST_BEGIN();
  TEST_CASE(test_buffer_length);
  TEST_CASE(test_init_clears_all_pixels);
  TEST_CASE(test_clear_after_set);
  TEST_CASE(test_set_led_red);
  TEST_CASE(test_set_led_green);
  TEST_CASE(test_set_led_blue);
  TEST_CASE(test_set_led_msb_first);
  TEST_CASE(test_set_led_out_of_bounds);
  TEST_CASE(test_set_all_white);
  TEST_CASE(test_mission_color_autocross_is_red);
  TEST_CASE(test_mission_color_manual_is_green);
  TEST_CASE(test_mission_color_skidpad_is_blue);
  TEST_CASE(test_mission_color_invalid_fallback);
  TEST_END();
}
