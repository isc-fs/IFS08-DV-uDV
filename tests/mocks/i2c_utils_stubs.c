/*
 * Stub implementations of i2c_utils_* — let bmi088.c link in tests where we
 * only exercise the pure conversion math, never the I/O paths.
 * These return HAL_OK / 0 for everything; if a test ever actually calls
 * bmi088_init_minimal or bmi088_read_*, it gets a no-op success.
 */
#include "i2c_utils.h"

void i2c_utils_init(I2C_Utils_t *ctx,
                    I2C_HandleTypeDef *hi2c,
                    GPIO_TypeDef *scl_port, uint16_t scl_pin,
                    GPIO_TypeDef *sda_port, uint16_t sda_pin)
{
  if (!ctx) return;
  ctx->hi2c = hi2c;
  ctx->scl_port = scl_port; ctx->scl_pin = scl_pin;
  ctx->sda_port = sda_port; ctx->sda_pin = sda_pin;
}

void i2c_utils_bus_recovery(I2C_Utils_t *ctx) { (void)ctx; }

uint8_t i2c_utils_scan(I2C_Utils_t *ctx, uint8_t *found_addrs, uint8_t max_addrs)
{
  (void)ctx; (void)found_addrs; (void)max_addrs;
  return 0;
}

HAL_StatusTypeDef i2c_utils_read_u8(I2C_Utils_t *ctx, uint8_t dev7,
                                    uint8_t reg, uint8_t *val)
{
  (void)ctx; (void)dev7; (void)reg;
  if (val) *val = 0;
  return HAL_OK;
}

HAL_StatusTypeDef i2c_utils_write_u8(I2C_Utils_t *ctx, uint8_t dev7,
                                     uint8_t reg, uint8_t val)
{
  (void)ctx; (void)dev7; (void)reg; (void)val;
  return HAL_OK;
}

HAL_StatusTypeDef i2c_utils_read(I2C_Utils_t *ctx, uint8_t dev7,
                                 uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)ctx; (void)dev7; (void)reg;
  if (buf) {
    for (uint16_t i = 0; i < len; i++) buf[i] = 0;
  }
  return HAL_OK;
}
