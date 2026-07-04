/*
 * Stub implementations of HAL functions for host-side tests.
 * Most are no-ops; HAL_SPI_Transmit captures its buffer so tests can
 * inspect what production code wrote.
 */
#include "stm32h7xx_hal.h"
#include <string.h>

uint8_t  hal_mock_spi_buf[1024];
uint16_t hal_mock_spi_len = 0;
uint32_t hal_mock_tick = 0;
HAL_I2C_StateTypeDef hal_mock_i2c_state = HAL_I2C_STATE_READY;

void hal_mock_reset(void)
{
  memset(hal_mock_spi_buf, 0, sizeof(hal_mock_spi_buf));
  hal_mock_spi_len = 0;
  hal_mock_tick = 0;
  hal_mock_i2c_state = HAL_I2C_STATE_READY;
}

/* --- Tick / Delay --- */
uint32_t HAL_GetTick(void) { return hal_mock_tick; }
void HAL_Delay(uint32_t Delay) { (void)Delay; }

/* --- I2C --- */
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
  (void)hi2c;
  return hal_mock_i2c_state;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
  (void)pData; (void)Size; (void)Timeout;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
  (void)pData; (void)Size; (void)Timeout;
  return HAL_OK;
}

/* --- SPI --- */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout)
{
  (void)hspi; (void)Timeout;
  uint16_t copy = Size;
  if (copy > sizeof(hal_mock_spi_buf)) copy = sizeof(hal_mock_spi_buf);
  if (pData) memcpy(hal_mock_spi_buf, pData, copy);
  hal_mock_spi_len = copy;
  return HAL_OK;
}

/* --- CORDIC --- */
HAL_StatusTypeDef HAL_CORDIC_Configure(CORDIC_HandleTypeDef *hcordic,
                                       CORDIC_ConfigTypeDef *sConfig)
{
  (void)hcordic; (void)sConfig;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_CORDIC_Calculate(CORDIC_HandleTypeDef *hcordic,
                                       int32_t *pInBuff, int32_t *pOutBuff,
                                       uint32_t NbCalc, uint32_t Timeout)
{
  (void)hcordic; (void)pInBuff; (void)NbCalc; (void)Timeout;
  /* Return zeros — tests that exercise CORDIC math are out of scope for
     pure-logic host tests (would need a working atan2 emulation). */
  if (pOutBuff) {
    pOutBuff[0] = 0;
    pOutBuff[1] = 0;
  }
  return HAL_OK;
}

/* --- GPIO --- */
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  (void)GPIOx; (void)GPIO_Init;
}

void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
  (void)GPIOx; (void)GPIO_Pin;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  (void)GPIOx; (void)GPIO_Pin; (void)PinState;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  (void)GPIOx; (void)GPIO_Pin;
  return GPIO_PIN_SET;
}
