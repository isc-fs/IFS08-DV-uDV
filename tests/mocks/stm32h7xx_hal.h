/*
 * Minimal STM32 HAL shim for host-side unit tests.
 * Provides only what the production .c files reference. Real hardware HAL
 * is NOT used — these are stubs whose behavior is controlled per test.
 */
#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Status enum --- */
typedef enum {
  HAL_OK      = 0x00,
  HAL_ERROR   = 0x01,
  HAL_BUSY    = 0x02,
  HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

#define HAL_MAX_DELAY 0xFFFFFFFFU

/* --- I2C --- */
typedef enum {
  HAL_I2C_STATE_RESET = 0x00,
  HAL_I2C_STATE_READY = 0x20
} HAL_I2C_StateTypeDef;

typedef struct {
  int dummy;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT 0x01u

HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                   uint16_t MemAddress, uint16_t MemAddSize,
                                   uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* --- SPI --- */
typedef struct {
  int dummy;
} SPI_HandleTypeDef;

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout);

/* --- CORDIC --- */
typedef struct {
  int dummy;
} CORDIC_HandleTypeDef;

typedef struct {
  uint32_t Function;
  uint32_t Precision;
  uint32_t Scale;
  uint32_t NbWrite;
  uint32_t NbRead;
  uint32_t InSize;
  uint32_t OutSize;
} CORDIC_ConfigTypeDef;

#define CORDIC_FUNCTION_PHASE     0u
#define CORDIC_PRECISION_6CYCLES  0u
#define CORDIC_SCALE_0            0u
#define CORDIC_NBWRITE_2          0u
#define CORDIC_NBREAD_2           0u
#define CORDIC_INSIZE_32BITS      0u
#define CORDIC_OUTSIZE_32BITS     0u

HAL_StatusTypeDef HAL_CORDIC_Configure(CORDIC_HandleTypeDef *hcordic,
                                       CORDIC_ConfigTypeDef *sConfig);
HAL_StatusTypeDef HAL_CORDIC_Calculate(CORDIC_HandleTypeDef *hcordic,
                                       int32_t *pInBuff, int32_t *pOutBuff,
                                       uint32_t NbCalc, uint32_t Timeout);

/* --- GPIO --- */
typedef struct {
  int dummy;
} GPIO_TypeDef;

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

typedef struct {
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
} GPIO_InitTypeDef;

#define GPIO_MODE_OUTPUT_OD 0u
#define GPIO_NOPULL         0u
#define GPIO_PULLUP         1u
#define GPIO_SPEED_FREQ_LOW 0u

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* --- Tick / Delay --- */
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

/* --- Test-controllable mock state --- */
/* Buffer captured by the most recent HAL_SPI_Transmit call. */
extern uint8_t  hal_mock_spi_buf[1024];
extern uint16_t hal_mock_spi_len;
/* Value returned by HAL_GetTick. Tests can advance this manually. */
extern uint32_t hal_mock_tick;
/* Value returned by HAL_I2C_GetState. */
extern HAL_I2C_StateTypeDef hal_mock_i2c_state;

void hal_mock_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32H7XX_HAL_H */
