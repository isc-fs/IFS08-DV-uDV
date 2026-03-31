/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "cordic.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include "usb_cdc_transport.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceHS;

osThreadId_t microRosPingTaskHandle;
const osThreadAttr_t microRosPingTask_attributes = {
  .name = "microRosPingTask",
  .stack_size = 16384,
  .priority = (osPriority_t) osPriorityNormal
};

extern volatile uint8_t g_transport_open_called;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
void StartMicroRosPingTask(void *argument);
static void led_on(void);
static void led_off(void);
static void led_toggle(void);
static void blink_blocking(uint32_t times, uint32_t delay_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void led_on(void)
{
  HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_SET);
}

static void led_off(void)
{
  HAL_GPIO_WritePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin, GPIO_PIN_RESET);
}

static void led_toggle(void)
{
  HAL_GPIO_TogglePin(OK_STATUS_GPIO_Port, OK_STATUS_Pin);
}

static void blink_blocking(uint32_t times, uint32_t delay_ms)
{
  for (uint32_t i = 0; i < times; i++)
  {
    led_on();
    HAL_Delay(delay_ms);
    led_off();
    HAL_Delay(delay_ms);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_FDCAN3_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_CORDIC_Init();

  led_off();
  HAL_Delay(300);

  MX_USB_DEVICE_Init();

  /* 1 blink = reached USB init */
  blink_blocking(1, 150);

  /* Wait until host configures USB */
  uint32_t t0 = HAL_GetTick();
  while (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
  {
    led_toggle();
    HAL_Delay(100);

    if ((HAL_GetTick() - t0) > 10000U)
    {
      while (1)
      {
        led_toggle();
        HAL_Delay(50);
      }
    }
  }

  /* 2 blinks = USB configured */
  blink_blocking(2, 150);

  rmw_uros_set_custom_transport(
      true,
      NULL,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

  /* 3 blinks = transport registered */
  blink_blocking(3, 150);

  osKernelInitialize();

  MX_FREERTOS_Init();

  microRosPingTaskHandle = osThreadNew(StartMicroRosPingTask, NULL, &microRosPingTask_attributes);

  osKernelStart();

  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                              | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void StartMicroRosPingTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (!g_transport_open_called)
    {
      /* LED off = transport open not called yet */
      led_off();
    }
    else if (rmw_uros_ping_agent(1000, 1) == RMW_RET_OK)
    {
      /* double blink = ping OK */
      led_on();
      osDelay(100);
      led_off();
      osDelay(100);
      led_on();
      osDelay(100);
      led_off();
      osDelay(700);
    }
    else
    {
      /* single blink = ping attempted but failed */
      led_on();
      osDelay(100);
      led_off();
      osDelay(900);
    }

    osDelay(10);
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM23)
  {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(ERR_STATUS_GPIO_Port, ERR_STATUS_Pin);
    HAL_Delay(100);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */