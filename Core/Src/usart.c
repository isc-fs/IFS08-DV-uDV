/**
 ******************************************************************************
 * @file    usart.c
 * @brief   Minimal register-level USART10 transmit driver (ASSI LED bridge).
 *
 * The HAL UART module is intentionally NOT enabled in this project, so USART10
 * is driven directly through its registers. Transmit-only, 115200 baud, 8N1.
 *
 * Pin: PG12 = USART10_TX (AF11), matching the team's standard .ioc
 * (USART10 on PG11/PG12). RX (PG11) is unused here — transmit only.
 *
 * Kernel clock: USART1/6/9/10 default to PCLK2 (D2PCLK2). BRR is computed from
 * HAL_RCC_GetPCLK2Freq() so it tracks the configured clock tree.
 ******************************************************************************
 */
#include "usart.h"
#include "stm32h7xx_hal.h"

#define USART10_TX_PORT   GPIOG
#define USART10_TX_PIN    GPIO_PIN_12
#define USART10_TX_AF     GPIO_AF11_USART10
#define USART10_BAUD      115200U

void usart10_init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_USART10_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = USART10_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;   /* 115200 baud: LOW slew is plenty */
    gpio.Alternate = USART10_TX_AF;
    HAL_GPIO_Init(USART10_TX_PORT, &gpio);

    USART10->CR1 = 0U;                       /* disable while configuring       */
    USART10->CR2 = 0U;                       /* 1 stop bit                      */
    USART10->CR3 = 0U;                       /* no flow control, no FIFO        */
    /* Oversampling by 16 (OVER8=0): BRR = f_ck / baud, rounded. 8N1 by reset. */
    USART10->BRR = (HAL_RCC_GetPCLK2Freq() + (USART10_BAUD / 2U)) / USART10_BAUD;
    USART10->CR1 = USART_CR1_TE | USART_CR1_UE;

    while ((USART10->ISR & USART_ISR_TEACK) == 0U) { /* wait transmit-enable ack */ }
}

void usart10_write(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        while ((USART10->ISR & USART_ISR_TXE_TXFNF) == 0U) { /* wait TDR empty */ }
        USART10->TDR = data[i];
    }
    while ((USART10->ISR & USART_ISR_TC) == 0U) { /* wait last byte shifted out */ }
}
