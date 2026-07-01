/**
 ******************************************************************************
 * @file    usart.h
 * @brief   Minimal register-level USART10 transmit driver.
 *          Used as the ASSI LED bridge: the STM32 sends short commands to an
 *          Arduino Nano, which drives the physical WS2812 strip.
 ******************************************************************************
 */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Initialize USART10 for transmit-only, 115200 baud, 8N1, on PE3 (AF4).
 * @note  The HAL UART driver is not part of this project, so this is done at
 *        register level. Called once from main() after the clocks are up.
 */
void usart10_init(void);

/**
 * @brief Blocking transmit of @p len bytes over USART10.
 */
void usart10_write(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
