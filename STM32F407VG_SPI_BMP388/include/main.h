/**
 ******************************************************************************
 * @file    UART/UART_TwoBoards_ComPolling/Inc/main.h
 * @author  MCD Application Team
 * @brief   Header for main.c module
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "bmp388.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*############################################################################*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx USART2
#define USARTx_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET() __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET() __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN GPIO_PIN_2
#define USARTx_TX_GPIO_PORT GPIOA
#define USARTx_TX_AF GPIO_AF7_USART2
#define USARTx_RX_PIN GPIO_PIN_3
#define USARTx_RX_GPIO_PORT GPIOA
#define USARTx_RX_AF GPIO_AF7_USART2

/* Size of Transmission buffer */
#define TXBUFFERSIZE (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE TXBUFFERSIZE
/*############################################################################*/

/*############################################################################*/
/* User can use this section to tailor SPIx instance used and associated
   resources */
/* Definition for SPIx clock resources */
#define SPIx SPI2
#define SPIx_CLK_ENABLE() __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPIx_FORCE_RESET() __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET() __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
// #define SPIx_SCK_PIN GPIO_PIN_13
// #define SPIx_SCK_GPIO_PORT GPIOB
// #define SPIx_SCK_AF GPIO_AF5_SPI2
// #define SPIx_MISO_PIN GPIO_PIN_14
// #define SPIx_MISO_GPIO_PORT GPIOB
// #define SPIx_MISO_AF GPIO_AF5_SPI2
// #define SPIx_MOSI_PIN GPIO_PIN_15
// #define SPIx_MOSI_GPIO_PORT GPIOB
// #define SPIx_MOSI_AF GPIO_AF5_SPI2
/*############################################################################*/

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */
