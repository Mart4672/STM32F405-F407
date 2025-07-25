/**
 ******************************************************************************
 * @file    stm32f4_discovery.h
 * @author  MCD Application Team
 * @brief   This file contains definitions for STM32F4-Discovery Kit's Leds and
 *          push-button hardware resources.
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
#ifndef __STM32F4_DISCOVERY_H
#define __STM32F4_DISCOVERY_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

    /** @addtogroup BSP
     * @{
     */

    /** @addtogroup STM32F4_DISCOVERY
     * @{
     */

    /** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL
     * @{
     */

    /** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types STM32F4 DISCOVERY LOW LEVEL_Exported_Types
     * @{
     */

    typedef enum
    {
        LED4 = 0,
        LED3 = 1,
        LED5 = 2,
        LED6 = 3
    } Led_TypeDef;

    /**     STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types
     * @}
     */

    /** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Constants STM32F4 DISCOVERY LOW LEVEL Exported Constants
     * @{
     */

/**
 * @brief  Define for STM32F4_DISCOVERY board
 */
#if !defined(USE_STM32F4_DISCO)
#define USE_STM32F4_DISCO
#endif

    /** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_LED STM32F4 DISCOVERY LOW LEVEL LED
     * @{
     */

#define LEDn 4

#define LED4_PIN                GPIO_PIN_12
#define LED4_GPIO_PORT          GPIOD
#define LED4_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE() __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED3_PIN                GPIO_PIN_13
#define LED3_GPIO_PORT          GPIOD
#define LED3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE() __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED5_PIN                GPIO_PIN_14
#define LED5_GPIO_PORT          GPIOD
#define LED5_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE() __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED6_PIN                GPIO_PIN_15
#define LED6_GPIO_PORT          GPIOD
#define LED6_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED6_GPIO_CLK_DISABLE() __HAL_RCC_GPIOD_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__) \
    do                                  \
    {                                   \
        if ((__INDEX__) == 0)           \
            LED4_GPIO_CLK_ENABLE();     \
        else if ((__INDEX__) == 1)      \
            LED3_GPIO_CLK_ENABLE();     \
        else if ((__INDEX__) == 2)      \
            LED5_GPIO_CLK_ENABLE();     \
        else if ((__INDEX__) == 3)      \
            LED6_GPIO_CLK_ENABLE();     \
    }                                   \
    while (0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__) \
    do                                   \
    {                                    \
        if ((__INDEX__) == 0)            \
            LED4_GPIO_CLK_DISABLE();     \
        else if ((__INDEX__) == 1)       \
            LED3_GPIO_CLK_DISABLE();     \
        else if ((__INDEX__) == 2)       \
            LED5_GPIO_CLK_DISABLE();     \
        else if ((__INDEX__) == 3)       \
            LED6_GPIO_CLK_DISABLE();     \
    }                                    \
    while (0)
    /**     STM32F4_DISCOVERY_LOW_LEVEL_LED
     * @}
     */

    /** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_BUS STM32F4 DISCOVERY LOW LEVEL BUS
     * @{
     */

// UART/USART2
/*###########################################################################*/
#define USARTx USART2

// Baud Rate
#define UART2_BAUD_RATE 57600 // 9600, 57600, 115200

// Funtions
#define USARTx_CLK_ENABLE()         __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_FORCE_RESET()        __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()      __HAL_RCC_USART2_RELEASE_RESET()

// RX Pin
#define USARTx_RX_PIN       GPIO_PIN_3 // PA.03
#define USARTx_RX_GPIO_PORT GPIOA      // PA.03
#define USARTx_RX_AF        GPIO_AF7_USART2

// TX Pin
#define USARTx_TX_PIN       GPIO_PIN_2 // PA.02
#define USARTx_TX_GPIO_PORT GPIOA      // PA.02
#define USARTx_TX_AF        GPIO_AF7_USART2

// NVIC Interrupt
#define USARTx_IRQn       USART2_IRQn
#define USARTx_IRQHandler USART2_IRQHandler

// Size of Transmission buffer
#define TXBUFFERSIZE (COUNTOF(aTxBuffer) - 1)
// Size of Reception buffer
#define RXBUFFERSIZE TXBUFFERSIZE

// SPI
/*############################################################################*/

/** Maximum Timeout values for flags waiting loops. These timeouts are not based
 * on accurate values, they just guarantee that the application will not remain
 * stuck if the SPI communication is corrupted. You may modify these timeout
 * values depending on CPU frequency and application conditions (interrupts
 * routines ...).
 */
#define SPIx_TIMEOUT_MAX 0x1000 // The value of the maximal timeout for BUS waiting loops

// SPI1 peripheral configuration defines
#define SPI1_GPIO_PORT         GPIOA      // PA.05, PA.06, and PA.07
#define SPI1_SCK               GPIO_PIN_5 // PA.05
#define SPI1_MISO              GPIO_PIN_6 // PA.06
#define SPI1_MOSI              GPIO_PIN_7 // PA.07
#define SPI1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE();

// SPI2 peripheral configuration defines
#define SPI2_GPIO_PORT           GPIOB       // PB.10
#define SPI2_GPIO_PORT_2         GPIOC       // PC.02 and PC.03
#define SPI2_SCK                 GPIO_PIN_10 // PB.10
#define SPI2_MISO                GPIO_PIN_2  // PC.02
#define SPI2_MOSI                GPIO_PIN_3  // PC.03
#define SPI2_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE();
#define SPI2_GPIO_CLK_ENABLE_2() __HAL_RCC_GPIOC_CLK_ENABLE();

// BMI270
/*############################################################################*/
// Read/Write command
#define READWRITE_CMD    ((uint8_t)0x80)
// Multiple byte read/write command
#define MULTIPLEBYTE_CMD ((uint8_t)0x40)
// Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device
#define DUMMY_BYTE       ((uint8_t)0x00)

// Chip Select macro definition
#define BMI270_CS_LOW()  HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_RESET)
#define BMI270_CS_HIGH() HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET)

// Chip Select
#define BMI270_CS_PIN       GPIO_PIN_4 // PA.04
#define BMI270_CS_GPIO_PORT GPIOA      // PA.04

// Primary Interrupt
#define BMI270_INT1_PIN       GPIO_PIN_1 // PD.01
#define BMI270_INT1_EXTI_IRQn EXTI1_IRQn // The interrupt pin MUST match the interrupt line
#define BMI270_INT1_GPIO_PORT GPIOD      // PD.01

// functions
#define BMI270_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define BMI270_CS_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()
#define BMI270_INT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define BMI270_INT_GPIO_CLK_DISABLE() __HAL_RCC_GPIOD_CLK_DISABLE()

// other
#define BMI270_INT2_PIN       GPIO_PIN_1 // PC.01
#define BMI270_INT2_EXTI_IRQn EXTI1_IRQn // The interrupt pin MUST match the interrupt line
#define BMI270_INT2_GPIO_PORT GPIOC      // PC.01

    // Link functions for Barometer peripheral
    void BMI270_IO_Init(void);
    void BMI270_IO_ITConfig(void);

// BMP388
/*############################################################################*/

// Read/Write command - perform a read operation when using this command
#define READWRITE_CMD ((uint8_t)0x80) // 0b 1000 0000

// Dummy Byte to send in order to generate the Clock to the RX device
#define BMP388_DUMMY_BYTE ((uint8_t)0x00)

// Chip Select macro definition
#define BMP388_CS_LOW()  HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_PIN, GPIO_PIN_RESET)
#define BMP388_CS_HIGH() HAL_GPIO_WritePin(BMP388_CS_GPIO_PORT, BMP388_CS_PIN, GPIO_PIN_SET)

// Chip Select
#define BMP388_CS_PIN       GPIO_PIN_12 // PB.12
#define BMP388_CS_GPIO_PORT GPIOB       // PB.12

// Primary Interrupt
#define BMP388_INT1_PIN       GPIO_PIN_0 // PB.00
#define BMP388_INT1_EXTI_IRQn EXTI0_IRQn // The interrupt pin MUST match the interrupt line
#define BMP388_INT1_GPIO_PORT GPIOB      // PB.00

// Functions
#define BMP388_CS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define BMP388_CS_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define BMP388_INT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define BMP388_INT_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()

// Alternate Interrupt
#define BMP388_INT2_PIN       GPIO_PIN_1 // PB.01
#define BMP388_INT2_EXTI_IRQn EXTI1_IRQn // The interrupt pin MUST match the interrupt line
#define BMP388_INT2_GPIO_PORT GPIOB      // PB.01

    /**     STM32F4_DISCOVERY_LOW_LEVEL_BUS
     * @}
     */

    // Link functions for Barometer peripheral
    void BMP388_IO_Init(void);
    void BMP388_IO_ITConfig(void);
    void BMP388_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
    void BMP388_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

// I2C1
/*############################################################################*/

// I2C clock speed configuration (in Hz)
#ifndef BSP_I2C_SPEED
#define BSP_I2C_SPEED 100000
#endif // BSP_I2C_SPEED

// I2C peripheral configuration defines
#define DISCOVERY_I2Cx                   I2C1
#define DISCOVERY_I2Cx_SCL_PIN           GPIO_PIN_6 // PB.06
#define DISCOVERY_I2Cx_SDA_PIN           GPIO_PIN_9 // PB.09
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT GPIOB      // PB.06 and PB.09
#define DISCOVERY_I2Cx_SCL_SDA_AF        GPIO_AF4_I2C1

// Functions
#define DISCOVERY_I2Cx_CLK_ENABLE()              __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2Cx_FORCE_RESET()             __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()           __HAL_RCC_I2C1_RELEASE_RESET()

// I2C interrupt requests
#define DISCOVERY_I2Cx_EV_IRQn I2C1_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn I2C1_ER_IRQn

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define I2Cx_TIMEOUT_MAX 0x1000 /*<! The value of the maximal timeout for BUS waiting loops */

    /**     STM32F4_DISCOVERY_LOW_LEVEL_Exported_Constants
     * @}
     */

    /**     STM32F4_DISCOVERY_LOW_LEVEL
     * @}
     */

    /** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Macros STM32F4 DISCOVERY LOW LEVEL Exported Macros
     * @{
     */
    /**     STM32F4_DISCOVERY_LOW_LEVEL_Exported_Macros
     * @}
     */

    /** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions STM32F4 DISCOVERY LOW LEVEL Exported Functions
     * @{
     */
    uint32_t BSP_GetVersion(void);
    void BSP_LED_Init(Led_TypeDef Led);
    void BSP_LED_On(Led_TypeDef Led);
    void BSP_LED_Off(Led_TypeDef Led);
    void BSP_LED_Toggle(Led_TypeDef Led);

    /**     STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions
     * @}
     */

    /**     STM32F4_DISCOVERY
     * @}
     */

    /**     BSP
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif // __STM32F4_DISCOVERY_H
