/**
 ******************************************************************************
 * @file    stm32f4_discovery.c
 * @author  MCD Application Team
 * @brief   This file provides set of firmware functions to manage Leds and
 *          push-button available on STM32F4-Discovery Kit from STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"

/** @defgroup BSP BSP
 * @{
 */

/** @defgroup STM32F4_DISCOVERY STM32F4 DISCOVERY
 * @{
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL STM32F4 DISCOVERY LOW LEVEL
 * @brief This file provides set of firmware functions to manage Leds and push-button
 *        available on STM32F4-Discovery Kit from STMicroelectronics.
 * @{
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_TypesDefinitions STM32F4 DISCOVERY LOW LEVEL Private TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Defines STM32F4 DISCOVERY LOW LEVEL Private Defines
 * @{
 */

/**
 * @brief STM32F4 DISCO BSP Driver version number V2.1.5
 */
#define __STM32F4_DISCO_BSP_VERSION_MAIN (0x02) /*!< [31:24] main version */
#define __STM32F4_DISCO_BSP_VERSION_SUB1 (0x01) /*!< [23:16] sub1 version */
#define __STM32F4_DISCO_BSP_VERSION_SUB2 (0x05) /*!< [15:8]  sub2 version */
#define __STM32F4_DISCO_BSP_VERSION_RC (0x00)   /*!< [7:0]  release candidate */
#define __STM32F4_DISCO_BSP_VERSION ((__STM32F4_DISCO_BSP_VERSION_MAIN << 24) | (__STM32F4_DISCO_BSP_VERSION_SUB1 << 16) | (__STM32F4_DISCO_BSP_VERSION_SUB2 << 8) | (__STM32F4_DISCO_BSP_VERSION_RC))
/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Macros STM32F4 DISCOVERY LOW LEVEL Private Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Variables STM32F4 DISCOVERY LOW LEVEL Private Variables
 * @{
 */
GPIO_TypeDef *GPIO_PORT[LEDn] = {LED4_GPIO_PORT,
                                 LED3_GPIO_PORT,
                                 LED5_GPIO_PORT,
                                 LED6_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED4_PIN,
                                 LED3_PIN,
                                 LED5_PIN,
                                 LED6_PIN};

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */

static SPI_HandleTypeDef hspi2;

/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes STM32F4 DISCOVERY LOW LEVEL Private FunctionPrototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Functions STM32F4 DISCOVERY LOW LEVEL Private Functions
 * @{
 */

static void SPI2_Init(void);
static uint8_t SPI2_WriteRead(uint8_t Byte);
static void SPI2_Error(void);

/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_LED_Functions STM32F4 DISCOVERY LOW LEVEL LED Functions
 * @{
 */

/**
 * @brief  This method returns the STM32F4 DISCO BSP Driver revision
 * @retval version : 0xXYZR (8bits for each decimal, R for RC)
 */
uint32_t BSP_GetVersion(void)
{
    return __STM32F4_DISCO_BSP_VERSION;
}

/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 */
void BSP_LED_Init(Led_TypeDef Led)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable the GPIO_LED Clock */
    LEDx_GPIO_CLK_ENABLE(Led);

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = GPIO_PIN[Led];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

    HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 */
void BSP_LED_On(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 */
void BSP_LED_Off(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 */
void BSP_LED_Toggle(Led_TypeDef Led)
{
    HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
 * @}
 */

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_BUS_Functions STM32F4 DISCOVERY LOW LEVEL BUS Functions
 * @{
 */

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* SPI Routines *********************************/

/**
 * @brief  SPI2 Bus initialization
 */
static void SPI2_Init(void)
{
    // only initialize if not set up
    if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_RESET)
    {
        /* SPI configuration -----------------------------------------------------*/
        hspi2.Instance = SPI2;
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        hspi2.Init.Direction = SPI_DIRECTION_2LINES;
        hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        hspi2.Init.CRCPolynomial = 7;
        hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
        hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi2.Init.NSS = SPI_NSS_SOFT;
        hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
        hspi2.Init.Mode = SPI_MODE_MASTER;

        // HAL_SPI_Init internally calls HAL_SPI_MspInit in stm32f4xx_hal_msp.c
        HAL_SPI_Init(&hspi2);
    }
}

/**
 * @brief  Sends a Byte through the SPI interface and return the Byte received
 *         from the SPI bus.
 * @param  Byte: Byte send.
 * @retval The received byte value
 */
static uint8_t SPI2_WriteRead(uint8_t Byte)
{
    uint8_t receivedbyte = 0;

    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&Byte, (uint8_t *)&receivedbyte, 1, SpixTimeout) != HAL_OK)
    {
        SPI2_Error();
    }

    return receivedbyte;
}

/**
 * @brief  SPI2 error treatment function.
 */
static void SPI2_Error(void)
{
    // HAL_SPI_DeInit internally calls HAL_SPI_MspDeInit in stm32f4xx_hal_msp.c
    HAL_SPI_DeInit(&hspi2);

    // Re-Initialize the SPI communication bus
    SPI2_Init();
}

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/***************************** LINK BMP388 *****************************/

/**
 * @brief  Configures the BMP388 SPI interface.
 */
void BMP388_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable the GPIO clock for the Chip Select pin
    BMP388_CS_GPIO_CLK_ENABLE();

    // Configure the GPIO pin for BMP388 Chip select
    GPIO_InitStructure.Pin = BMP388_CS_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BMP388_CS_GPIO_PORT, &GPIO_InitStructure);

    // set the CS pin low for a while to ensure the SPI interface is activated and I2C is disabled
    BMP388_CS_LOW();
    HAL_Delay(200);

    // BMP388 CS pin should be default high
    BMP388_CS_HIGH();

    // Initalize SPI2 Bus
    SPI2_Init();
}

/**
 * @brief  Configures the BMP388 IO Interrupts.
 */
void BMP388_IO_ITConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable INT GPIO clock and configure GPIO PINs to detect Interrupts */
    BMP388_INT_GPIO_CLK_ENABLE();

    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStructure.Pin = BMP388_INT1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BMP388_INT1_GPIO_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(BMP388_INT1_EXTI_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(BMP388_INT1_EXTI_IRQn);
}

/**
 * @brief  Writes a block of data to the BMP388
 * @param  pBuffer: pointer to the buffer containing the data to be written.
 * @param  WriteAddr: internal address to write to.
 * @param  NumByteToWrite: Number of bytes to write.
 */
void BMP388_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    /* Set chip select Low at the start of the transmission */
    BMP388_CS_LOW();

    /* Send the Address of the indexed register */
    SPI2_WriteRead(WriteAddr);

    /* Send the data that will be written into the device (MSB First) */
    while (NumByteToWrite >= 0x01)
    {
        SPI2_WriteRead(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }

    /* Set chip select High at the end of the transmission */
    BMP388_CS_HIGH();
}

/**
 * @brief  Reads a block of data from the BMP388.
 * @param  pBuffer: pointer to the buffer that receives the data read.
 * @param  ReadAddr: internal address to read from.
 * @param  NumByteToRead: number of bytes to read.
 */
void BMP388_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    /* set the Most Significant Bit (MSB) to perform a read command */
    ReadAddr |= (uint8_t)READWRITE_CMD;

    /* Set chip select Low at the start of the transmission */
    BMP388_CS_LOW();

    /* Send the Address of the indexed register */
    SPI2_WriteRead(ReadAddr);

    /* When reading from the BMP388, the second transaction byte is a dummy byte - see section 5.3.2 */
    SPI2_WriteRead(ReadAddr); // read again to increment past the returned dummy byte

    /* Receive the data that will be read from the device (MSB First) */
    while (NumByteToRead > 0x00)
    {
        /* Send dummy byte (0x00) to generate the SPI clock for the peripheral device */
        *pBuffer = SPI2_WriteRead(BMP388_DUMMY_BYTE);
        NumByteToRead--;
        pBuffer++;
    }

    /* Set chip select High at the end of the transmission */
    BMP388_CS_HIGH();
}

/***************************** LINK BMI270 *****************************/

/**
 * @brief  Configures the BMI270 SPI interface.
 */
void BMI270_IO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure the BMI270 Control pins --------------------------------*/
    /* Enable CS GPIO clock and configure GPIO pin for BMI270 Chip select */
    BMI270_CS_GPIO_CLK_ENABLE();

    /* Configure GPIO PIN for LIS Chip select */
    GPIO_InitStructure.Pin = BMI270_CS_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BMI270_CS_GPIO_PORT, &GPIO_InitStructure);

    /* Deselect: Chip Select high */
    BMI270_CS_HIGH();
}

/**
 * @brief  Configures the BMI270 IO Interrupts.
 *         EXTI0 is already used by user button so INT1 is not configured here.
 */
void BMI270_IO_ITConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable INT1 GPIO clock and configure GPIO PINs to detect Interrupts */
    BMI270_INT_GPIO_CLK_ENABLE();

    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStructure.Pin = BMI270_INT1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BMI270_INT1_GPIO_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority((IRQn_Type)BMI270_INT1_EXTI_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)BMI270_INT1_EXTI_IRQn);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
