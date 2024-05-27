/**
 *****************************************************************************
 * @file bmp388.c
 * @author Redstone Space Systems
 * @brief This file contains the BMP388 driver implementation. This driver is
 * intended to be used with the STM32F407 or STM32F405 to read
 * data from the BMP388 barometric pressure sensor using the SPI HAL API
 * @version 0.0.1
 * @date 2024-05-25
 * @copyright Copyright (c) 2024
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bmp388.h"

/** @addtogroup BMP388
 * @{
 */

/** @defgroup BMP388_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @defgroup BMP388_Private_Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup BMP388_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup BMP388_Private_Variables
 * @{
 */
// TODO add private variables

/**
 * @}
 */

/** @defgroup BMP388_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @defgroup BMP388_Private_Functions
 * @{
 */

/**
 * @brief  Initialize the BMP388
 * @param  initializationConfig: struct containing BMP388 init parameters
 * @retval None
 */
void BMP388_Initialize(BMP388_InitializationConfig *initializationConfig)
{
    /* Configure the low level interface */
    BMP388_IO_Init();

    /* Configure BMP388 communication mode */
    BMP388_IO_Write(&(initializationConfig->Communication_Mode), BMP388_SERIAL_INTERFACE_CONFIG_ADDR, 1);

    /* Configure BMP388 sensor measurement mode */
    uint8_t pwrConfig = initializationConfig->Sensor_Measurement_Mode;
    uint8_t measusurementConfig = initializationConfig->Sensor_Power_Mode;
    uint8_t pwrMeasurementConfig = pwrConfig | measusurementConfig;
    BMP388_IO_Write(&pwrMeasurementConfig, BMP388_MEASUREMENT_CONFIG_ADDR, 1);

    /* Configure BMP388 oversampling mode */
    uint8_t pressureOSR = initializationConfig->Oversampling_Rate_Pressure;
    uint8_t tempOSR = initializationConfig->Oversampling_Rate_Temperature;
    uint8_t osrConfig = pressureOSR | tempOSR;
    BMP388_IO_Write(&osrConfig, BMP388_OVERSAMPLING_CONFIG_ADDR, 1);

    /* Configure BMP388 output data rate */
    BMP388_IO_Write(&(initializationConfig->Output_DataRate), BMP388_ODR_CONFIG_ADDR, 1);

    /* Configure BMP388 IIR filter setting */
    BMP388_IO_Write(&(initializationConfig->Filter_Setting), BMP388_IIR_FILTER_CONFIG_ADDR, 1);
}

/**
 * @brief  De-Initialize the BMP388
 * @param  None
 * @retval None.
 */
void BMP388_Teardown(void)
{
}

/**
 * @brief  Read the BMP388 device ID.
 * @param  None
 * @retval The Device ID (one byte).
 */
uint8_t BMP388_ReadID(void)
{
    uint8_t tmp = 0;

    /* Configure the low level interface */
    BMP388_IO_Init();

    /* Read chip ID register */
    BMP388_IO_Read(&tmp, BMP388_CHIP_ID_ADDR, 1);

    /* Return the ID */
    return (uint8_t)tmp;
}

/**
 * @brief  Configure Interrupts for the BMP388
 * @param  interruptConfig: pointer to a BMP388_InterruptConfig struct that contains
 * the configurations settings for the BMP388 Interrupt.
 * @retval None
 */
void BMP388_ConfigureInterrupt(BMP388_InterruptConfig *interruptConfig)
{
    /* Configure the device Interrupt IO pin */
    BMP388_IO_ITConfig();

    /* Configure interrupt settings */
    uint8_t outputType = interruptConfig->Output_Type;
    uint8_t activeLevel = interruptConfig->Active_Level;
    uint8_t latching = interruptConfig->Latching;
    uint8_t fifoWatermark = interruptConfig->Fifo_Watermark;
    uint8_t fifoFull = interruptConfig->Fifo_Full;
    uint8_t dataReady = interruptConfig->Data_Ready;
    uint8_t interruptRegisterConfig = outputType | activeLevel | latching | fifoWatermark | fifoFull | dataReady;
    BMP388_IO_Write(&interruptRegisterConfig, BMP388_INTERRUPT_CONFIG_ADDR, 1);
}

/**
 * @brief  Change the power mode for the BMP388.
 * @param  powerMode: new power mode state.
 * This parameter can be one of the following values:
 * @arg BMP388_MEASUREMENT_CONFIG_SLEEP_MODE: Sleep mode (minimal power usage)
 * @arg BMP388_MEASUREMENT_CONFIG_FORCED_1_MODE: Forced mode (low power usage*)
 * @arg BMP388_MEASUREMENT_CONFIG_FORCED_2_MODE: Forced mode (low power usage*)
 * @arg BMP388_MEASUREMENT_CONFIG_NORMAL_MODE: Normal mode
 * @note *if samples are requested at a very low rate
 * @retval None
 */
void BMP388_SetPowerMode(uint8_t powerMode)
{
    uint8_t tmpreg;

    /* Read power measurement configuration register */
    BMP388_IO_Read(&tmpreg, BMP388_MEASUREMENT_CONFIG_ADDR, 1);

    /* Clear only the config address bits 4-5 (power mode selection bits) */
    tmpreg &= (uint8_t)~BMP388_MEASUREMENT_CONFIG_NORMAL_MODE;

    /* Mask the new power mode selection bits */
    tmpreg |= powerMode;

    /* Write the new power measurement configuration value */
    BMP388_IO_Write(&tmpreg, BMP388_MEASUREMENT_CONFIG_ADDR, 1);
}

/**
 * @brief  Change the output data rate for the BMP388.
 * @param  dataRateValue: new data rate value.
 * This parameter can be one of the following values:
 * @arg BMP388_ODR_200_HZ - sampling period = 5 ms
 * @arg BMP388_ODR_100_HZ - sampling period = 10 ms
 * @arg BMP388_ODR_50_HZ - sampling period = 20 ms
 * @arg BMP388_ODR_25_HZ - sampling period = 40 ms
 * @arg BMP388_ODR_12p5_HZ - sampling period = 80 ms
 * @arg BMP388_ODR_6p25_HZ - sampling period = 160 ms
 * @arg BMP388_ODR_3p1_HZ - sampling period = 320 ms
 * @arg BMP388_ODR_1p5_HZ - sampling period = 640 ms
 * @arg BMP388_ODR_0p78_HZ - sampling period = 1.280 s
 * @arg BMP388_ODR_0p39_HZ - sampling period = 2.560 s
 * @arg BMP388_ODR_0p2_HZ - sampling period = 5.120 s
 * @arg BMP388_ODR_0p1_HZ - sampling period = 10.24 s
 * @arg BMP388_ODR_0p05_HZ - sampling period = 20.48 s
 * @arg BMP388_ODR_0p02_HZ - sampling period = 40.96 s
 * @arg BMP388_ODR_0p01_HZ - sampling period = 81.92 s
 * @arg BMP388_ODR_0p006_HZ - sampling period = 163.84 s
 * @arg BMP388_ODR_0p003_HZ - sampling period = 327.68 s
 * @arg BMP388_ODR_0p0015_HZ - sampling period = 655.36 s
 * @retval None
 */
void BMP388_SetDataRate(uint8_t dataRateValue)
{
    /* Write the new power measurement configuration value */
    BMP388_IO_Write(&dataRateValue, BMP388_ODR_CONFIG_ADDR, 1);
}

/**
 * @brief  Reboot memory content of BMP388.
 * @param  None
 * @retval None
 */
void BMP388_Soft_Reset(void)
{
    uint8_t resetCmd = BMP388_CMD_SOFTRESET;

    /* Write reset value to the command register */
    BMP388_IO_Write(&resetCmd, BMP388_CMD_ADDR, 1);
}

/**
 * @brief  Read BMP388 temperature and pressure output registers
 * - return the raw uncompensated output for each reading
 * @param  pointer on floating buffer.
 * @note see section 3.10 for data register readout
 * @note 24 bit UNSIGNED format for pressure and temp
 * @retval None
 */
void BMP388_ReadRawData(uint32_t *pressureUnsignedInt, uint32_t *tempUnsignedInt)
{
    // These 6 registers will be read: (3 pressure and 3 temperature)
    // BMP388_DATA_0_PRESS_7_0, BMP388_DATA_1_PRESS_15_8, BMP388_DATA_2_PRESS_23_16
    // BMP388_DATA_3_TEMP_7_0, BMP388_DATA_4_TEMP_15_8, BMP388_DATA_5_TEMP_23_16

    uint8_t buffer[6];

    // burst read so that data register shadowing (3.10.1) can ensure
    // data consistency - i.e. no register overwriting while reading
    BMP388_IO_Read((uint8_t *)&buffer, BMP388_DATA_0_PRESS_7_0, 6);

    // set data values from buffer
    pressureUnsignedInt[0] = (buffer[2] << 16) + (buffer[1] << 8) + buffer[1];
    tempUnsignedInt[0] = (buffer[5] << 16) + (buffer[4] << 8) + buffer[3];
}

/**     BMP388_Private_Functions
 * @}
 */

/**     BMP388
 * @}
 */
