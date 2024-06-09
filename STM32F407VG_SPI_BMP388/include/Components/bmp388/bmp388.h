/**
 ******************************************************************************
 * @file    bmp388.h
 * @version 0.0.1
 * @date    16-February-2024
 * @brief   This file contains all the functions prototypes for the bmp388.c
 * firmware driver.
 * @cite   Datasheet:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
 *
 * ****************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMP388_H
#define __BMP388_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4_discovery.h"

    /** @defgroup BMP388
     * @{
     */

    /** @defgroup BMP388_Exported_Types
     * @{
     */

    /* BMP388 Initialization struct */
    typedef struct
    {
        uint8_t Communication_Mode;            // SPI Mode - 4 or 3 wire
        uint8_t Sensor_Measurement_Mode;       // Enable/Disable Pressure/Temp measurements
        uint8_t Sensor_Power_Mode;             // Sleep/Forced/Normal Mode
        uint8_t Oversampling_Rate_Pressure;    // Oversample rate for pressure measurements
        uint8_t Oversampling_Rate_Temperature; // Oversample rate for temperature measurements
        uint8_t Output_DataRate;               // Ouput data rate in Hz
        uint8_t Filter_Setting;                // IIR filter setting
    } BMP388_InitializationConfig;

    /* BMP388 Interrupt struct */
    typedef struct
    {
        uint8_t Output_Type;    // Interrupt Output Type
        uint8_t Active_Level;   // Interrupt Pin Active State
        uint8_t Latching;       // Interrupt Pin and Status Register Latching
        uint8_t Fifo_Watermark; // FIFO watermark reached interrupt
        uint8_t Fifo_Full;      // FIFO full interrupt
        uint8_t Data_Ready;     // Data Ready interrupt
    } BMP388_InterruptConfig;

    /* BMP388 Data Calibration struct */
    typedef struct
    {
        // temperature calibration parameters
        float par_t1;
        float par_t2;
        float par_t3;
        // calibrated temperature
        float t_lin;
        // pressure calibration parameters
        float par_p1;
        float par_p2;
        float par_p3;
        float par_p4;
        float par_p5;
        float par_p6;
        float par_p7;
        float par_p8;
        float par_p9;
        float par_p10;
        float par_p11;
    } BMP388_calib_data;

    /**   BMP388_Exported_Types
     * @}
     */

    /** @defgroup BMP388_Exported_Functions
     * @{
     */

    void BMP388_Initialize(BMP388_InitializationConfig *initializationConfig);
    void BMP388_ConfigureInterrupt(BMP388_InterruptConfig *interruptConfig);
    uint8_t BMP388_ReadID(void);
    uint8_t BMP388_ReadIntStatus(void);
    void BMP388_SetPowerMode(uint8_t powerMode);
    void BMP388_SetDataRate(uint8_t dataRateValue);
    void BMP388_Soft_Reset(void);
    void BMP388_Teardown(void);
    void BMP388_ReadRawData(uint32_t *pressureUnsignedInt, uint32_t *tempUnsignedInt);
    void BMP388_GetCalibratedData(float *pressureCalibratedFloat, float *tempCalibratedFloat);
    void BMP388_CalibrateRawData(uint32_t pressureRaw, uint32_t tempRaw, float *pressureCal, float *tempCal);

    /**   BMP388_Exported_Functions
     * @}
     */

/** @defgroup BMP388_Exported_Constants
 * @{
 */

/**
 * @defgroup BMP388_CHIP_ID_REGISTER
 * @note Read Only
 * @note Static value of 0x50
 * @{
 */
#define BMP388_CHIP_ID_ADDR 0x00
#define BMP388_CHIP_ID 0x50
/**
 * @}
 */

/**
 * @defgroup BMP388_ERROR_REGISTER
 * @note Read Only
 * @note Default value of 0x00
 * @note Bit 0: fatal_err - Fatal error
 * @note Bit 1: cmd_err - Command execution failed. Cleared on read
 * @note Bit 2: conf_err - Sensor configuration error detected. Cleared on read
 * @{
 */
#define BMP388_ERROR_ADDR 0x02
/**
 * @}
 */

/**
 * @defgroup BMP388_STATUS_REGISTER
 * @note Read Only
 * @note Default value of 0x00
 * @note Bit 4: cmd_rdy - 0 = command in progress, 1 = command decoder ready to accept a new command
 * @note Bit 5: drdy_press - pressure data ready, reset when 1 pressure DATA register is read
 * @note Bit 6: drdy_temp - temp data ready, reset when 1 temp DATA register is read
 * @{
 */
#define BMP388_STATUS_ADDR 0x03
/**
 * @}
 */

/**
 * @defgroup BMP388_PRESSURE_DATA_REGISTERS
 * @note Read Only
 * @note DATA_0: Pressure Data Bits 7 to 0 (default value 0x00)
 * @note DATA_1: Pressure Data Bits 15 to 8 (default value 0x00)
 * @note DATA_2: Pressure Data Bits 23 to 16 (default value 0x80)
 * @{
 */
#define BMP388_DATA_0_PRESS_7_0 0x04
#define BMP388_DATA_1_PRESS_15_8 0x05
#define BMP388_DATA_2_PRESS_23_16 0x06
/**
 * @}
 */

/**
 * @defgroup BMP388_TEMPERATURE_DATA_REGISTERS
 * @note Read Only
 * @note DATA_3: Temperature Data Bits 7 to 0 (default value 0x00)
 * @note DATA_4: Temperature Data Bits 15 to 8 (default value 0x00)
 * @note DATA_5: Temperature Data Bits 23 to 16 (default value 0x80)
 * @{
 */
#define BMP388_DATA_3_TEMP_7_0 0x07
#define BMP388_DATA_4_TEMP_15_8 0x08
#define BMP388_DATA_5_TEMP_23_16 0x09
/**
 * @}
 */

/**
 * @defgroup BMP388_SENSOR_TIME_DATA_REGISTERS
 * @note 24 bits across 3 registers, Read Only
 * @note Default value of 0x00
 * @note SENSORTIME_0: Sensor Time Data Bits 7 to 0
 * @note SENSORTIME_1: Sensor Time Data Bits 15 to 8
 * @note SENSORTIME_2: Sensor Time Data Bits 23 to 16
 * @{
 */
#define BMP388_DATA_0_SENSORTIME_7_0 0x0C
#define BMP388_DATA_1_SENSORTIME_15_8 0x0D
#define BMP388_DATA_2_SENSORTIME_23_16 0x0E
/**
 * @}
 */

/**
 * @defgroup BMP388_SENSOR_POWER_EVENT_REGISTER
 * @note Read Only
 * @note Default value of 0x01
 * @note Bit 0: por_detected - 1 after power up or soft reset, cleared on read
 * @{
 */
#define BMP388_POWER_EVENT_ADDR 0x10
/**
 * @}
 */

/**
 * @defgroup BMP388_INTERRUPT_STATAUS_REGISTER
 * @note Read Only
 * @note Default value of 0x00
 * @note BMP388 has a FIFO Buffer that can store sensor readings
 * @note The FIFO size is 512 Bytes (4096 Bits)
 * @note The FIFO Watermark is a user-configurable # of bytes
 * @note The following bits are set to 1 when the interrupt is active
 * @note Bit 0: fwm_int - FIFO Watermark (editable # Bytes) Interrupt
 * @note Bit 1: ffull_int - FIFO Full (512 Bytes) Interrupt
 * @note Bit 3: drdy - Data Ready Interrupt
 * @{
 */
#define BMP388_INTERRUPT_STATUS_ADDR 0x11
/**
 * @}
 */

/**
 * @defgroup BMP388_FIFO_BUFFER_FILL_LEVEL_REGISTERS
 * @note 9 bits across 2 registers, Read Only
 * @note Default value of 0x00
 * @note FIFO_LENTH_0: FIFO Byte Count Bits 7 to 0
 * @note FIFO_LENTH_1: Bit 0 = FIFO Byte Count Bit 8
 * @{
 */
#define BMP388_FIFO_LENGTH_0_B_7_0 0x12
#define BMP388_FIFO_LENGTH_1_B_8 0x13
/**
 * @}
 */

/**
 * @defgroup BMP388_FIFO_DATA_OUTPUT_REGISTER
 * @note Read Only
 * @note Default value of 0x00
 * @note Reading this register again presumably reads the next Byte in the FIFO
 * @{
 */
#define BMP388_FIFO_DATA 0x14
/**
 * @}
 */

/**
 * @defgroup BMP388_FIFO_WATERMARK_REGISTERS
 * @note 9 bits across 2 registers, Read/Write
 * @note FIFO_WTM_0: FIFO Watermark Level Bits 7 to 0 (default value 0x01)
 * @note FIFO_WTM_1: Bit 0 = FIFO Watermark Level Bit 8 (default value 0x00)
 * @todo define values for each watermark level
 * @{
 */
#define BMP388_FIFO_WATERMARK_0_BIT_7_0 0x15
#define BMP388_FIFO_WATERMARK_1_BIT_8 0x16
/**
 * @}
 */

/**
 * @defgroup BMP388_FIFO_CONFIGURATION_REGISTERS
 * @note 10 bits across 2 registers, Read/Write
 * @note Default value of 0x02
 * @todo define values for each configuration setting
 * @{
 */

/**
 * @brief Configuration Register 1
 * @note Bit 0: fifo_mode - 0 = FIFO disabled, 1 = FIFO enabled
 * @note Bit 1: fifo_stop_on_full - 0 = keep writing when full, 1 = stop writing when full
 * @note Bit 2: fifo_time_en - 0 = don't return sensortime frame after last valid data frame, 1 = return sensortime frame after last valid data frame
 * @note Bit 3: fifo_press_en - 0 = don't store pressure data in FIFO, 1 = store pressure data in FIFO
 * @note Bit 4: fifo_temp_en - 0 = don't store temperature data in FIFO, 1 = store temperature data in FIFO
 */
#define BMP388_FIFO_CONFIG_1_BIT_4_0 0x17

/**
 * @brief Configuration Register 2
 * @note Bits 2 to 0: fifo_subsampling - can be 0-7 based on bits 0, 1, and 2
 * @note Read Section 3.6.2 FIFO data sampling selection
 * @note Reduce FIFO input data rate by using a down-sampling factor
 * @note the down-sampling factor "n" can range from 1 to 128 based on the value of 2^(fifo_subsampling)
 * @note Bit 4 to 3: data_select
 * @note Select data source for pressure and temperature
 * @note 00 = unfiltered data (compensated or uncompensated), 01 = filtered data (compensated or uncompensated), 11 = reserved, same as for "unfilt" (don't use?)
 */
#define BMP388_FIFO_CONFIG_2_BIT_4_0 0x18

/**
 * @}
 */

/**
 * @defgroup BMP388_INTERRUPT_CONFIGURATION_REGISTER
 * @note Read/Write
 * @note Default value of 0x02
 * @note Bit 0: int_od - 0 = push-pull (BMP sets INT to 0 or 1), 1 = open-drain (BMP sets int to floating or 0)
 * @note Bit 1: int_level - level of interrupt pin, 0 = active low, 1 = active high
 * @note Bit 2: int_latch - latching of interrupts for INT pin and INT_STATUS register
 * @note - i.e. interrupt pin won't reset after 2.5ms until the INT_STATUS register is read
 * @note Bit 3: fwtm_en - enable interrupt for INT pin and INT_STATUS register when FIFO watermark has been reached
 * @note Bit 4: ffull_en - enable interrupt for INT pin and INT_STATUS register when FIFO is full
 * @note Bit 6: drdy_en - enable interrupt for INT pin and INT_STATUS register when sensor data is ready
 * @{
 */
#define BMP388_INTERRUPT_CONFIG_ADDR 0x19
#define BMP388_INTERRUPT_OUTOUT_PUSH_PULL ((uint8_t)0x00)        // 0b 0000 0000
#define BMP388_INTERRUPT_OUTPUT_OPEN_DRAIN ((uint8_t)0x01)       // 0b 0000 0001
#define BMP388_INTERRUPT_ACTIVE_LOW ((uint8_t)0x00)              // 0b 0000 0000
#define BMP388_INTERRUPT_ACTIVE_HIGH ((uint8_t)0x02)             // 0b 0000 0010
#define BMP388_INTERRUPT_LATCHING_DISABLED ((uint8_t)0x00)       // 0b 0000 0000
#define BMP388_INTERRUPT_LATCHING_ENABLED ((uint8_t)0x04)        // 0b 0000 0100
#define BMP388_INTERRUPT_FIFO_WATERMARK_DISABLED ((uint8_t)0x00) // 0b 0000 0000
#define BMP388_INTERRUPT_FIFO_WATERMARK_ENABLED ((uint8_t)0x08)  // 0b 0000 1000
#define BMP388_INTERRUPT_FIFO_FULL_DISABLED ((uint8_t)0x00)      // 0b 0000 0000
#define BMP388_INTERRUPT_FIFO_FULL_ENABLED ((uint8_t)0x10)       // 0b 0001 0000
#define BMP388_INTERRUPT_DATA_READY_DISABLED ((uint8_t)0x00)     // 0b 0000 0000
#define BMP388_INTERRUPT_DATA_READY_ENABLED ((uint8_t)0x40)      // 0b 0100 0000
/**
 * @}
 */

/**
 * @defgroup BMP388_SERIAL_INTERFACE_CONFIGURATION_REGISTER
 * @note Read/Write
 * @note Default value of 0x00
 * @note Bit 0: spi3 - 0 = SPI 4-wire mode, 1 = SPI 3-wire mode
 * @note Bit 1: i2c_wdt_en - 0 = disable I2C Watchdog timer, 1 = enable I2C Watchdog timer, backed by NVM
 * @note Bit 2: i2c_wdt_sel - 0 I2C watchdog timeout after 1.25 ms, 1 = I2C watchdog timeout after 40 ms
 * @{
 */
#define BMP388_SERIAL_INTERFACE_CONFIG_ADDR 0x1A
#define BMP388_SERIAL_INTERFACE_4WIRE ((uint8_t)0x00)            // 0b 0000 0000
#define BMP388_SERIAL_INTERFACE_3WIRE ((uint8_t)0x01)            // 0b 0000 0001
#define BMP388_SERIAL_INTERFACE_I2C_WDT_DISABLED ((uint8_t)0x00) // 0b 0000 0000
#define BMP388_SERIAL_INTERFACE_I2C_WDT_ENABLED ((uint8_t)0x02)  // 0b 0000 0010
#define BMP388_SERIAL_INTERFACE_WDT_TIMEOUT_1MS ((uint8_t)0x00)  // 0b 0000 0000
#define BMP388_SERIAL_INTERFACE_WDT_TIMEOUT_40MS ((uint8_t)0x04) // 0b 0000 0100
/**
 * @}
 */

/**
 * @defgroup BMP388_POWER_MEASUREMENT_CONFIGURATION_REGISTER
 * @note Read/Write
 * @note Default value of 0x00
 * @note Bit 0: press_en - 0 = disable pressure sensor, 1 = enable pressure sensor
 * @note Bit 1: temp_en - 0 = disable temp sensor, 1 = enable temp sensor
 * @note Bit 5 to 4: mode - Sensor Power Mode - Read Section 3.3 "Power Modes" for more information on each mode
 * @note - 00 = Sleep Mode (No measurements)
 * @note - 01 or 10 (either works) = Forced Mode (1 measurement then sleep until sensor put into forced mode again)
 * @note - 11 = Normal Mode (Continuous measurements)
 * @{
 */
#define BMP388_MEASUREMENT_CONFIG_ADDR 0x1B
#define BMP388_MEASUREMENT_CONFIG_NO_MEASUREMENTS ((uint8_t)0x00)   // 0b 0000 0000
#define BMP388_MEASUREMENT_CONFIG_PRESSURE_ONLY ((uint8_t)0x01)     // 0b 0000 0001
#define BMP388_MEASUREMENT_CONFIG_TEMPERATURE_ONLY ((uint8_t)0x02)  // 0b 0000 0010
#define BMP388_MEASUREMENT_CONFIG_TEMP_AND_PRESSURE ((uint8_t)0x03) // 0b 0000 0011
#define BMP388_MEASUREMENT_CONFIG_SLEEP_MODE ((uint8_t)0x00)        // 0b 0000 0000
#define BMP388_MEASUREMENT_CONFIG_FORCED_1_MODE ((uint8_t)0x10)     // 0b 0001 0000
#define BMP388_MEASUREMENT_CONFIG_FORCED_2_MODE ((uint8_t)0x20)     // 0b 0010 0000
#define BMP388_MEASUREMENT_CONFIG_NORMAL_MODE ((uint8_t)0x30)       // 0b 0011 0000
/**
 * @}
 */

/**
 * @defgroup BMP388_OVERSAMPLING_CONFIGURATION_REGISTER
 * @note Read/Write
 * @note Default value of 0x02
 * @note Bit 2 to 0: osr_p - Oversampling for pressure measurement
 * @note - 000 = x1 (no oversampling), 001 = x2, 010 = x4, 011 = x8, 100 = x16, 101 = x32
 * @note Bit 5 to 3: osr4_t - Oversampling for temperature measurement
 * @note - 000 = x1 (no oversampling), 001 = x2, 010 = x4, 011 = x8, 100 = x16, 101 = x32
 * @{
 */
#define BMP388_OVERSAMPLING_CONFIG_ADDR 0x1C
#define BMP388_PRESSURE_OSR_X1 ((uint8_t)0x00)     // 0b 0000 0000
#define BMP388_PRESSURE_OSR_X2 ((uint8_t)0x01)     // 0b 0000 0001
#define BMP388_PRESSURE_OSR_X4 ((uint8_t)0x02)     // 0b 0000 0010
#define BMP388_PRESSURE_OSR_X8 ((uint8_t)0x03)     // 0b 0000 0011
#define BMP388_PRESSURE_OSR_X16 ((uint8_t)0x04)    // 0b 0000 0100
#define BMP388_PRESSURE_OSR_X32 ((uint8_t)0x05)    // 0b 0000 0101
#define BMP388_TEMPERATURE_OSR_X1 ((uint8_t)0x00)  // 0b 0000 0000
#define BMP388_TEMPERATURE_OSR_X2 ((uint8_t)0x08)  // 0b 0000 1000
#define BMP388_TEMPERATURE_OSR_X4 ((uint8_t)0x10)  // 0b 0001 0000
#define BMP388_TEMPERATURE_OSR_X8 ((uint8_t)0x18)  // 0b 0001 1000
#define BMP388_TEMPERATURE_OSR_X16 ((uint8_t)0x20) // 0b 0010 0000
#define BMP388_TEMPERATURE_OSR_X32 ((uint8_t)0x28) // 0b 0010 1000
/**
 * @}
 */

/**
 * @defgroup BMP388_OUPUT_DATA_RATE_CONFIGURATION_REGISTER
 * @note Read/Write
 * @note Default value of 0x00
 * @note Bit 4 to 0: odr_sel - Output Data Rate Select (range 0 to 17)
 * @note ODR uses subdivision/prescaler value of 2^(odr_sel)
 * @{
 */
#define BMP388_ODR_CONFIG_ADDR 0x1D
#define BMP388_ODR_200_HZ ((uint8_t)0x00)    // sampling period = 5 ms, 2^odr_sel = 1
#define BMP388_ODR_100_HZ ((uint8_t)0x01)    // sampling period = 10 ms, 2^odr_sel = 2
#define BMP388_ODR_50_HZ ((uint8_t)0x02)     // sampling period = 20 ms, 2^odr_sel = 4
#define BMP388_ODR_25_HZ ((uint8_t)0x03)     // sampling period = 40 ms, 2^odr_sel = 8
#define BMP388_ODR_12p5_HZ ((uint8_t)0x04)   // sampling period = 80 ms, 2^odr_sel = 16
#define BMP388_ODR_6p25_HZ ((uint8_t)0x05)   // sampling period = 160 ms, 2^odr_sel = 32
#define BMP388_ODR_3p1_HZ ((uint8_t)0x06)    // sampling period = 320 ms, 2^odr_sel = 64
#define BMP388_ODR_1p5_HZ ((uint8_t)0x07)    // sampling period = 640 ms, 2^odr_sel = 128
#define BMP388_ODR_0p78_HZ ((uint8_t)0x08)   // sampling period = 1.280 s, 2^odr_sel = 256
#define BMP388_ODR_0p39_HZ ((uint8_t)0x09)   // sampling period = 2.560 s, 2^odr_sel = 512
#define BMP388_ODR_0p2_HZ ((uint8_t)0x0A)    // sampling period = 5.120 s, 2^odr_sel = 1024
#define BMP388_ODR_0p1_HZ ((uint8_t)0x0B)    // sampling period = 10.24 s, 2^odr_sel = 2048
#define BMP388_ODR_0p05_HZ ((uint8_t)0x0C)   // sampling period = 20.48 s, 2^odr_sel = 4096
#define BMP388_ODR_0p02_HZ ((uint8_t)0x0D)   // sampling period = 40.96 s, 2^odr_sel = 8192
#define BMP388_ODR_0p01_HZ ((uint8_t)0x0E)   // sampling period = 81.92 s, 2^odr_sel = 16384
#define BMP388_ODR_0p006_HZ ((uint8_t)0x0F)  // sampling period = 163.84 s, 2^odr_sel = 32768
#define BMP388_ODR_0p003_HZ ((uint8_t)0x10)  // sampling period = 327.68 s, 2^odr_sel = 65536
#define BMP388_ODR_0p0015_HZ ((uint8_t)0x11) // sampling period = 655.36 s, 2^odr_sel = 131072
/**
 * @}
 */

/**
 * @defgroup BMP388_IIR_FILTER_CONFIGURATION_REGISTER
 * @note Read/Write
 * @note Default value of 0x00
 * @note Bit 3 to 1: iir_filter - coefficient for IIR filter - See section 3.4.3 for more information
 * @{
 */
#define BMP388_IIR_FILTER_CONFIG_ADDR 0x1F
#define BMP388_IIR_FILTER_COEF_0 ((uint8_t)0x00)   // 0b 0000 0000 = 0 coefficient (bypass mode)
#define BMP388_IIR_FILTER_COEF_1 ((uint8_t)0x02)   // 0b 0000 0010 = filter coefficient of 1
#define BMP388_IIR_FILTER_COEF_3 ((uint8_t)0x04)   // 0b 0000 0100 = filter coefficient of 3
#define BMP388_IIR_FILTER_COEF_7 ((uint8_t)0x06)   // 0b 0000 0110 = filter coefficient of 7
#define BMP388_IIR_FILTER_COEF_15 ((uint8_t)0x08)  // 0b 0000 1000 = filter coefficient of 15
#define BMP388_IIR_FILTER_COEF_31 ((uint8_t)0x0A)  // 0b 0000 1010 = filter coefficient of 31
#define BMP388_IIR_FILTER_COEF_63 ((uint8_t)0x0C)  // 0b 0000 1100 = filter coefficient of 63
#define BMP388_IIR_FILTER_COEF_127 ((uint8_t)0x0E) // 0b 0000 1110 = filter coefficient of 127
/**
 * @}
 */

/**
 * @defgroup BMP388_OUTPUT_COMPENSATION_REGISTERS
 * @note Read Only
 * @note Default values set at factory
 * @note Registers 0x30 to 0x57 - Calibration Data
 * @note See section 3.11.1 - Memory Map Trimming Coefficients for more info
 * @{
 */
#define BMP388_NVM_PAR_T1_7_0_ADDR 0x31 // 16 bit unsigned
#define BMP388_NVM_PAR_T1_15_8_ADDR 0x32
#define BMP388_NVM_PAR_T2_7_0_ADDR 0x33 // 16 bit unsigned
#define BMP388_NVM_PAR_T2_15_8_ADDR 0x34
#define BMP388_NVM_PAR_T3_7_0_ADDR 0x35 // 8 bit signed
#define BMP388_NVM_PAR_P1_7_0_ADDR 0x36 // 16 bit signed
#define BMP388_NVM_PAR_P1_15_8_ADDR 0x37
#define BMP388_NVM_PAR_P2_7_0_ADDR 0x38 // 16 bit signed
#define BMP388_NVM_PAR_P2_15_8_ADDR 0x39
#define BMP388_NVM_PAR_P3_7_0_ADDR 0x3A // 8 bit signed
#define BMP388_NVM_PAR_P4_7_0_ADDR 0x3B // 8 bit signed
#define BMP388_NVM_PAR_P5_7_0_ADDR 0x3C // 16 bit unsigned
#define BMP388_NVM_PAR_P5_15_8_ADDR 0x3D
#define BMP388_NVM_PAR_P6_7_0_ADDR 0x3E // 16 bit unsigned
#define BMP388_NVM_PAR_P6_15_8_ADDR 0x3F
#define BMP388_NVM_PAR_P7_7_0_ADDR 0x40 // 8 bit signed
#define BMP388_NVM_PAR_P8_7_0_ADDR 0x41 // 8 bit signed
#define BMP388_NVM_PAR_P9_7_0_ADDR 0x42 // 16 bit signed
#define BMP388_NVM_PAR_P9_15_8_ADDR 0x43
#define BMP388_NVM_PAR_P10_7_0_ADDR 0x44 // 8 bit signed
#define BMP388_NVM_PAR_P11_7_0_ADDR 0x45 // 8 bit signed
/**
 * @}
 */

/**
 * @defgroup BMP388_COMMAND_REGISTER
 * @note Read/Write
 * @note Default value of 0x00
 * @note Bit 7 to 0: cmd - all available commands - see datasheet
 * @{
 */
#define BMP388_CMD_ADDR 0x7E
#define BMP388_CMD_NO_OPERATION ((uint8_t)0x00) // 0b 0000 0000 = reserved - no command
#define BMP388_CMD_FIFO_FLUSH ((uint8_t)0xB0)   // 0b 1011 0000 = clear FIFO, does not change FIFO_CONFIG registers
#define BMP388_CMD_SOFTRESET ((uint8_t)0xB6)    // 0b 1011 0110 = softreset, all config settings set to default
    /**
     * @}
     */

    /**   BMP388_Exported_Constants
     * @}
     */

    /**   BMP388
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif /* __BMP388_H */
