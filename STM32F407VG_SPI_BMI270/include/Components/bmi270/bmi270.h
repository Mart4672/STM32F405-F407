#ifndef BMI270_IMU_H
#define BMI270_IMU_H

#include "stm32f4xx_hal.h"

// Conversion defines
#define EARTH_G_TO_METER_PER_SECOND 9.81f
#define DEGREES_TO_RADIANS          0.01745329251f
#define RADIANS_TO_DEGREES          57.29577951f

// Register defines

#define BMI270_CHIP_ID_ADDR         0x00
#define BMI_ACC_DATA_ADDR           0x0C
#define BMI_GYR_DATA_ADDR           0x12
#define BMI_INT_STATUS_1_ADDR       0x1D
#define BMI_INTERNAL_STATUS_ADDR    0x21
#define BMI_TEMP_DATA_ADDR          0x22
// TODO add a function that reads the GYR_CAS
// TODO see if GYR_CAS needs to be read each time
#define BMI_GYR_CAS_ADDR            0x3C
#define BMI_ACC_CONF_ADDR           0x40
#define BMI_ACC_RANGE_ADDR          0x41
#define BMI_GYR_CONF_ADDR           0x42
#define BMI_GYR_RANGE_ADDR          0x43
#define BMI_SATURATION_ADDR         0x4A
#define BMI_INT1_IO_CONF_ADDR       0x53
#define BMI_INT2_IO_CONF_ADDR       0x54
#define BMI_INT_LATCH_ADDR          0x55
#define BMI_INT1_INT2_MAP_DATA_ADDR 0x58
#define BMI_INIT_CTRL_ADDR          0x59
#define BMI_INIT_DATA_ADDR          0x5E
#define BMI_PWR_CONF_ADDR           0x7C
#define BMI_PWR_CTRL_ADDR           0x7D
#define BMI_CMD_ADDR                0x7E

// Register (configuration) value defines

// BMI270 static chip id
#define BMI270_CHIP_ID 0x24

// BMI270 INTERNAL_STATUS.message value for ASIC initialized
#define BMI_INTERNAL_STATUS_INIT_OK 0x01

/**
 *  BMI270 accelerometer configuration value
 ******************************************************************************
 *  bits 3 to 0: acc_odr - output data rate in Hz
 *   0x00 - reserved
 *   0x01 - odr_0p78 - 25/32 Hz output data rate
 *   0x02 - odr_1p5  - 25/16 Hz output data rate
 *   0x03 - odr_3p1  - 25/8 Hz output data rate
 *   0x04 - odr_6p25 - 25/4 Hz output data rate
 *   0x05 - odr_12p5 - 25/2 Hz output data rate
 *   0x06 - odr_25   - 25 Hz output data rate
 *   0x07 - odr_50   - 50 Hz output data rate
 *   0x08 - odr_100  - 100 Hz output data rate (default)
 *   0x09 - odr_200  - 200 Hz output data rate
 *   0x0a - odr_400  - 400 Hz output data rate
 *   0x0b - odr_800  - 800 Hz output data rate
 *   0x0c - odr_1k6  - 1600 Hz output data rate
 *   0x0d - odr_3k2  - reserved
 *   0x0e - odr_6k4  - reserved
 *   0x0f - odr_12k8 - reserved
 ******************************************************************************
 *  bits 6 to 4: acc_bwp - filter bandwidth
 *  SENSOR BEHAVIOR FOR THIS SECTION DEPENDS ON BIT 7
 *  (hp mode behavior)_(ulp mode behavior)
 *   0x00 - osr4_avg1        - oversample x4 OR no averaging
 *   0x01 - osr2_avg2        - oversample x2 OR average 2 samples
 *   0x02 - norm_avg4        - oversample x1 OR average 4 samples (default)
 *   0x03 - cic_avg8         - no oversampling OR average 8 samples
 *   0x04 - reserved_avg16   - reserved OR average 16 samples
 *   0x05 - reserved_avg32   - reserved OR average 32 samples
 *   0x06 - reserved_avg64   - reserved OR average 64 samples
 *   0x07 - reserved_avg128  - reserved OR average 128 samples
 ******************************************************************************
 *  bit 7: acc_filter_perf - accelerometer filter mode
 *   0x00 - ulp - power optimized
 *   0x01 - hp  - performance optimized (default)
 */
#define BMI_ACC_CONF ((0x01 << 7) | (0x00 << 4) | (0x08))

/**
 *  BMI270 accelerometer range value
 ******************************************************************************
 *  bits 1 to 0: acc_range - accelerometer g-range
 *   0x00 - range_2g    - acceleration detection range of +/- 2g
 *   0x01 - range_4g    - acceleration detection range of +/- 4g
 *   0x02 - range_8g    - acceleration detection range of +/- 8g (default)
 *   0x03 - range_16g   - acceleration detection range of +/- 16g
 */
#define BMI_ACC_RANGE 0x00

/**
 *  BMI270 gyroscpe configuration value
 ******************************************************************************
 *  bits 3 to 0: gyr_odr - output data rate in Hz
 *   0x00 - reserved
 *   0x01 - odr_0p78 - 25/32 Hz output data rate
 *   0x02 - odr_1p5  - 25/16 Hz output data rate
 *   0x03 - odr_3p1  - 25/8 Hz output data rate
 *   0x04 - odr_6p25 - 25/4 Hz output data rate
 *   0x05 - odr_12p5 - 25/2 Hz output data rate
 *   0x06 - odr_25   - 25 Hz output data rate
 *   0x07 - odr_50   - 50 Hz output data rate
 *   0x08 - odr_100  - 100 Hz output data rate
 *   0x09 - odr_200  - 200 Hz output data rate (default)
 *   0x0a - odr_400  - 400 Hz output data rate
 *   0x0b - odr_800  - 800 Hz output data rate
 *   0x0c - odr_1k6  - 1600 Hz output data rate
 *   0x0d - odr_3k2  - 3200 Hz output data rate
 *   0x0e - odr_6k4  - reserved
 *   0x0f - odr_12k8 - reserved
 ******************************************************************************
 *  bits 5 to 4: gyr_bwp - filter bandwidth
 *   0x00 - osr4        - oversample x4
 *   0x01 - osr2        - oversample x2
 *   0x02 - norm        - no oversampling (default)
 *   0x03 - res         - reserved
 ******************************************************************************
 *  bit 6: gyr_noise_perf - gyroscope noise performance
 *   0x00 - ulp - power optimized (default)
 *   0x01 - hp  - performance optimized
 ******************************************************************************
 *  bit 7: gyr_filter_perf - gyroscope filter mode
 *   0x00 - ulp - power optimized
 *   0x01 - hp  - performance optimized (default)
 */
#define BMI_GYR_CONF ((0x01 << 7) | (0x01 << 6) | (0x00 << 4) | (0x09))

/**
 *  BMI270 gyroscope range value
 ******************************************************************************
 *  bits 2 to 0: gyr_range - gyroscope angular rate range
 *   0x00 - range_2000  - range of +/- 2000°/s (dps), 16.4 LSB/dps (default)
 *   0x01 - range_1000  - range of +/- 1000°/s (dps), 32.8 LSB/dps
 *   0x02 - range_500   - range of +/- 500°/s (dps), 65.6 LSB/dps
 *   0x03 - range_250   - range of +/- 250°/s (dps), 131.2 LSB/dps
 *   0x04 - range_125   - range of +/- 125°/s (dps), 262.4 LSB/dps
 ******************************************************************************
 *  bit 3: ois_range - angular rate range for Optical Image Stabilization (OIS)
 *   0x00 - range_250   - range of +/- 250°/s (dps), 131.2 LSB/dps (default)
 *   0x01 - range_2000  - range of +/- 2000°/s (dps), 16.4 LSB/dps
 */
#define BMI_GYR_RANGE ((0x00 << 3) | (0x01))

/**
 *  BMI270 interrupt 1 IO configuration value
 ******************************************************************************
 *  bit 1: lvl - output level of INT1 pin
 *   0x00 - active_low  - pin is pulled low when interrupt is activated (default)
 *   0x01 - active_high - pin is pulled high when interrupt is activated
 ******************************************************************************
 *  bit 2: od - output behavior of INT1 pin
 *   0x00 - push_pull (default)
 *   0x01 - open_drain
 ******************************************************************************
 *  bit 3: output_en - output enable for INT1 pin
 *   0x00 - off     - interrupt output disabled (default)
 *   0x01 - on      - interrupt output enabled
 ******************************************************************************
 *  bit 4: input_en - input enable for INT1 pin
 *   0x00 - off     - interrupt input disabled (default)
 *   0x01 - on      - interrupt input enabled
 */
#define BMI_INT1_IO_CONF ((0x00 << 4) | (0x01 << 3) | (0x00 << 2) | (0x01 << 1))

/**
 *  BMI270 interrupt 2 IO configuration value
 ******************************************************************************
 *  bit 1: lvl - output level of INT2 pin
 *   0x00 - active_low  - pin is pulled low when interrupt is activated (default)
 *   0x01 - active_high - pin is pulled high when interrupt is activated
 ******************************************************************************
 *  bit 2: od - output behavior of INT2 pin
 *   0x00 - push_pull (default)
 *   0x01 - open_drain
 ******************************************************************************
 *  bit 3: output_en - output enable for INT2 pin
 *   0x00 - off     - interrupt output disabled (default)
 *   0x01 - on      - interrupt output enabled
 ******************************************************************************
 *  bit 4: input_en - input enable for INT2 pin
 *   0x00 - off     - interrupt input disabled (default)
 *   0x01 - on      - interrupt input enabled
 */
#define BMI_INT2_IO_CONF ((0x00 << 4) | (0x01 << 3) | (0x00 << 2) | (0x01 << 1))

/**
 *  BMI270 interrupt latching configuration value
 *  Latched = interrupt pin stays on until BMI_INT_STATUS_1_ADDR is read
 *  NOT Latched = interrupt pin automatically turns off after 1/6400 s
 *
 ******************************************************************************
 *  bit 0: int_latch - latched/non-latched interrupt mode select
 *   0x00 - none        - NOT Latched (default)
 *   0x01 - permanent   - Latched
 */
#define BMI_INT_LATCH 0x00

/**
 *  BMI270 interrupt 1/2 mapping configuration value
 ******************************************************************************
 *  bit 0: ffull_int1 - FIFO Full interrupt mapped to INT1
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 1: fwm_int1 - FIFO Watermark interrupt mapped to INT1
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 2: drdy_int1 - Data Ready interrupt mapped to INT1
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 3: err_int1 - Error interrupt mapped to INT1
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 4: ffull_int2 - FIFO Full interrupt mapped to INT2
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 5: fwm_int2 - FIFO Watermark interrupt mapped to INT2
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 6: drdy_int2 - Data Ready interrupt mapped to INT2
 *   0x00 / 0x01 - not mapped (default) / mapped
 ******************************************************************************
 *  bit 7: err_int2 - Error interrupt mapped to INT2
 *   0x00 / 0x01 - not mapped (default) / mapped
 */
#define BMI_INT1_INT2_MAP_DATA (0x01 << 2)

// BMI270 start initialization value
#define BMI_INIT_CTRL_PREPARE 0x00

// BMI270 finish initialization value
#define BMI_INIT_CTRL_COMPLETE 0x01

/**
 *  BMI270 power configuration value
 ******************************************************************************
 *  bit 0: adv_power_save - enable advanced power save
 *   0x00 - aps_off - advanced power save disabled
 *   0x01 - aps_on  - advanced power save enabled (default)
 ******************************************************************************
 *  bit 1: fifo_self_wake_up - enable FIFO read in low power mode
 *   0x00 - fsw_off - FIFO read disabled in low power mode (LPM)
 *   0x01 - fsw_on  - read enabled in LPM after FIFO interrupt fired (default)
 ******************************************************************************
 *  bit 2: fup_en - fast power up enable
 *   0x00 - fup_off - fast power up disabled (default)
 *   0x01 - fup_on  - fast power up enabled
 */
#define BMI_PWR_CONF ((0x00 << 2) | (0x00 << 1) | (0x00))

/**
 *  BMI270 power control value
 ******************************************************************************
 *  bit 0: aux_en - enable the auxiliary sensor
 *   0x00 / 0x01 - disabled (default) / enabled
 ******************************************************************************
 *  bit 1: gyr_en - enable the gyroscope
 *   0x00 / 0x01 - disabled (default) / enabled
 ******************************************************************************
 *  bit 2: acc_en - enable the accelerometer
 *   0x00 / 0x01 - disabled (default) / enabled
 ******************************************************************************
 *  bit 3: temp_en - enable the temperature sensor
 *   0x00 / 0x01 - disabled (default) / enabled
 */
#define BMI_PWR_CTRL ((0x01 << 3) | (0x01 << 2) | (0x01 << 1) | (0x00))

// BMI270 soft reset command value
#define BMI_CMD_SOFTRESET 0xB6

typedef struct
{
    // SPI
    SPI_HandleTypeDef* spiHandle;
    GPIO_TypeDef* chipSelectPinBank;
    uint16_t chipSelectPin;

    // DMA
    uint8_t readingAcc;
    uint8_t readingGyr;
    uint8_t accTxBuf[8];
    uint8_t gyrTxBuf[8];
    volatile uint8_t accRxBuf[8];
    volatile uint8_t gyrRxBuf[8];

    // Conversion constants (raw to m/s^2 and raw to rad/s)
    float accConversion;
    float gyrConversion;

    // x-y-z measurements
    float acc_mps2[3];
    float gyr_rps[3];

    // Configuration values
    uint16_t configFileSize;
    uint8_t pwrConf;
    uint8_t pwrCtrl;
    uint8_t accConf;
    uint8_t accRange;
    uint8_t gyrConf;
    uint8_t gyrRange;
    uint8_t gyrRangeBits2To0;
    uint8_t int1IOConf;
    uint8_t int1Int2MapData;
    uint8_t intLatch;
} BMI270;

/**
 * @brief Initializes a BMI270 IMU with the specified parameters.
 * SPI parameters are stored and the SPI is activated for the sensor.
 * The accelerometer and gyroscope are configured for use.
 * An interrupt pin is also configured.
 * @param imu BMI270 struct that stores instance-specific data
 * @param spiHandle SPI handle for the specific BMI270 being used
 * @param chipSelectPinBank chip select bank for the BMI270 in use
 * @param chipSelectPin chip select pin for the BMI270 being used
 * @return uint8_t (1 on success)
 */
uint8_t BMI270_Init(BMI270* imu, SPI_HandleTypeDef* spiHandle, GPIO_TypeDef* chipSelectPinBank, uint16_t chipSelectPin);

/**
 * @brief Low level register function that reads a BMI270 register
 * @note Register reads for the BMI270 are as follows:
 *  Send 1 byte          |   Read 1 dummy byte   |   Read true data
 *  transaction byte 1   |   transaction byte 2  |   transaction byte 3+
 * @param imu BMI270 struct
 * @param regAddr BMI270 register address to read
 * @param data pointer to store the read data with
 * @return uint8_t (1 on success)
 */
uint8_t BMI270_ReadRegister(BMI270* imu, uint8_t regAddr, uint8_t* data);

/**
 * @brief Low level register function that writes to a BMI270 register
 * @note Register writes for the BMI270 are as follows:
 *  Send 1 byte         |   write true data
 *  transaction byte 1  |   transaction byte 2+
 * @param imu BMI270 struct
 * @param regAddr BMI270 register address to write to
 * @param data pointer to store the data to write
 * @return uint8_t (1 on success)
 */
uint8_t BMI270_WriteRegister(BMI270* imu, uint8_t regAddr, uint8_t data);

// TODO determine if acceleration readings from this function remain at the max/min value when the sensor is saturated
// or if the values overflow if overflow occurs, use the saturation register in the Read functions to determine when
// this happens so that the final value can be overriden
/**
 * @brief High level Polling function to gets x/y/z accelerations in m/s^2
 *
 * @param imu BMI270 struct
 * @return uint8_t (1 on success)
 */
uint8_t BMI270_ReadAccelerometer(BMI270* imu);

/**
 * @brief High level Polling function to gets x/y/z angular rates in rad/s
 *
 * @param imu BMI270 struct
 * @return uint8_t (1 on success)
 */
uint8_t BMI270_ReadGyroscope(BMI270* imu);

/**
 * @brief High level Polling function that attempts the read the accelerometer data using Direct Memory Access (DMA)
 *
 * @param imu BMI270 struct
 * @return uint8_t (returns 1 if HAL_SPI_TransmitReceive_DMA was successful, 0 otherwise)
 */
uint8_t BMI270_ReadAccelerometerDMA(BMI270* imu);

/**
 * @brief Function that completes a successful accelerometer Direct Memory Access (DMA) read.
 * @note if BMI270_ReadAccelerometerDMA returns 1, call this function to save the readings in the BMI270 struct in m/s^2
 *
 * @param imu BMI270 struct
 */
void BMI270_ReadAccelerometerDMA_Complete(BMI270* imu);

/**
 * @brief High level Polling function that attempts the read the gyroscope data using Direct Memory Access (DMA)
 *
 * @param imu BMI270 struct
 * @return uint8_t (returns 1 if HAL_SPI_TransmitReceive_DMA was successful, 0 otherwise)
 */
uint8_t BMI270_ReadGyroscopeDMA(BMI270* imu);

/**
 * @brief Function that completes a successful gyroscope Direct Memory Access (DMA) read.
 * @note if BMI270_ReadGyroscopeDMA returns 1, call this function to save the readings in the BMI270 struct in rad/s
 *
 * @param imu BMI270 struct
 */
void BMI270_ReadGyroscopeDMA_Complete(BMI270* imu);

#endif
