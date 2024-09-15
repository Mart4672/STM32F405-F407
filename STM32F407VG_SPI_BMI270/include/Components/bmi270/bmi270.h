#ifndef BMI270_IMU_H
#define BMI270_IMU_H

#include "stm32f4xx_hal.h"

// Register defines
#define BMI270_CHIP_ID_ADDR     0x00
#define BMI_ACC_DATA 			0x0C
#define BMI_GYR_DATA			0x12
#define BMI_INT_STATUS_1 		0x1D
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define	BMI_GYR_CONF		    0x42
#define	BMI_GYR_RANGE			0x43
#define	BMI_SATURATION			0x4A
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT2_IO_CONF 	   	0x54
#define BMI_INT_LATCH 	        0x55
#define BMI_INT1_INT2_MAP_DATA 	0x58
#define BMI_PWR_CONF 		    0x7C
#define BMI_PWR_CTRL 		    0x7D
#define BMI_CMD 		        0x7E

// TODO add additional command defines
// Register value defines
#define BMI_SOFTRESET_CMD       0xB6
#define BMI270_CHIP_ID          0x24

typedef struct {
	// SPI
	SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef 	  *chipSelectPinBank;
	uint16_t 		   chipSelectPin;

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
    // TODO add configuration values

} BMI270;

/**
 * @brief Initializes a BMI270 IMU with the specified parameters.
 * SPI parameters are stored and the SPI is activated for the sensor.
 * The accelerometer and gyroscope are configured for use.
 * An interrupt pin is also configured. 
 * A nonzero value is returned if any register transactions failed.
 * @param imu BMI270 struct that stores instance-specific data
 * @param spiHandle SPI handle for the specific BMI270 being used
 * @param chipSelectPinBank chip select bank for the BMI270 in use
 * @param chipSelectPin chip select pin for the BMI270 being used 
 * @return uint8_t
 */
uint8_t BMI270_Init(BMI270 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *chipSelectPinBank, uint16_t chipSelectPin);

/**
 * @brief Low level register function that reads a BMI270 register
 * @note Register reads for the BMI270 are as follows:
 *  Send 1 byte          |   Read 1 dummy byte   |   Read true data
 *  transaction byte 1   |   transaction byte 2  |   transaction byte 3+
 * @param imu BMI270 struct
 * @param regAddr BMI270 register address to read
 * @param data pointer to store the read data with
 * @return uint8_t (nonzero if any register transactions fail)
 */
uint8_t BMI270_ReadRegister(BMI270 *imu, uint8_t regAddr, uint8_t *data);

/**
 * @brief Low level register function that writes to a BMI270 register
 * @note Register writes for the BMI270 are as follows:
 *  Send 1 byte         |   write true data
 *  transaction byte 1  |   transaction byte 2+
 * @param imu BMI270 struct
 * @param regAddr BMI270 register address to write to
 * @param data pointer to store the data to write
 * @return uint8_t (nonzero if any register transactions fail)
 */
uint8_t BMI270_WriteRegister(BMI270 *imu, uint8_t regAddr, uint8_t data);

// TODO determine if acceleration readings from this function remain at the max/min value when the sensor is saturated or if the values overflow
// if overflow occurs, use the saturation register in the Read functions to determine when this happens so that the final value can be overriden
/**
 * @brief High level Polling function to gets x/y/z accelerations in m/s^2
 * 
 * @param imu BMI270 struct
 * @return uint8_t (nonzero if any register transactions fail)
 */
uint8_t BMI270_ReadAccelerometer(BMI270 *imu);

/**
 * @brief High level Polling function to gets x/y/z angular rates in rad/s
 * 
 * @param imu BMI270 struct
 * @return uint8_t (nonzero if any register transactions fail)
 */
uint8_t BMI270_ReadGyroscope(BMI270 *imu);

/**
 * @brief High level Polling function that attempts the read the accelerometer data using Direct Memory Access (DMA)
 * 
 * @param imu BMI270 struct
 * @return uint8_t (returns 1 if HAL_SPI_TransmitReceive_DMA was successful, 0 otherwise)
 */
uint8_t BMI270_ReadAccelerometerDMA(BMI270 *imu);

/**
 * @brief Function that completes a successful accelerometer Direct Memory Access (DMA) read.
 * @note if BMI270_ReadAccelerometerDMA returns 1, call this function to save the readings in the BMI270 struct in m/s^2
 * 
 * @param imu BMI270 struct
 */
void BMI270_ReadAccelerometerDMA_Complete(BMI270 *imu);

/**
 * @brief High level Polling function that attempts the read the gyroscope data using Direct Memory Access (DMA)
 * 
 * @param imu BMI270 struct
 * @return uint8_t (returns 1 if HAL_SPI_TransmitReceive_DMA was successful, 0 otherwise)
 */
uint8_t BMI270_ReadGyroscopeDMA(BMI270 *imu);

/**
 * @brief Function that completes a successful gyroscope Direct Memory Access (DMA) read.
 * @note if BMI270_ReadGyroscopeDMA returns 1, call this function to save the readings in the BMI270 struct in rad/s
 * 
 * @param imu BMI270 struct
 */
void BMI270_ReadGyroscopeDMA_Complete(BMI270 *imu);

#endif
