/**
 ******************************************************************************
 * @file    STM32F407VG_SPI_BMI270/src/main.c
 * @author  Redstone Space Systems
 * @brief   This code shows how to use the STM32F407 (or STM32F405) to read
 * data from the BMI270 barometric pressure sensor using the SPI HAL API
 * and send the data to a connected computer over serial using the UART HAL API
 * @remark STM32CubeF4 examples:
 * https://github.com/STMicroelectronics/STM32CubeF4.git
 * @remark The following STM32CubeF4 examples were useful for this project:
 * @remark UART functionality -
 * Projects/STM32F4-Discovery/Examples/UART/UART_TwoBoards_ComPolling/
 * @remark SPI functionality -
 * Projects/STM32F4-Discovery/Examples/SPI/SPI_FullDuplex_AdvComPolling/Master
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
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
 * @{
 */

/** @addtogroup UART_TwoBoards_ComPolling
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// UART handler declaration
UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;

static SPI_HandleTypeDef hspi1;

// static SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim5;

BMI270 imu1;

BMP388_InitializationConfig bmp388initConfig;

BMP388_InterruptConfig bmp388interruptConfig;

uint8_t bmi270readAccel = 0;

uint8_t bmi270reset = 0;

uint8_t bmi270readGyro = 0;

uint8_t bmp388readData = 0;

uint32_t timer_val;

static const uint8_t waitForUartTransmit = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void USART_UART_Init(void);
static void MX_TIM5_Init(void);
static void SPI_Init(SPI_HandleTypeDef* hspi);
// static uint8_t SPI_WriteRead(SPI_HandleTypeDef* hspi, uint8_t Byte);
// static void SPI_Error(SPI_HandleTypeDef* hspi);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{

    /**
     * STM32F4xx HAL library initialization:
     * - Configure the Flash prefetch, instruction and Data caches
     * - Configure the Systick to generate an interrupt each 1 msec
     * - Set NVIC Group Priority to 4
     * - Global MSP (MCU Support Package) initialization
     */
    HAL_Init();

    /* Configure LED3, LED4, LED5 and LED6 */
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);

    /* Configure the system clock to 168 MHz */
    SystemClock_Config();

    // configure timer 5 (32 bit timer)
    // see STM32 DS8626 - Section 2.2.21 - Table 4. Timer feature comparison
    __HAL_RCC_TIM5_CLK_ENABLE();
    MX_TIM5_Init();
    // start timer 5
    if (HAL_TIM_Base_Start(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    // use HAL_TIM_PeriodElapsedCallback with HAL_TIM_Base_Start_IT to run a callback at a specific interval

    // Initialize SPI1
    hspi1.Instance = SPI1;
    SPI_Init(&hspi1);

    // Initialize BMI270 Chip Select
    BMI270_IO_Init();

    // Set config values for this BMI270 instance
    imu1.pwrConf = BMI_PWR_CONF;
    imu1.pwrCtrl = BMI_PWR_CTRL;
    imu1.accConf = BMI_ACC_CONF;
    imu1.accRange = BMI_ACC_RANGE;
    imu1.gyrConf = BMI_GYR_CONF;
    imu1.gyrRange = BMI_GYR_RANGE;
    imu1.int1IOConf = BMI_INT1_IO_CONF;
    imu1.int1Int2MapData = BMI_INT1_INT2_MAP_DATA;
    imu1.intLatch = BMI_INT_LATCH;

    uint8_t initializationSuccessImu1 = 1;
    initializationSuccessImu1 = BMI270_Init(&imu1, &hspi1, BMI270_CS_GPIO_PORT, BMI270_CS_PIN);

    if (!initializationSuccessImu1)
    {
        // softreset the imu
        BMI270_WriteRegister(&imu1, BMI_CMD_ADDR, BMI_CMD_SOFTRESET);
        HAL_Delay(50);
        // try to initialize the device again
        initializationSuccessImu1 = BMI270_Init(&imu1, &hspi1, BMI270_CS_GPIO_PORT, BMI270_CS_PIN);
    }

    HAL_Delay(50);

    // Initialize BMI270 Interrupt Pin
    BMI270_IO_ITConfig();

    /*#### Configure the UART peripheral #####################################*/
    USART_UART_Init();

    char buf1[100];
    int len1;
    len1 = sprintf(buf1, "BMI270_Init returned %u\r\n\n\n", initializationSuccessImu1);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)buf1, len1, 100);

    /*#### Configure the BMP388 Device ######################################*/

    // TODO initialize BMP388 with configurable SPI handle
    // initial BMP388 settings to save to the device
    bmp388initConfig.Communication_Mode = BMP388_SERIAL_INTERFACE_4WIRE; // default
    bmp388initConfig.Sensor_Measurement_Mode = BMP388_MEASUREMENT_CONFIG_TEMP_AND_PRESSURE;
    bmp388initConfig.Sensor_Power_Mode = BMP388_MEASUREMENT_CONFIG_NORMAL_MODE;
    bmp388initConfig.Oversampling_Rate_Pressure = BMP388_PRESSURE_OSR_X16;
    bmp388initConfig.Oversampling_Rate_Temperature = BMP388_TEMPERATURE_OSR_X2;
    bmp388initConfig.Output_DataRate = BMP388_ODR_25_HZ;        // see section 3.9.2 for maximum ODR based on OSR values
    bmp388initConfig.Filter_Setting = BMP388_IIR_FILTER_COEF_0; // BMP388_IIR_FILTER_COEF_3

    BMP388_Initialize(&bmp388initConfig);

    // initial BMP388 interrupt settings to save to the device
    bmp388interruptConfig.Output_Type = BMP388_INTERRUPT_OUTOUT_PUSH_PULL; // defualt
    bmp388interruptConfig.Active_Level = BMP388_INTERRUPT_ACTIVE_HIGH;     // default
    bmp388interruptConfig.Latching = BMP388_INTERRUPT_LATCHING_ENABLED;
    bmp388interruptConfig.Fifo_Watermark = BMP388_INTERRUPT_FIFO_WATERMARK_DISABLED;
    bmp388interruptConfig.Fifo_Full = BMP388_INTERRUPT_FIFO_FULL_DISABLED;
    bmp388interruptConfig.Data_Ready = BMP388_INTERRUPT_DATA_READY_ENABLED;

    BMP388_ConfigureInterrupt(&bmp388interruptConfig);

    // HAL_GetTick() should return the number of milliseconds elapsed since startup

    timer_val = __HAL_TIM_GET_COUNTER(&htim5);
    uint32_t printTime = 0;
    uint32_t totalPrintTime = 0;
    uint32_t interruptStatusTime = 0;
    uint32_t imu1AccelTime = 0;
    uint32_t imu1GyroTime = 0;
    float pressure = 0.0f;
    float temp = 0.0f;
    char buf3[100];
    int len3;
    char buf4[150];
    int len4;
    uint16_t numReadDataCalls = 0;
    uint16_t numAccelReadDataCalls = 0;
    uint32_t bmiResetCount = 0;
    uint16_t numGyroReadDataCalls = 0;

    HAL_Delay(1000);

    len3 = sprintf(buf3, "timer_val is %lu\r\n\n\n", timer_val);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)buf3, len3, 100);

    while (1)
    {
        // BMI270 reset task
        if (bmi270reset)
        {
            bmi270reset = 0;
            HAL_NVIC_DisableIRQ((IRQn_Type)BMI270_INT1_EXTI_IRQn);

            // softreset the imu
            BMI270_WriteRegister(&imu1, BMI_CMD_ADDR, BMI_CMD_SOFTRESET);
            HAL_Delay(1);
            // try to initialize the device again
            initializationSuccessImu1 = BMI270_Init(&imu1, &hspi1, BMI270_CS_GPIO_PORT, BMI270_CS_PIN);

            HAL_NVIC_EnableIRQ((IRQn_Type)BMI270_INT1_EXTI_IRQn);
            bmiResetCount++;
        }

        // read BMI270 accelerometer data task
        if ((bmi270readAccel == 1) && (timer_val - imu1AccelTime) >= READ_BMI270_ACCEL_TASK_PERIOD_MICROSECONDS)
        {
            bmi270readAccel = 0;
            imu1AccelTime = __HAL_TIM_GET_COUNTER(&htim5);

            // read calibrated sensor values
            BMI270_ReadAccelerometer(&imu1);

            numAccelReadDataCalls++;
        }

        // read BMI270 gyroscope data task
        if ((bmi270readGyro == 1) && (timer_val - imu1GyroTime) >= READ_BMI270_GYRO_TASK_PERIOD_MICROSECONDS)
        {
            bmi270readGyro = 0;
            imu1GyroTime = __HAL_TIM_GET_COUNTER(&htim5);

            // read calibrated sensor values
            BMI270_ReadGyroscope(&imu1);

            numGyroReadDataCalls++;
        }

        // read BMP388 data task
        if ((bmp388readData == 1) && (timer_val - interruptStatusTime) >= READ_BMP388_TASK_PERIOD_MICROSECONDS)
        {
            bmp388readData = 0;
            interruptStatusTime = __HAL_TIM_GET_COUNTER(&htim5);

            // read the interrupt status register to de-asssert the sensor interrupt pin
            BMP388_ReadIntStatus();

            // read calibrated sensor values
            BMP388_GetCalibratedData(&pressure, &temp);

            numReadDataCalls++;
        }

        // UART print task
        if ((timer_val - printTime) >= UART_PRINT_TASK_PERIOD_MICROSECONDS)
        {
            printTime = __HAL_TIM_GET_COUNTER(&htim5);

            // len4 = snprintf(buf4, 103, "[%10lu] Data reads: %3u, pressure: %6d Pa, temp: (%5d/100) C, print task time: %6lu us\r\n\n\n", timer_val, numReadDataCalls, (int)pressure, (int)(temp * 100.f), totalPrintTime);
            len4 = snprintf(buf4, 145, "[%04lus]\r\n[Accel] Reads: %3u, x/y/z: %3d/%3d/%3d m/s/10\r\n[Gyro] Reads: %3u, x/y/z: %5d/%5d/%5d deg/s, resets: %lu, printT: %5lu us\r\n", timer_val/1000000, numAccelReadDataCalls, (int)(imu1.acc_mps2[0] * 10.f), (int)(imu1.acc_mps2[1] * 10.f), (int)(imu1.acc_mps2[2] * 10.f), numGyroReadDataCalls, (int)(imu1.gyr_rps[0] * RADIANS_TO_DEGREES), (int)(imu1.gyr_rps[1] * RADIANS_TO_DEGREES), (int)(imu1.gyr_rps[2] * RADIANS_TO_DEGREES), bmiResetCount, totalPrintTime);
            // HAL_UART_Transmit(&UartHandle, (uint8_t *)buf4, len4, 150);
            HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)buf4, len4);

            if (waitForUartTransmit)
            {
                // wait until the transfer is complete if desired
                while (UartReady != SET)
                {
                }
                UartReady = RESET;
            }

            numReadDataCalls = 0;
            numAccelReadDataCalls = 0;
            numGyroReadDataCalls = 0;

            // time for UART Task to run
            totalPrintTime = __HAL_TIM_GET_COUNTER(&htim5) - printTime;
        }

        // update timer_val
        timer_val = __HAL_TIM_GET_COUNTER(&htim5);
    }

    /* Infinite loop */
    while (1)
    {
    }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 168000000
 *            HCLK(Hz)                       = 168000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 336
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    /* STM32F405x/407x/415x/417x Revision Z and upper devices: prefetch is supported  */
    if (HAL_GetREVID() >= 0x1001)
    {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI270_INT1_PIN)
    {
        /* Toggle LED5 */
        BSP_LED_Toggle(LED5);

        // read BMI270 INT_STATUS_1 register
        uint8_t bmi270IntStatus1Value;
        uint8_t bmi270ReadSuccess = BMI270_ReadRegister(&imu1, BMI_INT_STATUS_1_ADDR, &bmi270IntStatus1Value);
        if (!bmi270ReadSuccess)
        {
            // Try restting the BMI
            // bmi270reset = 1;
            // Error_Handler();
        }

        // Check if any data ready interrupt bits are set

        // check INT_STATUS_1 bit 6 - gyroscope data ready interrupt
        if ((bmi270IntStatus1Value & 0x40))
        {
            // Set flag for reading data registers
            bmi270readGyro = 1;
        }

        // check INT_STATUS_1 bit 7 - accelerometer data ready interrupt
        if ((bmi270IntStatus1Value & 0x80))
        {
            // Set flag for reading data registers
            bmi270readAccel = 1;
        }
    }
    if (GPIO_Pin == BMP388_INT1_PIN) // BMP388_INT1_PIN
    {
        /* Toggle LED4 */
        BSP_LED_Toggle(LED4);

        // set BMP388 read data registers flag
        bmp388readData = 1;
    }
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{
    htim5.Instance = TIM5;
    // set the timer prescaler to PLLN/PLLP
    htim5.Init.Prescaler = 84 - 1; // (HAL_RCC_GetPCLK1Freq()/2) / 1000000 - 1
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4294967295;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  SPI Initialization Function
 */
static void SPI_Init(SPI_HandleTypeDef* hspi)
{
    if (HAL_SPI_GetState(hspi) == HAL_SPI_STATE_RESET)
    {
        // Common SPI Configuration
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        hspi->Init.Direction = SPI_DIRECTION_2LINES;
        hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
        hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        hspi->Init.CRCPolynomial = 7;
        hspi->Init.DataSize = SPI_DATASIZE_8BIT;
        hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
        hspi->Init.NSS = SPI_NSS_SOFT;
        hspi->Init.TIMode = SPI_TIMODE_DISABLED;
        hspi->Init.Mode = SPI_MODE_MASTER;

        // HAL_SPI_Init internally calls HAL_SPI_MspInit in stm32f4xx_hal_msp.c
        HAL_SPI_Init(hspi);
    }
}

/**
 * @brief  Sends a Byte through the SPI interface and return the Byte received
 *         from the SPI bus.
 * @param  Byte: Byte send.
 * @retval The received byte value
 */
// static uint8_t SPI_WriteRead(SPI_HandleTypeDef* hspi, uint8_t Byte)
// {
//     uint8_t receivedbyte = 0;

//     /* Send a Byte through the SPI peripheral */
//     /* Read byte from the SPI bus */
//     if (HAL_SPI_TransmitReceive(hspi, (uint8_t *)&Byte, (uint8_t *)&receivedbyte, 1, SPIx_TIMEOUT_MAX) != HAL_OK)
//     {
//         SPI_Error(hspi);
//     }

//     return receivedbyte;
// }

/**
 * @brief  SPI error treatment function.
 */
// static void SPI_Error(SPI_HandleTypeDef* hspi)
// {
//     // HAL_SPI_DeInit internally calls HAL_SPI_MspDeInit in stm32f4xx_hal_msp.c
//     HAL_SPI_DeInit(hspi);

//     // Re-Initialize the SPI communication bus
//     SPI_Init(hspi);
// }

/**
 * @brief USART Initialization Function
 * @param None
 * @retval None
 */
static void USART_UART_Init(void)
{
    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follow:
        - Word Length = 8 Bits
        - Stop Bit = One Stop bit
        - Parity = None
        - BaudRate = 9600 baud (user configurable)
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance = USARTx;
    UartHandle.Init.BaudRate = UART2_BAUD_RATE;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    // Set transmission flag: transfer complete
    UartReady = SET;

    BSP_LED_Toggle(LED6);
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    /* Turn LED3 on: Transfer error in reception/transmission process */
    BSP_LED_On(LED3);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
 * @}
 */

/**
 * @}
 */
