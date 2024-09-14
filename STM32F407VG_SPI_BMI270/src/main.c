/**
 ******************************************************************************
 * @file    STM32F407VG_SPI_BMP388/src/main.c
 * @author  Redstone Space Systems
 * @brief   This code shows how to use the STM32F407 (or STM32F405) to read
 * data from the BMP388 barometric pressure sensor using the SPI HAL API
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

SPI_HandleTypeDef SpiHandle;

TIM_HandleTypeDef htim5;

BMP388_InitializationConfig bmp388initConfig;

BMP388_InterruptConfig bmp388interruptConfig;

uint8_t bmp388readData = 0;

uint32_t timer_val;

static const uint8_t waitForUartTransmit = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void USART_UART_Init(void);
static void MX_TIM5_Init(void);

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

    /*#### Configure the UART peripheral #####################################*/
    USART_UART_Init();

    /*#### Configure the BMP388 Device ######################################*/

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
    // uint8_t interruptStatus = 0;
    float pressure = 0.0f;
    float temp = 0.0f;
    char buf3[100];
    int len3;
    char buf4[150];
    int len4;
    uint16_t numReadDataCalls = 0;

    HAL_Delay(1000);

    len3 = sprintf(buf3, "timer_val is %lu\r\n\n\n", timer_val);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)buf3, len3, 100);

    while (1)
    {
        // read data task
        if ((bmp388readData == 1) && (timer_val - interruptStatusTime) >= 40000)
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
        if ((timer_val - printTime) >= 1000000)
        {
            printTime = __HAL_TIM_GET_COUNTER(&htim5);

            len4 = snprintf(buf4, 103, "[%10lu] Data reads: %3u, pressure: %6d Pa, temp: (%5d/100) C, print task time: %6lu us\r\n\n\n", timer_val, numReadDataCalls, (int)pressure, (int)(temp * 100.f), totalPrintTime);
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

            // time for UART Task to run
            totalPrintTime = __HAL_TIM_GET_COUNTER(&htim5) - printTime;
        }

        // update timer_val
        timer_val = __HAL_TIM_GET_COUNTER(&htim5);
    }

    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
    // Wait for USER Button press before starting the Communication
    while (BSP_PB_GetState(BUTTON_KEY) == RESET)
    {
        // Toggle LED3 waiting for user to press button
        BSP_LED_Toggle(LED3);
        HAL_Delay(400);
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
    // char buf2[100];
    // int len2;

    // if (GPIO_Pin == KEY_BUTTON_PIN)
    // {
    //     // /* Toggle LED3 */
    //     // BSP_LED_Toggle(LED3);
    //     // /* Toggle LED4 */
    //     // BSP_LED_Toggle(LED4);
    //     // /* Toggle LED5 */
    //     // BSP_LED_Toggle(LED5);
    //     // /* Toggle LED6 */
    //     BSP_LED_Toggle(LED6);

    //     len2 = sprintf(buf2, "push button callback\r\n\n\n");
    //     HAL_UART_Transmit(&UartHandle, (uint8_t *)buf2, len2, 100);
    // }
    if (GPIO_Pin == BMP388_INT1_PIN) // BMP388_INT1_PIN
    {
        // HAL_Delay(10);

        /* Toggle LED3 */
        // BSP_LED_Toggle(LED3);
        /* Toggle LED4 */
        BSP_LED_Toggle(LED4);
        /* Toggle LED5 */
        // BSP_LED_Toggle(LED5);
        /* Toggle LED6 */
        // BSP_LED_Toggle(LED6);

        // HAL_Delay(1000);

        // read INT_STATUS register to de-assert the INT pin
        // BMP388_ReadIntStatus();

        // set BMP388 read data registers flag
        bmp388readData = 1;

        // BMP388_ReadRawData(&pressureRawGlobal, &tempRawGlobal);

        // len2 = sprintf(buf2, "BMP388 int pin callback\r\n\n\n");
        // HAL_UART_Transmit(&UartHandle, (uint8_t *)buf2, len2, 100);
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
        - BaudRate = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance = USARTx;
    UartHandle.Init.BaudRate = 9600; // 115200, 57600
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
    /* Turn LED5 on */
    BSP_LED_On(LED5);
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
