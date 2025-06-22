#include "EventLoop.hpp"
#include "Adafruit_NeoPixel.hpp"
#include "cmsis_os.h"   // Include CMSIS RTOS header for osKernelStart and other RTOS functions
// #include "stm32f4xx_hal.h"
#include "main.h"   // Include main header for HAL and GPIO definitions
// #include "CppBlinkPinout.hpp" // might delete later
#include "GpioPin.hpp"

/* Definitions for neoBlink */
osThreadId_t neoBlinkHandle;
const osThreadAttr_t neoBlink_attributes = {
  .name = "neoBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
void StartNeoBlink(void *argument);

/* Definitions for blink1 */
osThreadId_t blink1Handle;
const osThreadAttr_t blink1_attributes = {
  .name = "blink1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
void StartBlink1(void *argument);

/* Definitions for blink2 */
osThreadId_t blink2Handle;
const osThreadAttr_t blink2_attributes = {
  .name = "blink2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
void StartBlink2(void *argument);

/* Definitions for blink3 */
osThreadId_t blink3Handle;
const osThreadAttr_t blink3_attributes = {
  .name = "blink3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
void StartBlink3(void *argument);

/* Definitions for blink4 */
osThreadId_t blink4Handle;
const osThreadAttr_t blink4_attributes = {
  .name = "blink4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
void StartBlink4(void *argument);

// Main Cpp event loop to run application
void EventLoopCpp()
{
    /* Create the thread(s) */
    /* creation of blink1 */
    blink1Handle = osThreadNew(StartBlink1, NULL, &blink1_attributes);

    /* creation of blink2 */
    blink2Handle = osThreadNew(StartBlink2, NULL, &blink2_attributes);

    /* creation of blink3 */
    blink3Handle = osThreadNew(StartBlink3, NULL, &blink3_attributes);

    /* creation of blink4 */
    blink4Handle = osThreadNew(StartBlink4, NULL, &blink4_attributes);

    /* creation of neoBlink */
    neoBlinkHandle = osThreadNew(StartNeoBlink, NULL, &neoBlink_attributes);

    osKernelStart();

    // Your init code here
    while(1)
    {
        // Your loop code here
    }
}

// Define all C function calls from main.c below
extern "C"
{
    void EventLoopC()
    {
        EventLoopCpp();
    }

    void InitializeInterface(ReadHwTimer readHwTimer, ReadOverflow readTimerOverFlow)
    {
        /* 
        * Can now link these function pointers to any C++ class for use. 
        * For example, Could pass them into a static global system time 
        * class where they would be used to calculate the ongoing system 
        * time from system startup. 
        */
    }
}

/* USER CODE BEGIN Header_StartBlink1 */
/**
  * @brief  Function implementing the blink1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink1 */
void StartBlink1(void *argument)
{
  GpioPin led1(LED1_Pin, GPIOD);

  for(;;)
  {
    led1.Set();
    osDelay(10);
    led1.Reset();
    osDelay(4990);
  }

  // In case we accidentally exit from the task loop
  osThreadTerminate(NULL);
}

/* USER CODE BEGIN Header_StartBlink2 */
/**
* @brief Function implementing the blink2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink2 */
void StartBlink2(void *argument)
{
  GpioPin led2(LED2_Pin, GPIOD);

  for(;;)
  {
    led2.Set();
    osDelay(10);
    led2.Reset();
    osDelay(9990);
  }

  // In case we accidentally exit from the task loop
  osThreadTerminate(NULL);
}

/* USER CODE BEGIN Header_StartBlink3 */
/**
* @brief Function implementing the blink3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink3 */
void StartBlink3(void *argument)
{
  GpioPin led3(LED3_Pin, GPIOD);

  for(;;)
  {
    led3.Set();
    osDelay(10);
    led3.Reset();
    osDelay(49990);
  }

  // In case we accidentally exit from the task loop
  osThreadTerminate(NULL);
}

/* USER CODE BEGIN Header_StartBlink4 */
/**
* @brief Function implementing the blink4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink4 */
void StartBlink4(void *argument)
{
  GpioPin led4(LED4_Pin, GPIOD);

  for(;;)
  {
    led4.Set();
    osDelay(10);
    led4.Reset();
    osDelay(99990);
  }

  // In case we accidentally exit from the task loop
  osThreadTerminate(NULL);
}

/* USER CODE BEGIN Header_StartNeoBlink */
/**
* @brief Function implementing the neoBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNeoBlink */
void StartNeoBlink(void *argument)
{
  GpioPin led4(LED4_Pin, GPIOD);

  for(;;)
  {
    led4.Set();
    osDelay(10);
    led4.Reset();
    osDelay(9990);
  }

  // In case we accidentally exit from the task loop
  osThreadTerminate(NULL);
}

