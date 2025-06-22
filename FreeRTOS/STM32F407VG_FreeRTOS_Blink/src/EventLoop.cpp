#include "EventLoop.hpp"
#include "Adafruit_NeoPixel.hpp"
#include "cmsis_os.h"   // Include CMSIS RTOS header for osKernelStart and other RTOS functions
// #include "stm32f4xx_hal.h"
#include "main.h"   // Include main header for HAL and GPIO definitions

/* Definitions for neoBlink */
osThreadId_t neoBlinkHandle;
const osThreadAttr_t neoBlink_attributes = {
  .name = "neoBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
void StartNeoBlink(void *argument);

// Main Cpp event loop to run application
void EventLoopCpp()
{
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

/* USER CODE BEGIN Header_StartNeoBlink */
/**
* @brief Function implementing the neoBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNeoBlink */
void StartNeoBlink(void *argument)
{
  /* USER CODE BEGIN StartNeoBlink */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_SET);
    osDelay(10);
    HAL_GPIO_WritePin(GPIOD, LED4_Pin, GPIO_PIN_RESET);
    osDelay(9990);
  }

  // In case we accidentally exit from the task loop
  osThreadTerminate(NULL);

  /* USER CODE END StartBlink4 */
}

