#include "EventLoop.hpp"
#include "Adafruit_NeoPixel.hpp"
#include "cmsis_os.h"   // Include CMSIS RTOS header for osKernelStart and other RTOS functions
// Include GPIO definitions
// Also includes stm32f4xx_hal.h
#include "main.h"
// #include "CppBlinkPinout.hpp" // TODO delete later if not needed
#include "GpioPin.hpp"
#include "CppTimerManager.hpp"

// Blink 1 Task Period in microseconds
constexpr uint32_t blink1TaskPeriod = 5'000'000;

// Blink 2 Task Period in microseconds
constexpr uint32_t blink2TaskPeriod = 10'000'000;

// Blink 3 Task Period in microseconds
constexpr uint32_t blink3TaskPeriod = 50'000'000;

// Blink 4 Task Period in microseconds
constexpr uint32_t blink4TaskPeriod = 100'000'000;

// NeoPixel Blink Task Period in microseconds
constexpr uint32_t neoBlinkTaskPeriod = 20'000'000;

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

// Global timer manager instance
CppTimerManager timerManager = CppTimerManager();

// TODO update the NeoPixel library to use the timer class

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
        timerManager.setHWTimerCountFunction(readHwTimer);
        timerManager.setHWTimerOverflowFunction(readTimerOverFlow);
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
    // setup
    GpioPin led1(LED1_Pin, GPIOD);
    uint32_t blink1Time = timerManager.getHWTimerCount();
    uint32_t blink1Overflow = timerManager.getHWTimerOverflow();

    // inifinite loop
    while(true)
    {
        // Detect rollover and only delay the task if the timer rolled over more than once
        // This is the preferred option for tasks that need to run at a specific rate
        uint32_t now = timerManager.getHWTimerCount();
        if ((timerManager.getHWTimerOverflow() - blink1Overflow) == 1)
        {
            blink1Overflow = timerManager.getHWTimerOverflow();
            // If there has only been 1 new timer rollover, no action is needed
            // since the timer values in the "Task Wait Period Elapsed Check"
            // will still be valid for the next check.
            // Ex:
            // uint32_t (0 - 4,294,967,295) = uint32_t (1) = 1
            // Ex:
            // uint32_t (100 - 4,294,967,195) = uint32_t (100 + 101) = 201
        }
        else if (timerManager.getHWTimerOverflow() > blink1Overflow)
        {
            blink1Overflow = timerManager.getHWTimerOverflow();
            // Reset the task timer if there have been multiple rollovers
            // since the subtraction in the "Task Wait Period Elapsed Check"
            // will not be valid anymore.
            blink1Time = now;
        }

        // Task Wait Period Elapsed Check
        if((timerManager.getHWTimerCount() - blink1Time) >= blink1TaskPeriod)
        {
            // Reset the timer
            blink1Time = timerManager.getHWTimerCount();
            // blink the LED
            led1.Set();
            osDelay(10);
            led1.Reset();

            // seeing that calling 1'680'000'000 NOPs takes 40s instead of 10s
            // The number of NOPs called in the NeoPixel library has been decreased by a factor of 4
            // but the NEO Pixel LEDs are still not lighting up properly

            // __disable_irq();
            // // The number of NOPs needed for proper timing is:
            // //  1,000,000,000ns / 5.95238 ns = 168,000,000 cycles
            // uint32_t totalNOPs = 1'680'000'000; // Number of NOPs needed for 10s
            // uint32_t ledOnNOPs = 168'000'000 / 10; // Number of NOPs needed for .1s
            // uint32_t ledOffNOPs = totalNOPs - ledOnNOPs; // Number of NOPs needed for 9.9s

            // while (true)
            // {
            //     for (uint32_t j = 0; j < ledOffNOPs; j++)
            //     {
            //         __NOP();
            //     }
            //     // blink the led
            //     led1.Set();
            //     for (uint32_t j = 0; j < ledOnNOPs; j++)
            //     {
            //         __NOP();
            //     }
            //     led1.Reset();
            // }
        }
        // Yield to allow other tasks to run
        osDelay(1);
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
    uint32_t blink2Time = timerManager.getHWTimerCount();
    // TODO add logic for dealing with overflow
    // uint32_t blink2Overflow = timerManager.getHWTimerOverflow();

    // inifinite loop
    while(true)
    {
        if((timerManager.getHWTimerCount() - blink2Time) >= blink2TaskPeriod)
        {
            // Reset the timer
            blink2Time = timerManager.getHWTimerCount();
            // blink the LED
            led2.Set();
            osDelay(10);
            led2.Reset();
        }
        // Yield to allow other tasks to run
        osDelay(1);
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
    uint32_t blink3Time = timerManager.getHWTimerCount();
    // TODO add logic for dealing with overflow
    // uint32_t blink3Overflow = timerManager.getHWTimerOverflow();

    // inifinite loop
    while(true)
    {
        if((timerManager.getHWTimerCount() - blink3Time) >= blink3TaskPeriod)
        {
            // Reset the timer
            blink3Time = timerManager.getHWTimerCount();
            // blink the LED
            led3.Set();
            osDelay(10);
            led3.Reset();
        }
        // Yield to allow other tasks to run
        osDelay(1);
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
    uint32_t blink4Time = timerManager.getHWTimerCount();
    // TODO add logic for dealing with overflow
    // uint32_t blink4Overflow = timerManager.getHWTimerOverflow();

    // inifinite loop
    while(true)
    {
        if((timerManager.getHWTimerCount() - blink4Time) >= blink4TaskPeriod)
        {
            // Reset the timer
            blink4Time = timerManager.getHWTimerCount();
            // blink the LED
            led4.Set();
            osDelay(10);
            led4.Reset();
        }
        // Yield to allow other tasks to run
        osDelay(1);
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
    GpioPin neoPixelPin(NEO_Pin, NEO_GPIO_Port);
    uint16_t n = 2; // Number of NeoPixels
    // TODO confirm that this is the correct type
    // neoPixelType type = NEO_GRB + NEO_KHZ800;

    // auto neoPixel = Adafruit_NeoPixel(n, pin, type);
    auto neoPixels = Adafruit_NeoPixel(n, neoPixelPin);
    // neoPixels.micros = std::bind(&CppTimerManager::getHWTimerCount);

    // set the function for reading the hardware timer count
    neoPixels.micros = timerManager.getHWTimerCount;

    // Initialize the NeoPixel library
    neoPixels.begin();

    // set initial brightness
    neoPixels.setBrightness(15); // Set brightness to a moderate level (0-255)

    // Set all pixels to off initially
    neoPixels.clear();

    uint32_t neoBlinkTime = timerManager.getHWTimerCount();
    // TODO add logic for dealing with overflow
    // uint32_t neoBlinkOverflow = timerManager.getHWTimerOverflow();

    // inifinite loop
    while(true)
    {
        if((timerManager.getHWTimerCount() - neoBlinkTime) >= neoBlinkTaskPeriod)
        {
            // Reset the timer
            neoBlinkTime = timerManager.getHWTimerCount();
            // blink the neoPixel
            for(uint16_t i = 0; i < n; i++)
            {
                // neoPixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
                // Here we're using a moderately bright green color:
                // neoPixels.setPixelColor(i, neoPixels.Color(15, 0, 0));
                // Alternative way to set color
                neoPixels.setPixelColor(i, 15, 0, 0);

                // Send the updated pixel colors to the hardware
                neoPixels.show();
                // Delay for a short period to see each pixel update
                osDelay(1000);
            }
            // Set all pixels to off
            neoPixels.clear();
        }
        // Yield to allow other tasks to run
        osDelay(1);
    }

    // In case we accidentally exit from the task loop
    osThreadTerminate(NULL);
}
