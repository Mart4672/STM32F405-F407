#include "EventLoop.hpp"
// #include "main.h"
#include "Adafruit_NeoPixel.hpp"
#include "cmsis_os.h"   // Include CMSIS RTOS header for osKernelStart and other RTOS functions

// Main Cpp event loop to run application
void EventLoopCpp()
{
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



