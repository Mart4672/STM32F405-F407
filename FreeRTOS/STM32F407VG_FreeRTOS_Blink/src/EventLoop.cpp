#include "EventLoop.hpp"
// #include "main.h"
#include "Adafruit_NeoPixel.hpp"

// Main Cpp event loop to run application
void EventLoopCpp()
{
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
}



