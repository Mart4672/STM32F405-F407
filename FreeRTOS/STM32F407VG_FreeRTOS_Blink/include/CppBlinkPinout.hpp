#pragma once

#include "GpioPin.hpp"
#include "main.h"

class CppBlinkPinout 
{
public: 
    CppBlinkPinout();

    // Define all desired GPIO pins
    // can optionally initialize them in the constructor initializer list
    GpioPin mLED1 {LED1_Pin, GPIOD};
    GpioPin mLED2 {LED2_Pin, GPIOD};
    GpioPin mLED3 {LED3_Pin, GPIOD};
    GpioPin mLED4 {LED4_Pin, GPIOD};
};
