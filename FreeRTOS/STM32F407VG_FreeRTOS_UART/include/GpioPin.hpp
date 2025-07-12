#pragma once

#include "stm32f4xx_hal.h"

class GpioPin 
{
public: 
    GpioPin(uint16_t pin, GPIO_TypeDef* port);   // Constructor

    bool Read();   
    void Set();
    void Reset();
    void Toggle();
    void SetMode(unsigned long pinMode, unsigned long pullMode, unsigned long speed);
    uint16_t getPin() const { return mGpioPin; }
    GPIO_TypeDef* getPort() const { return mGpioPort; }

private:
    uint16_t mGpioPin;
    GPIO_TypeDef* mGpioPort;
}; 
