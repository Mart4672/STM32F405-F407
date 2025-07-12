#include "GpioPin.hpp"

GpioPin::GpioPin(uint16_t pin, GPIO_TypeDef* port) :
    mGpioPin(pin),
    mGpioPort(port)
{
}

bool GpioPin::Read()
{
    return HAL_GPIO_ReadPin(mGpioPort, mGpioPin);
}

void GpioPin::Set()
{
    HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_SET);
}

void GpioPin::Reset()
{
    HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_RESET);
}

void GpioPin::Toggle()
{
    HAL_GPIO_TogglePin(mGpioPort, mGpioPin);
}

void GpioPin::SetMode(unsigned long pinMode = GPIO_MODE_OUTPUT_PP,
                      unsigned long pullMode = GPIO_NOPULL,
                      unsigned long speed = GPIO_SPEED_FREQ_LOW)
{
    // create empty GPIO_InitStruct
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Assume that the GPIO Port clock has already been enabled
    // __HAL_RCC_GPIOD_CLK_ENABLE();

    // reset the pin before configuring it
    HAL_GPIO_WritePin(mGpioPort, mGpioPin, GPIO_PIN_RESET);

    // Configure the GPIO_InitStruct with the provided parameters
    GPIO_InitStruct.Pin = mGpioPin;
    GPIO_InitStruct.Mode = pinMode;
    GPIO_InitStruct.Pull = pullMode;
    GPIO_InitStruct.Speed = speed;
    HAL_GPIO_Init(mGpioPort, &GPIO_InitStruct);
}
