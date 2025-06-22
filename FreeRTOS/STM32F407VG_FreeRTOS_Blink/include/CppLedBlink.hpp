#pragma once

#include "CppBlinkPinout.hpp"

class CppLedBlink
{
public:
    CppLedBlink();

    private:
    CppBlinkPinout mGPIOs;
};
