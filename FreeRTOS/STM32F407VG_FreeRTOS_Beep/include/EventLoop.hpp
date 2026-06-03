#pragma once

// #include "main.h"
#include <stdint.h>
#include <stdio.h>

void EventLoopCpp();   // Cpp function to call into main event loop

// Define func pointers for reading hw timers and timer overflows
typedef uint32_t (*ReadHwTimer)();
typedef uint32_t (*ReadOverflow)();

// C Zone
// ############################################################################
#ifdef __cplusplus
extern "C"
{
#endif

void EventLoopC();  // C function to call into Cpp event loop from main
void InitializeInterface(ReadHwTimer readHwTimer, ReadOverflow readTimerOverFlow);

#ifdef __cplusplus
}
#endif
// ############################################################################
