#pragma once
// #ifndef EVENTLOOP_HPP_
// #define EVENTLOOP_HPP_

// #include "main.h"
#include <stdint.h>
#include <stdio.h>

void EventLoopCpp();   // Cpp function to call into main event loop

#ifdef __cplusplus
extern "C"
{
#endif

void EventLoopC();  // C function to call into Cpp event loop from main

#ifdef __cplusplus
}
#endif

// #endif /* EVENTLOOP_HPP_ */
