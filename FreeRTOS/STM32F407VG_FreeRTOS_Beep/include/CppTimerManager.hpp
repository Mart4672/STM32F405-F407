#pragma once

#include <functional>
#include <stdint.h>

class CppTimerManager 
{
public:
    // Default constructor
    CppTimerManager();
    // Constructor that takes function pointers to read hardware timer and overflow
    CppTimerManager(std::function<uint32_t()> getHWTimerCountFunc, 
                    std::function<uint32_t()> getHWTimerOverflowFunc);

    // Function pointer to read hardware timer
    std::function<uint32_t()> getHWTimerCount;
    // Function pointer to read timer overflow count
    std::function<uint32_t()> getHWTimerOverflow;

    void setHWTimerCountFunction(std::function<uint32_t()> func) {
        getHWTimerCount = func;
    };

    void setHWTimerOverflowFunction(std::function<uint32_t()> func) {
        getHWTimerOverflow = func;
    };
};
