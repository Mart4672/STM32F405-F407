
#include "CppTimerManager.hpp"

CppTimerManager::CppTimerManager()
{
}

CppTimerManager::CppTimerManager(std::function<uint32_t()> getHWTimerCountFunc, 
                                 std::function<uint32_t()> getHWTimerOverflowFunc) :
        getHWTimerCount(getHWTimerCountFunc),
        getHWTimerOverflow(getHWTimerOverflowFunc)
{
}
