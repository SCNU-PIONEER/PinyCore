#pragma once

#include "FreeRTOS.h"
#include "task.h"

__attribute__((weak)) void *operator new(size_t _size)
{
    void *p = pvPortMalloc(_size);
    if (p == nullptr) {
        configASSERT(!"Failed to allocate memory in new operator");
    }
    return p;
}

__attribute__((weak)) void operator delete(void *_p) noexcept { vPortFree(_p); }

__attribute__((weak)) void *operator new[](size_t _size)
{
    return operator new(_size);
}

__attribute__((weak)) void operator delete[](void *_p) noexcept
{
    operator delete(_p);
}

__attribute__((weak)) void operator delete(void *_p, size_t) noexcept
{
    operator delete(_p);
}

__attribute__((weak)) void operator delete[](void *_p, size_t) noexcept
{
    operator delete(_p);
}
