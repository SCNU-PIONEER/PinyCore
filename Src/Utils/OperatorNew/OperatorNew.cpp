#include <cstdlib>
#include <new>

#include "FreeRTOS.h"
#include "task.h"


void *operator new(size_t _size) { return pvPortMalloc(_size); }

void *operator new[](size_t _size) { return pvPortMalloc(_size); }

void *operator new(size_t _size, std::align_val_t _alignment)
{
    return aligned_alloc(static_cast<size_t>(_alignment), _size);
}

void *operator new[](size_t _size, std::align_val_t _alignment)
{
    return aligned_alloc(static_cast<size_t>(_alignment), _size);
}

void *operator new(size_t _size, const std::nothrow_t &) noexcept
{
    return ::operator new(_size);
}

void *operator new[](size_t _size, const std::nothrow_t &) noexcept
{
    return ::operator new[](_size);
}

void *operator new(size_t _size, std::align_val_t _alignment,
                   const std::nothrow_t &) noexcept
{
    return ::operator new(_size, _alignment);
}

void *operator new[](size_t _size, std::align_val_t _alignment,
                     const std::nothrow_t &) noexcept
{
    return ::operator new[](_size, _alignment);
}


void operator delete(void *_ptr) noexcept { vPortFree(_ptr); }
void operator delete[](void *_ptr) noexcept { vPortFree(_ptr); }

void operator delete(void *_ptr, std::align_val_t) noexcept { vPortFree(_ptr); }
void operator delete[](void *_ptr, std::align_val_t) noexcept
{
    vPortFree(_ptr);
}

void operator delete(void *_ptr, std::size_t) noexcept { vPortFree(_ptr); }
void operator delete[](void *_ptr, std::size_t) noexcept { vPortFree(_ptr); }

void operator delete(void *_ptr, std::size_t, std::align_val_t) noexcept
{
    vPortFree(_ptr);
}
void operator delete[](void *_ptr, std::size_t, std::align_val_t) noexcept
{
    vPortFree(_ptr);
}
