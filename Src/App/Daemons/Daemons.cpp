#include "Daemons.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "StmLog.hpp"
#include "Rc.hpp"


extern "C" void vApplicationMallocFailedHook()
{
    LOG::error("Daemons", "no heap to malloc");
    __BKPT(0x01);
}

extern "C" void vApplicationStackOverflowHook()
{
    LOG::error("Daemons", "stack overflow");
    __BKPT(0x01);
}

Daemons::Daemons()
{
    /* rc */
    schedule([]() {
        static uint32_t updateCnt = 0;
        if (!RC::Rc::instance().isOnline()) {
            if (xTaskGetTickCount() - updateCnt >= 2000) {
                updateCnt = xTaskGetTickCount();
                LOG::warn("Daemons", "RC Online");
            }
        }
    });

    /* FreeRTOS heap size monitor */
    schedule([]() {
        static constexpr size_t MINHEAP = 1024;
        static uint32_t updateCnt = 0;
        if (xTaskGetTickCount() - updateCnt >= 2000) {
            updateCnt = xTaskGetTickCount();
            uint32_t heap = xPortGetFreeHeapSize();
            // LOG::info("Daemons", "Free Heap: %u", heap);
            if (heap < MINHEAP) {
                LOG::warn("Daemons", "Heap: %u", heap);
            }
        }
    });
}

Daemons &Daemons::instance()
{
    static Daemons instance;
    return instance;
}

void Daemons::schedule(std::function<void()> _func)
{
    cb.push_back(std::move(_func));
}

void Daemons::update()
{
    static uint32_t updateCnt = 0;
    if (xTaskGetTickCount() - updateCnt >= SEND_INTERVAL) {
        updateCnt = xTaskGetTickCount();
        for (auto &func : cb) {
            func();
        }
    }
}
