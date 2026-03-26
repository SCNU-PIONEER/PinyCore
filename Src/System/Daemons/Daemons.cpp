#include "Daemons.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "StmLog.hpp"
#include "main.h"

extern "C" void vApplicationMallocFailedHook()
{
    LOG::error("Daemons", "no heap to malloc");
    __BKPT(0x01);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t _xTask, char *_pcTaskName)
{
    (void)_xTask;
    LOG::error("Daemons", "task: %s stack overflow", _pcTaskName);
    __BKPT(0x01);
}

void Daemons::init()
{
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

void Daemons::schedule(std::function<void()> _func) { cb.push_back(std::move(_func)); }

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
