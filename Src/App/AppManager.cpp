#include "AppManager.hpp"
#include "sdkconfig.h"
#if APP_USE_COMM
#include "CommManager.hpp"
#endif

#include "Cmd.hpp"
#include "Buzzer.hpp"
#include "Test/TestModule.hpp"
#include "Bsp.hpp"
#include "Lazy.hpp"

#if APP_USE_DAEMONS
#include "Daemons.hpp"
#endif

#include "etl/queue_spsc_atomic.h"
#include "etl/memory_model.h"
#include "Bsp.hpp"

//---------------------------------------------------------------------------------------------------

#if APP_USE_INS
#include "INS.hpp"
Lazy<INS_SYS::INS> ins;
#endif

//---------------------------------------------------------------------------------------------------

Lazy<Cmd> cmd;

//---------------------------------------------------------------------------------------------------


#if APP_USE_UI
#include "UI/UIApp.hpp"
#endif

etl::queue_spsc_atomic<uint32_t, 12, etl::memory_model::MEMORY_MODEL_SMALL> queue;
QueueHandle_t queue1;
volatile uint32_t b = 0;

void task1([[maybe_unused]] void *_arg)
{
    uint8_t a = 1;
    a = b;
    while (true) {
        // queue.push(a);             // 9.8us
        xQueueSend(queue1, &a, 0); // 13.1us
        // LOG::info("App", "push: 1");
        vTaskDelay(2);
    }
}
void task2([[maybe_unused]] void *_arg)
{
    uint32_t a = 0;
    while (true) {
        xQueueReceive(queue1, &a, 0); // 13.0us
        queue.pop(a);                 // 10.4us
        // if (queue.pop(a)) {
        // LOG::info("App", "pop: %u", a);
        // }
        vTaskDelay(1);
    }
}

//---------------------------------------------------------------------------------------------------

void AppManager::initApp()
{
    // Buzzer
    BUZZER::Buzzer::instance().init(&BEEP_TIMER, BEEP_TIM_CHANNEL, BEEP_APB_FREQ);

    queue1 = xQueueCreate(12, sizeof(uint32_t));

    xTaskCreate(task1, "Task1", 256, nullptr, 5, nullptr);
    xTaskCreate(task2, "Task2", 256, nullptr, 5, nullptr);

#if APP_USE_COMM
    schedule([]() { CommManager::instance().rxTask(); });
#endif

#if APP_USE_INS
    ins.init(&IMU_SPI);
#endif

    cmd.init();

#if APP_USE_UI
    UI::APP::instance().init();
    schedule([]() { UI::APP::instance().task(); });
#endif
#if APP_USE_DAEMONS
    Daemons::instance().init();
    schedule([]() { Daemons::instance().update(); });
#endif

    // TestModule
    if constexpr (APP_USE_TEST) {
        TestModule::instance().init();
    }

#if APP_USE_COMM
    schedule([]() { CommManager::instance().txTask(); });
#endif

    // Generate threads at the end
    this->createApp();
}


void AppManager::createApp()
{
    // Test-Module Continuous Task
    if constexpr (APP_USE_TEST) {
        TestModule::instance();
    }

    uint32_t freeHeap = xPortGetFreeHeapSize();
    LOG::info("App", "init complete, Free Heap: %u", freeHeap);
}

void AppManager::task()
{
    while (true) {
        for (auto &task : this->tasks) {
            task();
        }
        vTaskDelay(1);
    }
}
