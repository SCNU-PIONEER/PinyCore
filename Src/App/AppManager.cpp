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
#include "Daemons/Daemons.hpp"
Lazy<Daemons> daemon;
#endif


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
Lazy<UI::App> ui;
#endif

//---------------------------------------------------------------------------------------------------

void AppManager::initApp()
{
    // Buzzer
    BUZZER::Buzzer::getInstance().init(&BEEP_TIMER, BEEP_TIM_CHANNEL, BEEP_APB_FREQ);

#if APP_USE_COMM
    schedule([]() { CommManager::instance().rxTask(); });
#endif

#if APP_USE_INS
    ins.init(&IMU_SPI);
#endif

    cmd.init();

#if APP_USE_UI
    ui.init(UI_ROBOT_ID);
    schedule([]() { ui->task(); });
#endif
#if APP_USE_DAEMONS
    daemon.init();
    schedule([]() { daemon->update(); });
#endif

    // TestModule
    if constexpr (APP_USE_TEST) {
        TestModule::instance()->init();
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
