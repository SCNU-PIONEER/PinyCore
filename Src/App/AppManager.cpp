#include "AppManager.hpp"
#include "sdkconfig.h"
#if APP_USE_COMM
#include "CommManager.hpp"
#endif

#include "Cmd.hpp"
#include "Test/TestModule.hpp"
#include "Bsp.hpp"
#include "Lazy.hpp"

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

#if APP_USE_DAEMONS
#include "Daemons.hpp"
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
#endif

//---------------------------------------------------------------------------------------------------

#ifdef CHASSIS_TYPE
#if !APP_USE_COMM || TARGET_SLAVE
#include CHASSIS_FILE
Lazy<CHASSIS::CHASSIS_TYPE> chassis;
#include ARMORLOADER_FILE
Lazy<ARMORLOADER::ARMORLOADER_TYPE> armorLoader;
#endif
#endif
#ifdef GIMBAL_TYPE
#if !APP_USE_COMM || TARGET_MASTER
#include GIMBAL_FILE
Lazy<GIMBAL::GIMBAL_TYPE> gimbal;
#include ARMORBOOSTER_FILE
Lazy<ARMORBOOSTER::ARMORBOOSTER_TYPE> armorBooster;
#endif
#endif

//---------------------------------------------------------------------------------------------------

void AppManager::initApp()
{
#if EXTENSION_BUZZER
    BUZZER::buzz.init(&BEEP_TIMER, BEEP_TIM_CHANNEL, BEEP_APB_FREQ);
    BUZZER::buzz->playNote(BUZZER::PinyCore);
#endif

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

#if !APP_USE_COMM || TARGET_SLAVE
    chassis.init();
    schedule([]() { chassis->update(cmd->getMsgBus()); });
    armorLoader.init();
    schedule([]() { armorLoader->update(cmd->getMsgBus()); });
#endif

#if !APP_USE_COMM || TARGET_MASTER
    gimbal.init();
    schedule([]() { gimbal->update(cmd->getMsgBus()); });
    armorBooster.init();
    schedule([]() { armorBooster->update(cmd->getMsgBus()); });
#endif

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
