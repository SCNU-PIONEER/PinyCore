#include "AppManager.hpp"
#include "sdkconfig.h"
#if APP_USE_COMM
#include "CommManager.hpp"
#endif

#include "Cmd.hpp"
#include "Test/TestModule.hpp"
#include "Bsp.hpp"
#include "Lazy.hpp"
#include "INS.hpp"
#if APP_USE_CHASSIS
#include CHASSIS_FILE
#endif
#if APP_USE_GIMBAL
#include GIMBAL_FILE
#include "Arm.hpp"
#endif

#if EXTENSION_BUZZER
#include "Buzzer.hpp"
#endif

#if APP_USE_DAEMONS
#include "Daemons.hpp"
#endif

//---------------------------------------------------------------------------------------------------

Lazy<INS_SYS::INS> ins;

//---------------------------------------------------------------------------------------------------

Lazy<Cmd> cmd;

#if APP_USE_CHASSIS
Lazy<CHASSIS::Mecanum> chassis;
#endif
//---------------------------------------------------------------------------------------------------

#if APP_USE_GIMBAL
Lazy<GIMBAL::Standard> gimbal;
Lazy<Arm> arm;
#endif
//---------------------------------------------------------------------------------------------------

#if APP_USE_UI
#include "UI/UIApp.hpp"
#endif

//---------------------------------------------------------------------------------------------------

void AppManager::initApp()
{
    Power_5V_ON();

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

#if APP_USE_CHASSIS
    chassis.init();
    schedule([]() { chassis->update(&(cmd->getMsgBus()->chassisQueue)); });
#endif

#if APP_USE_GIMBAL
    gimbal.init();
    schedule([]() { gimbal->update((cmd->getMsgBus())); });

    arm.init();
    schedule([]() { arm->update(cmd->getMsgBus()); });
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
