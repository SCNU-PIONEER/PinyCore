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

#ifdef CHASSIS_TYPE
#if !APP_USE_COMM || APP_CHASSIS_SLAVE
#include ARMORLOADER_FILE
Lazy<ARMORLOADER::ARMORLOADER_TYPE> armorLoader;
#include CHASSIS_FILE
Lazy<CHASSIS::CHASSIS_TYPE> chassis;
#endif
#endif
#ifdef GIMBAL_TYPE
#if !APP_USE_COMM || APP_GIMBAL_MASTER
#include ARMORBOOSTER_FILE
Lazy<ARMORBOOSTER::ARMORBOOSTER_TYPE> armorBooster;
#include GIMBAL_FILE
Lazy<GIMBAL::GIMBAL_TYPE> gimbal;
#endif
#endif

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

#if !APP_USE_COMM || APP_CHASSIS_SLAVE
    armorLoader.init();
    schedule([]() { armorLoader->update(cmd->getMsgBus()); });
    chassis.init();
    schedule([]() { chassis->update(cmd->getMsgBus()); });
#endif

#if !APP_USE_COMM || APP_GIMBAL_MASTER
    armorBooster.init();
    schedule([]() { armorBooster->update(cmd->getMsgBus()); });
    gimbal.init();
    schedule([]() { gimbal->update(cmd->getMsgBus()); });
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

void AppManager::schedule(std::function<void()> _callback) { tasks.push_back(std::move(_callback)); }

void AppManager::createApp()
{
    // Test-Module Continuous Task
    if constexpr (APP_USE_TEST) {
        TestModule::instance();
    }

    // Buzzer Once Task
    xTaskCreate(
            [](void *_param) -> void {
                BUZZER::Buzzer::getInstance().playPinyCore();
                vTaskDelete(nullptr); // 否则会进ExistError
            },
            "buzzer_task", 128, nullptr, 3, nullptr);

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
