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
#include "INS.hpp"
#include "Swerve.hpp"
#include "Standard.hpp"
#include "UI/UIApp.hpp"

#if APP_USE_DAEMONS
#include "Daemons/Daemons.hpp"
Lazy<Daemons> daemon;
#endif


//---------------------------------------------------------------------------------------------------

Lazy<INS_SYS::INS> ins;

//---------------------------------------------------------------------------------------------------

Lazy<Cmd> cmd;

Lazy<CHASSIS::Swerve> chassis;
Lazy<GIMBAL::Standard> gimbal;

//---------------------------------------------------------------------------------------------------


Lazy<UI::App> ui;

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

#if APP_CHASSIS_SLAVE && APP_USE_SWERVE
    chassis.init();
    schedule([]() { chassis->update(); });
#endif

#if APP_GIMBAL_MASTER && APP_USE_GIMBAL
    gimbal.init();
    schedule([]() { gimbal->update((void *)cmd->getMsgBus()); });
#endif

    // TestModule
    if constexpr (APP_USE_TEST) {
        TestModule::instance()->init();
    }

#if APP_USE_COMM
    schedule([]() { CommManager::instance().txTask(); });
#endif

    schedule([]() { chassis->update(); });

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
