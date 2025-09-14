#include "AppManager.hpp"
#include "sdkconfig.h"
#include "cmsis_os2.h"
#include "MotorManager.hpp"
#if APP_USE_COMM
#include "CommManager.hpp"
#endif
#include "INS.hpp"
#include "Cmd.hpp"
#include "Buzzer.hpp"
#include "UI/UIApp.hpp"
#include "test/TestModule.hpp"
#include CHASSIS_FILE

extern TIM_HandleTypeDef BEEP_TIMER;

//---------------------------------------------------------------------------------------------------

#if APP_USE_INS
INS_SYS::INS ins;
#endif

//---------------------------------------------------------------------------------------------------

Cmd cmd;

// Ctrl
CHASSIS::Swerve chassis;

//---------------------------------------------------------------------------------------------------


#if APP_USE_UI
UI::App ui(UI_ROBOT_ID);
#endif

//---------------------------------------------------------------------------------------------------

void AppManager::initApp()
{
    // Buzzer
    BUZZER::Buzzer::getInstance().init(&BEEP_TIMER, BEEP_TIM_CHANNEL,
                                       BEEP_APB_FREQ);

#if APP_USE_COMM
    schedule([]() { CommManager::instance().rxTask(); });
#endif

    cmd = new Cmd();

#if APP_USE_UI
    ui.init();
    schedule([]() { ui.task(); });
#endif

#if APP_USE_DAEMONS
    schedule([]() { Daemons::instance().update(); });
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

void AppManager::schedule(std::function<void()> _callback)
{
    tasks.push_back(std::move(_callback));
}

void AppManager::createApp()
{
    // Robot-Ctrl Continuous Task
    xTaskCreate(AppManager::ctrlTask, "ctrl_task", 256, this,
                osPriorityRealtime, nullptr);

    // Test-Module Continuous Task
    if constexpr (APP_USE_TEST) {
        xTaskCreate(
                [](void *_param) -> void { TestModule::instance()->task(); },
                "test_task", 256, nullptr, osPriorityNormal, nullptr);
    }

    // Motor-Sending Continuous Task
    xTaskCreate(
            [](void *_param) -> void {
                PINYMOTOR::MotorManager::instance()->ctrlTask();
            },
            "motor_task", 512, nullptr, osPriorityRealtime6, nullptr);

    // Buzzer Once Task
    xTaskCreate(
            [](void *_param) -> void {
                BUZZER::Buzzer::getInstance().playPinyCore();
                vTaskDelete(nullptr); // 否则会进ExistError
            },
            "buzzer_task", 64, nullptr, osPriorityNormal, nullptr);
}

void AppManager::initApp()
{
    cmd.init();

    // Buzzer
    BUZZER::Buzzer::getInstance().init(&BEEP_TIMER, BEEP_TIM_CHANNEL,
                                       BEEP_APB_FREQ);

#if APP_USE_UI
    ui.init();
    schedule([]() { ui.task(); });
#endif

    // TestModule
    if constexpr (APP_USE_TEST) {
        TestModule::instance()->init();
    }

    schedule([]() { chassis.update(); });

    // Generate threads at the end
    this->createApp();
}