#include "AppManager.hpp"

#include "sdkconfig.h"

#include "cmsis_os2.h"

#include "MotorManager.hpp"

#include "INS.hpp"
#include "Bmi088.hpp"

#include "Cmd.hpp"

#include "Buzzer.hpp"

#include "UI/App.hpp"

#include "Mecanum.hpp"
#include "Standard.hpp"


extern SPI_HandleTypeDef IMU_SPI;

extern TIM_HandleTypeDef BEEP_TIMER;

//---------------------------------------------------------------------------------------------------
// INS
BMI088 *bmi088;
INS_SYS::INS *ins;
const AccCali_s accCali = {
    // default accelerometer calibration
    .accel_T = { { 1.010860f, 0.015129f, -0.001459f },
                 { 0.001142f, 1.009152f, 0.006399f },
                 { -0.005477f, 0.002071f, 1.013539f } },
    .accel_offs = { -34.944336f, -3.310059f, 107.792969f }
};
const GyroCali_s gyroCali = {
    // default gyroscope calibration
    .gx_bias = -1.93095636f, .gy_bias = -5.93262482f, .gz_bias = 0.222163752f,
    .gx_tco_k = 0.f,         .gx_tco_b0 = 0.f,        .gy_tco_k = 0.f,
    .gy_tco_b0 = 0.f,        .gz_tco_k = 0.f,         .gz_tco_b0 = 0.f
};

//---------------------------------------------------------------------------------------------------
// CMD
Cmd *cmd;

//---------------------------------------------------------------------------------------------------
// Ctrl
CHASSIS::MECANUM::Mecanum *chassis;
GIMBAL::STANDARD::Standard *gimbal;


UI::App *ui;
//---------------------------------------------------------------------------------------------------

void ctrlTask(void *_param)
{
    while (true) {
        gimbal->update(_param);
        chassis->update(_param);
        vTaskDelay(1);
    }
}

void INSTask(void *_param)
{
    while (true) {
        // read BMI088 data
        bmi088->readRaw(); // read raw 6 axis data from device
        bmi088->read();    // serialize data to real format

        // load raw INS needed data, you must transform the raw imu data to correct order
        // the order of axis is defined as:
        /*
                     Z
                     |
                     |
                     |
                     |     X:HEAD
                     |   /
                     | /
            Y<-------ROBOT 
        */
        INS_SYS::IMUSensorRawData_s data = {
            .a = { .x = bmi088->getRawAccelX(),
                   .y = bmi088->getRawAccelY(),
                   .z = static_cast<int16_t>(-bmi088->getRawAccelZ()),
                   .transK = bmi088->getAccelMappingVaule() },
            .g = { .x = bmi088->getRawGyroX(),
                   .y = bmi088->getRawGyroY(),
                   .z = static_cast<int16_t>(-bmi088->getRawGyroZ()),
                   .transK = bmi088->getGyroMappingVaule() },
            // .m = NULL TODO:
        };

        // update INS
        ins->update(&data, bmi088->getTimestamp(), bmi088->getTemperature());
        vTaskDelay(1);
    }
}

//---------------------------------------------------------------------------------------------------
// AppManager
void AppManager::createApp()
{
    // INS Continuous Task
    xTaskCreate(INSTask, "ins_task", 256, nullptr, osPriorityNormal, nullptr);

    // Cmd-Polling Continuous Task
    xTaskCreate([](void *_param) { cmd->task(); }, "cmd_task", 256,
                (void *)cmd->getMsgBus(), osPriorityNormal, nullptr);

    // Robot-Ctrl Continuous Task
    xTaskCreate(ctrlTask, "ctrl_task", 256, (void *)cmd->getMsgBus(),
                osPriorityRealtime, nullptr);

    // Test-Module Continuous Task
    if constexpr (APP_USE_TEST) {
        // xTaskCreate(
        //         [](void *_param) -> void { TestModule::instance()->task(); },
        //         "test_task", 256, nullptr, osPriorityNormal, nullptr);
    }

    // Motor-Sending Continuous Task
    xTaskCreate(
            [](void *_param) -> void {
                PINYMOTOR::MotorManager::instance()->ctrlTask();
            },
            "motor_task", 512, nullptr, osPriorityRealtime, nullptr);

    // Buzzer Once Task
    xTaskCreate(
            [](void *_param) -> void {
                BUZZER::Buzzer::getInstance().playPinyCore();
                vTaskDelete(nullptr); // 否则会进ExistError
            },
            "buzzer_task", 64, nullptr, osPriorityNormal, nullptr);

    if constexpr (APP_USE_UI) {
        xTaskCreate([](void *_param) { ui->task(_param); }, "ui_task", 256,
                    nullptr, osPriorityHigh, nullptr);
    }
}

void AppManager::initApp()
{
    // Cmd
    cmd = new Cmd;
    cmd->init();

    // INS
    bmi088 = new BMI088;
    ins = new INS_SYS::INS;
    bmi088->init(&IMU_SPI);
    ins->init(accCali, gyroCali);

    // Chassis
    chassis = new CHASSIS::MECANUM::Mecanum();

    // Gimbal
    gimbal = new GIMBAL::STANDARD::Standard();

    // Buzzer
    BUZZER::Buzzer::getInstance().init(&BEEP_TIMER, BEEP_TIM_CHANNEL,
                                       BEEP_APB_FREQ);

    // TestModule
    if constexpr (APP_USE_TEST) {
        // TestModule::instance()->init();
    }

    if constexpr (APP_USE_UI) {
        extern UART_HandleTypeDef UI_UART;
        ui = new UI::App(UI_UART, UI_ROBOT_ID);
    }

    // Generate threads at the end
    this->createApp();
}
