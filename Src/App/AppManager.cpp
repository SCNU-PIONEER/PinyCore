#include "AppManager.hpp"

#include "cmsis_os2.h"
#include "sdkconfig.h"

#include "MotorManager.hpp"

#include "INS.hpp"

#include "Mecanum.hpp"
#include "Chassis.hpp"

#include "Cmd.hpp"

#include "Bmi088.hpp"

#include "TestModule.hpp"

#include "Buzzer.hpp"


extern SPI_HandleTypeDef IMU_SPI;

extern TIM_HandleTypeDef BEEP_TIMER;

//---------------------------------------------------------------------------------------------------
// INS
BMI088 *bmi088;
INS *ins;
const AccCali_s accCali = {
    // default accelerometer calibration
    .accel_T = { { 1.010860f, 0.015129f, -0.001459f },
                 { 0.001142f, 1.009152f, 0.006399f },
                 { -0.005477f, 0.002071f, 1.013539f } },
    .accel_offs = { -34.944336f, -3.310059f, 107.792969f }
};
const GyroCali_s gyroCali = {
    // default gyroscope calibration
    .gx_bias = 1.0695599f, .gy_bias = -0.03854797f, .gz_bias = -1.87499213f,
    .gx_tco_k = 0.f,       .gx_tco_b0 = 0.f,        .gy_tco_k = 0.f,
    .gy_tco_b0 = 0.f,      .gz_tco_k = 0.f,         .gz_tco_b0 = 0.f
};

//---------------------------------------------------------------------------------------------------
// CMD
Cmd *cmd;

//---------------------------------------------------------------------------------------------------
// Ctrl
CHASSIS::Mecanum *mecanum;
Chassis *chassis;

//---------------------------------------------------------------------------------------------------
void ctrlTask(void *_param)
{
    while (true) {
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
        IMUSensorRawData_s data = {
            .a = { .x = bmi088->getRawAccelX(),
                   .y = bmi088->getRawAccelY(),
                   .z = bmi088->getRawAccelZ(),
                   .transK = bmi088->getAccelMappingVaule() },
            .g = { .x = bmi088->getRawGyroX(),
                   .y = bmi088->getRawGyroY(),
                   .z = bmi088->getRawGyroZ(),
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
    if constexpr (USE_TEST_MODULES) {
        xTaskCreate(
                [](void *_param) -> void { TestModule::instance()->task(); },
                "test_task", 256, nullptr, osPriorityNormal, nullptr);
    }

    // Motor-Sending Continuous Task
    xTaskCreate(
            [](void *_param) -> void {
                PINYMOTOR::MotorManager::instance()->ctrlTask();
            },
            "motor_task", 256, nullptr, osPriorityRealtime, nullptr);

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
    // Cmd
    cmd = new Cmd;
    cmd->init();

    // INS
    bmi088 = new BMI088;
    ins = new INS;
    bmi088->init(&IMU_SPI);
    ins->init(accCali, gyroCali);

    // Chassis
    mecanum = new CHASSIS::Mecanum;
    chassis = new Chassis(mecanum);

    // Buzzer
    BUZZER::Buzzer::getInstance().init(&BEEP_TIMER, BEEP_TIM_CHANNEL,
                                       BEEP_APB_FREQ);

    // TestModule
    if constexpr (USE_TEST_MODULES) {
        TestModule::instance()->init();
    }

    // Generate threads at the end
    this->createApp();
}
