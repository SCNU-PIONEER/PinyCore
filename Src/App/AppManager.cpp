#include "AppManager.hpp"

#include "cmsis_os2.h"
#include "sdkconfig.h"

#include "MotorManager.hpp"

#include "INS.hpp"

#include "mecanum.hpp"
#include "Chassis.hpp"

#include "cmd.hpp"

#include "bmi088.hpp"

#include "testModule.hpp"


extern SPI_HandleTypeDef IMU_SPI;

//---------------------------------------------------------------------------------------------------
// INS
BMI088 bmi088;
INS ins;
const AccCali_s accCali = {
    // default accelerometer calibration
    .accel_T = { { 1.010860f, 0.015129f, -0.001459f },
                 { 0.001142f, 1.009152f, 0.006399f },
                 { -0.005477f, 0.002071f, 1.013539f } },
    .accel_offs = { -34.944336f, -3.310059f, 107.792969f }
};
const GyroCali_s gyroCali = {
    // default gyroscope calibration
    .gx_bias = 1.0695599f,
    .gy_bias = -0.03854797f,
    .gz_bias = -1.87499213f,
    .gx_tco_k = 0.f,
    .gx_tco_b0 = 0.f,
    .gy_tco_k = 0.f,
    .gy_tco_b0 = 0.f,
    .gz_tco_k = 0.f,
    .gz_tco_b0 = 0.f
};

//---------------------------------------------------------------------------------------------------
// CMD
Cmd cmd;

//---------------------------------------------------------------------------------------------------
// Ctrl
Mecanum mecanum;
Chassis chassis(&mecanum);


//---------------------------------------------------------------------------------------------------
void ctrlTask(void *param)
{
    while (1) {
        chassis.update();
        vTaskDelay(1);
    }
}

void INSTask(void *param)
{
    while (1) {
        // read BMI088 data
        bmi088.readRaw();   // read raw 6 axis data from device
        bmi088.read();      // serialize data to real format

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
            .a = { .x = bmi088.getRawAccelX(),
                   .y = bmi088.getRawAccelY(),
                   .z = bmi088.getRawAccelZ(),
                   .transK = bmi088.getAccelMappingVaule() },
            .g = { .x = bmi088.getRawGyroX(),
                   .y = bmi088.getRawGyroY(),
                   .z = bmi088.getRawGyroZ(),
                   .transK = bmi088.getGyroMappingVaule() },
            // .m = NULL TODO:
        };

        // update INS
        ins.update(&data, bmi088.getTimestamp(), bmi088.getTemperature());
        vTaskDelay(1);
    }
}

//---------------------------------------------------------------------------------------------------
// AppManager
void AppManager::createApp()
{
    // INS Task
    xTaskCreate(INSTask, "ins_task", 256, NULL, osPriorityNormal, NULL);

    // Cmd Polling Task
    xTaskCreate(
        [](void *param) -> void { cmd.task(); },
        "cmd_task", 256, NULL, osPriorityNormal, NULL);

    // Robot Ctrl Task
    xTaskCreate(ctrlTask, "ctrl_task", 256, NULL, osPriorityRealtime, NULL);

    // Test Module Task
    xTaskCreate([](void *param) -> void { TestModule::instance()->task(); },
                "test_task", 256, this, osPriorityNormal, NULL);

    // Motor Sending Task
    xTaskCreate(
        [](void *param) -> void {
            PINYMOTOR::MotorManager::instance()->ctrlTask();
        },
        "motor_task", 256, NULL, osPriorityRealtime, NULL);
}

void AppManager::initApp()
{
    // INS
    bmi088.init(&IMU_SPI);
    ins.init(accCali, gyroCali);

    // cmd
    cmd.init();

    // TestModule
    TestModule::instance()->init();
    
    // Generate threads at the end
    this->createApp();
}
