#include "AppManager.hpp"

#include "cmsis_os2.h"
#include "sdkconfig.h"

#include "MotorManager.hpp"

#include "INS.hpp"

#include "mecanum.hpp"
#include "Chassis.hpp"

#include "cmd.hpp"
#include "rcMsgHandler.hpp"
#include "rttMsgHandler.hpp"

#include "bmi088.hpp"

#include "testModule.hpp"


extern UART_HandleTypeDef RC_UART;
extern SPI_HandleTypeDef IMU_SPI;

BMI088 bmi088;
INS ins;

Cmd cmd;

Mecanum mecanum;
Chassis chassis(&mecanum);



void cmdTask(void *param)
{

    // cmd.addObserver(&cmsg, &chassis);
    cmd.init();

    while (1) {
        cmd.parseMsg();
    }
}

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
        
        // load raw INS needed data
        IMUSensorData_s data = { .a = { .x = bmi088.getAccelX(),
                                        .y = bmi088.getAccelY(),
                                        .z = bmi088.getAccelZ() },
                                 .g = { .x = bmi088.getGyroX(),
                                        .y = bmi088.getGyroY(),
                                        .z = bmi088.getGyroZ() },
                                 // .m = NULL TODO:
                                 .temperature = bmi088.getTemperature() };
        
        // update INS
        ins.update(&data, bmi088.getTimestamp());
        vTaskDelay(1);
    }
}

void AppManager::createApp()
{
    // Motor Sending Task
    PINYMOTOR::MotorManager::instance()->taskCreate();

    // INS Task
    xTaskCreate(INSTask, "ins_task", 256, NULL, osPriorityNormal, NULL);

    // Cmd Polling Task
    xTaskCreate(cmdTask, "cmd_task", 256, NULL, osPriorityNormal, NULL);

    // Robot Ctrl Task
    xTaskCreate(ctrlTask, "ctrl_task", 256, NULL, osPriorityRealtime, NULL);

    // Test Module Task
    TestModule::instance()->taskCreate();
}

void AppManager::initApp()
{
    // INS 
    bmi088.init(&IMU_SPI);

    // Generate threads at the end
    this->createApp();
}


