#include "AppManager.hpp"
#include "cmsis_os2.h"
#include "testModule.hpp"
#include "MotorManager.hpp"
#include "sdkconfig.h"

#include "mecanum.hpp"
#include "Chassis.hpp"
#include "cmd.hpp"
#include "rcMsgHandler.hpp"
#include "rttMsgHandler.hpp"


extern UART_HandleTypeDef RC_UART;

Cmd cmd;

Mecanum mecanum;
Chassis chassis(&mecanum);

chassisMsg cmsg;

RTTMsgHandler rttMsgHandler(&cmsg, NULL);
rcMsgHandler rcMsgHandler(&cmsg, NULL);

void cmdTask(void *param)
{
    rcMsgHandler.init(&RC_UART);

    cmd.addHandler(&rcMsgHandler);
    cmd.addHandler(&rttMsgHandler);
    cmd.addObserver(&cmsg, &chassis);

    while (1) {
        cmd.parseMsg();
        vTaskDelay(10);
    }
}

void ctrlTask(void *param)
{
    while (1) {
        chassis.update();
        vTaskDelay(1);
    }
}

void AppManager::createApp()
{
    PINYMOTOR::MotorManager::instance()->taskCreate();

    // TestModule::instance()->taskCreate();

    xTaskCreate(cmdTask, "cmd_task", 256, NULL, osPriorityNormal, NULL);
    xTaskCreate(ctrlTask, "ctrl_task", 256, NULL, osPriorityRealtime, NULL);
}
