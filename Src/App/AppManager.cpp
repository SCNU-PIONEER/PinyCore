#include "AppManager.hpp"
#include "cmsis_os2.h"
#include "testModule.hpp"
#include "MotorManager.hpp"
#include "Bsp_can.hpp"

void AppManager::createApp()
{
    PINYMOTOR::MotorManager::instance()->taskCreate();

    TestModule::instance()->taskCreate();
}
