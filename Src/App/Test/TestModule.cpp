#include "TestModule.hpp"

#include "Can/Bsp_can.hpp"

#include "PidBasic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "sdkconfig.h"

using namespace TEST;

void TestModule::init() {}

void TestModule::update() { stateFactory_.update(); }

void TestModule::task()
{
    while (true) {
        this->update();
        vTaskDelay(100);
    }
}
