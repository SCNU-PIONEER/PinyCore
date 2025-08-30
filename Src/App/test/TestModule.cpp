#include "TestModule.hpp"

#include "Can/Bsp_can.hpp"

#include "PidBasic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "sdkconfig.h"
#include "Rc.hpp"

using namespace TEST;

TestModule *TestModule::instance()
{
    static TestModule instance;
    return &instance;
}

void TestModule::init() {}

void TestModule::update()
{
    rcMsg = RC::Rc::instance().getData();
    stateFactory_.update();
}

void TestModule::task()
{
    while (true) {
        this->update();
        vTaskDelay(100);
    }
}
