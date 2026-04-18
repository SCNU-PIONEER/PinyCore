#include "TestModule.hpp"
#include "TestDM/TestDM.hpp"
#include "FreeRTOS.h"
#include "Projdefs.hpp"
#include "sdkconfig.h"
#include "task.h"
#include <cmath>

using namespace TEST;
using namespace PINYMOTOR;

#if TEST_DM
TestDM testDM;
#endif

void TestModule::init()
{
#if TEST_DM
    schedule([]() { testDM.test(); });
    testDM.rebuildMotor(DmMotorModel_e::DM4310, 2, WorkMode_e::VDES);
#endif
}

void TestModule::update()
{
    for (auto &task : this->tasks) {
        task();
    }
}

void TestModule::task()
{
    while (true) {
        this->update();
        vTaskDelay(100);
    }
}
