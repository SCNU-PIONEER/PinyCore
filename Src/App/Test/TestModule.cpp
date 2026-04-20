#include "TestModule.hpp"
#include "TestDM/TestDM.hpp"
#include "FreeRTOS.h"
#include "Projdefs.hpp"
#include "sdkconfig.h"
#include "task.h"
#include <cmath>

#ifdef CONFIG_DMMOTOR_TEST
    #include "DMMotorTest/DMMotorTest.hpp"
#endif


using namespace TEST;
using namespace PINYMOTOR;

#if TEST_DM
TestDM testDM;
#endif

void TestModule::init()
{
#if TEST_DM
    schedule([]() { testDM.test(); });
    testDM.rebuildMotor(DmMotorModel_e::DM8009, 7, WorkMode_e::VDES);
#endif
}

void TestModule::update()
{
    for (auto &task : this->tasks) {
        task();
        vTaskSuspend(getTaskHandler());
    }
}

void TestModule::task()
{
    while (true) {
        this->update();
        vTaskDelay(100);
    }
}
