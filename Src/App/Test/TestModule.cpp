#include "TestModule.hpp"
#include "DMMotor.hpp"
#include "TestDM/TestDM.hpp"
#include "TestLK/TestLK.hpp"
#include "TestMT/TestMT.hpp"
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

#if TEST_LK
TestLK testLK;
#endif

#if TEST_MT
TestMT testMT;
#endif

void TestModule::init()
{
#if TEST_DM
    schedule([]() { testDM.test(); });
    testDM.rebuildMotor(DmMotorModel_e::DM4310, 1);
#endif

#if TEST_LK
    //BROADCAST MODE
    schedule([]() { testLK.test(); });
    testLK.addMotor(LkMotorModel_e::MG4005, 1);
    testLK.addMotor(LkMotorModel_e::MF9025, 2);
#endif

#if TEST_MT
    schedule([]() { testMT.test(); });
    testMT.buildMotor(MTMotorModel_e::RMD_X4_10, 1);
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
