#include "testModule.hpp"

#include "Can/Bsp_can.hpp"

#include "cmsis_os2.h"
#include "task.h"

void TestModule::init()
{
    extern canHandle hcan1;
    static PINYMOTOR::InitConfig_s testConfig = {
        (uint32_t *)(&hcan1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::MIT_TT,
        static_cast<uint8_t>(1),
        static_cast<float>(500.0f),
    };
    static PINYMOTOR::DM4310 testMotor("test", testConfig);
    this->testMotor_ = &testMotor;
}

void TestModule::task()
{
    for (;;) {
        this->motorTestTask();
    }
}

void TestModule::taskCreate()
{
    this->init();
    
    xTaskCreate([](void* param) -> void {
        TestModule *instance = reinterpret_cast<TestModule *>(param);
        instance->task();
    }, "test_task", 256, this, osPriorityNormal, NULL);
}

auto TestModule::motorTestTask() -> void
{
    debugCnt++;
}
