#include "testModule.hpp"

#include "Can/Bsp_can.hpp"

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

void TestModule::motorTestTask()
{
    
}