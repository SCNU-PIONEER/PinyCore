#include "testModule.hpp"

#include "Can/Bsp_can.hpp"

#include "cmsis_os2.h"
#include "task.h"

void TestModule::init()
{
    extern canHandle hcan1;
    // static PINYMOTOR::InitConfig_s testDM4310Config = {
    //     (uint32_t *)(&hcan1),
    //     PINYMOTOR::ComType_e::CAN,
    //     PINYMOTOR::WorkMode_e::MIT_TT,
    //     static_cast<uint8_t>(1),
    //     static_cast<float>(1.0f),
    // };
    // static PINYMOTOR::DM4310 testDM4310Motor("testdm4310", testDM4310Config);
    // this->testDM4310Motor_ = &testDM4310Motor;

    static PINYMOTOR::InitConfig_s testGM3510Config = {
        (uint32_t *)(&hcan1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::TRIP_VOLT,
        static_cast<uint8_t>(3),
        static_cast<float>(1.0f),
    };
    static PINYMOTOR::GM3510 testGM3510Motor("testgm3510", testGM3510Config);
    this->testGM3510Motor_ = &testGM3510Motor;

    // static PINYMOTOR::InitConfig_s testGM6020Config = {
    //     (uint32_t *)(&hcan1),
    //     PINYMOTOR::ComType_e::CAN,
    //     PINYMOTOR::WorkMode_e::QUAD_VOLT,
    //     static_cast<uint8_t>(4),
    //     static_cast<float>(1.0f),
    // };
    // static PINYMOTOR::GM6020 testGM6020Motor("testgm6020", testGM6020Config);
    // this->testGM6020Motor_ = &testGM6020Motor;
}

void TestModule::task()
{
    for (;;) {
        this->motorTestTask();
        vTaskDelay(1000);
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

float debugVolt = 1.0f;
float debugTorq = 0.1f;
auto TestModule::motorTestTask() -> void
{
    // this->testDM4310Motor_->cmd(PINYMOTOR::MotorCmdType_e::ON);
    // this->testDM4310Motor_->cmd(PINYMOTOR::MotorCmdType_e::SET_TORQ, debugTorq);

    // this->testGM6020Motor_->cmd(PINYMOTOR::MotorCmdType_e::ON);
    // this->testGM6020Motor_->cmd(PINYMOTOR::MotorCmdType_e::SET_VOLT, debugVolt);

    this->testGM3510Motor_->cmd(PINYMOTOR::MotorCmdType_e::ON);
    this->testGM3510Motor_->cmd(PINYMOTOR::MotorCmdType_e::SET_VOLT, debugVolt);
}
