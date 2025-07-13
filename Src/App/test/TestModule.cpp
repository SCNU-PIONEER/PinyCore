#include "TestModule.hpp"

#include "Can/Bsp_can.hpp"

#include "cmsis_os2.h"
#include "task.h"
#include "sdkconfig.h"

#include "Rc.hpp"

#include "TestModuleMotorVelCtrlState.hpp"
#include "TestModuleMotorPosCtrlState.hpp"
#include "TestModuleStopState.hpp"
#include <memory>

#include "PidBasic.hpp"

static positonalPid_s GM6020VelPidConfig = {
    .kp = 0.05f,
    .ki = 0.0f,
    .kd = 0.0f,
    .dt = 1.f / 100.f,
    .iMax = GM6020_TORQ_MAX,
    .outMax = GM6020_TORQ_MAX,
    .deadband = 0.f,
};
static positonalPid GM6020VelPid(GM6020VelPidConfig);

void TestModule::init()
{
    stateFactory_.addState("MotorVelCtrl",
                           std::make_unique<MotorVelCtrlState>(this));
    stateFactory_.addState("MotorPosCtrl",
                           std::make_unique<MotorPosCtrlState>(this));
    stateFactory_.addState("MotorStop", std::make_unique<MotorStopState>(this));
    stateFactory_.init(stateFactory_.getNextState("MotorStop"));

    extern canHandle HCAN1;

    // Test DM4310 ----------------------------------------
    static PINYMOTOR::InitConfig_s testDM4310Config = {
        (uint32_t *)(&hcan1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::MIT_TT,
        static_cast<uint8_t>(1),
        static_cast<float>(1.0f),
        nullptr,
        nullptr,
        nullptr
    };
    // this->testDM4310Motor = std::make_unique<PINYMOTOR::DMMOTOR::DM4310>(
    //         "DM4310", testDM4310Config);

    // Test GM3510 ----------------------------------------
    static PINYMOTOR::InitConfig_s testGM3510Config = {
        (uint32_t *)(&HCAN1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::TRIP_VOLT,
        static_cast<uint8_t>(3),
        static_cast<float>(1.0f),
        nullptr,
        nullptr,
        nullptr
    };
    // this->testGM3510Motor = std::make_unique<PINYMOTOR::DJI_ODMOTOR::GM3510>(
    //         "GM3510", testGM3510Config);

    // Test GM6020 ----------------------------------------
    static PINYMOTOR::InitConfig_s testGM6020Config = {
        (uint32_t *)(&hcan1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::QUAD_VOLT,
        static_cast<uint8_t>(7),
        static_cast<float>(100.0f),
        nullptr,
        reinterpret_cast<PID *>(&GM6020VelPid),
        nullptr
    };
    this->testGM6020Motor = std::make_unique<PINYMOTOR::DJIMOTOR::GM6020>(
            "GM6020", testGM6020Config);
}

void TestModule::update()
{
    rcMsg = RC::Rc::instance().getData();
    // stateFactory_.update();
}

void TestModule::task()
{
    while (1) {
        this->update();
        vTaskDelay(100);
    }
}