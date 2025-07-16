#include "TestModule.hpp"

#include "Can/Bsp_can.hpp"

#include "PidBasic.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "sdkconfig.h"
#include "Rc.hpp"

#include "TestModuleMotorVelCtrlState.hpp"
#include "TestModuleMotorPosCtrlState.hpp"
#include "TestModuleStopState.hpp"
#include <memory>


using namespace TEST;
using namespace PINYMOTOR;

void TestModule::init()
{
    stateFactory_.addState(static_cast<uint8_t>(fsmState_e::MotorVelCtrl),
                           std::make_unique<MotorVelCtrlState>(this));
    stateFactory_.addState(static_cast<uint8_t>(fsmState_e::MotorPosCtrl),
                           std::make_unique<MotorPosCtrlState>(this));
    stateFactory_.addState(static_cast<uint8_t>(fsmState_e::MotorStop),
                           std::make_unique<MotorStopState>(this));
    stateFactory_.init(stateFactory_.getNextState(
            static_cast<uint8_t>(fsmState_e::MotorStop)));

    extern canHandle HCAN1;

    // Test DM4310 ----------------------------------------
    PINYMOTOR::InitConfig_s testDM4310Config = { (uint32_t *)(&HCAN1),
                                                 PINYMOTOR::ComType_e::CAN,
                                                 PINYMOTOR::WorkMode_e::MIT_TT,
                                                 static_cast<uint8_t>(1),
                                                 static_cast<float>(1.0f),
                                                 nullptr,
                                                 nullptr,
                                                 nullptr };
    this->testDM4310Motor = std::make_unique<PINYMOTOR::DMMOTOR::DM4310>(
            "DM4310", std::move(testDM4310Config));

    // Test GM3510 ----------------------------------------
    PINYMOTOR::InitConfig_s testGM3510Config = {
        (uint32_t *)(&HCAN1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::TRIP_VOLT,
        static_cast<uint8_t>(3),
        static_cast<float>(1.0f),
        nullptr,
        nullptr,
        nullptr
    };
    this->testGM3510Motor = std::make_unique<PINYMOTOR::DJI_ODMOTOR::GM3510>(
            "GM3510", std::move(testGM3510Config));

    // Test GM6020 ----------------------------------------
    PINYMOTOR::InitConfig_s testGM6020Config = {
        (uint32_t *)(&HCAN1),
        PINYMOTOR::ComType_e::CAN,
        PINYMOTOR::WorkMode_e::QUAD_VOLT,
        static_cast<uint8_t>(7),
        static_cast<float>(100.0f),
        nullptr,
        std::unique_ptr<PID>(new positonalPid(
                0.05f, 0.f, 0.f, 0.01, GM6020_TORQ_MAX, GM6020_TORQ_MAX, 0.f)),
        nullptr
    };
    this->testGM6020Motor = std::make_unique<DJIMOTOR::GM6020>(
            "GM6020", std::move(testGM6020Config));
}

void TestModule::update()
{
    rcMsg = RC::Rc::instance().getData();
    stateFactory_.update();
}

void TestModule::task()
{
    while (1) {
        this->update();
        vTaskDelay(100);
    }
}
