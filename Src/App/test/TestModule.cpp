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
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::MOTOR_VEL_CTRL),
                           std::make_unique<MotorVelCtrlState>(this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::MOTOR_POS_CTRL),
                           std::make_unique<MotorPosCtrlState>(this));
    stateFactory_.addState(static_cast<uint8_t>(FSMState_e::MOTOR_STOP),
                           std::make_unique<MotorStopState>(this));
    stateFactory_.init(stateFactory_.getNextState(
            static_cast<uint8_t>(FSMState_e::MOTOR_STOP)));

    extern canHandle HCAN1;

    // Test DM4310 ----------------------------------------
    PINYMOTOR::InitConfig_s testDM4310Config = {
        .pComHandle = (uint32_t *)(&HCAN1),
        .comType = PINYMOTOR::ComType_e::CAN,
        .workMode = PINYMOTOR::WorkMode_e::MIT_TT,
        .offsetId = static_cast<uint8_t>(1),
        .txFreq = 1.0f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr
    };
    this->testDM4310Motor = std::make_unique<PINYMOTOR::DMMOTOR::DM4310>(
            "DM4310", testDM4310Config);

    // Test GM3510 ----------------------------------------
    PINYMOTOR::InitConfig_s testGM3510Config = {
        .pComHandle = (uint32_t *)(&HCAN1),
        .comType = PINYMOTOR::ComType_e::CAN,
        .workMode = PINYMOTOR::WorkMode_e::TRIP_VOLT,
        .offsetId = static_cast<uint8_t>(3),
        .txFreq = 1.0f,
        .posPID = nullptr,
        .velPID = nullptr,
        .torqPID = nullptr
    };
    this->testGM3510Motor = std::make_unique<PINYMOTOR::DJI_ODMOTOR::GM3510>(
            "GM3510", testGM3510Config);

    // Test GM6020 ----------------------------------------
    PINYMOTOR::InitConfig_s testGM6020Config = {
        .pComHandle = (uint32_t *)(&HCAN1),
        .comType = PINYMOTOR::ComType_e::CAN,
        .workMode = PINYMOTOR::WorkMode_e::QUAD_VOLT,
        .offsetId = static_cast<uint8_t>(7),
        .txFreq = (100.0f),
        .posPID = nullptr,
        .velPID = new PositonalPid(0.05f, 0.f, 0.f, 0.01,
                                   PINYMOTOR::DJIMOTOR::GM6020::TORQ_MAX,
                                   PINYMOTOR::DJIMOTOR::GM6020::TORQ_MAX, 0.f),
        .torqPID = nullptr
    };
    this->testGM6020Motor =
            std::make_unique<DJIMOTOR::GM6020>("GM6020", testGM6020Config);
}

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
