#include "FreeRTOS.h"
#include "GM6020.hpp"
#include "M3508.hpp"
#include "Projdefs.hpp"
#include "Soc.hpp"
#include "StmLog.hpp"
#include "task.h"
#include "TestDJI.hpp"
#include <cstdint>

using namespace TEST;
using namespace PINYMOTOR;

InitConfig_s configDJI = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .motorType = SupportMotor_e::DJI,
    .offsetId = 2,
    .txFreq = 500.0f,
};

void TestDJI::test()
{
    testON();
    vTaskDelay(100);
    testRot(0.5f);
    vTaskDelay(1000);
    testOFF();
    vTaskDelay(100);
    vTaskSuspend(getTaskHandler());
}

void TestDJI::rebuildMotor(DJIMotorModel_e _model, uint8_t _id)
{
    curModel_ = _model;
    configDJI.offsetId = _id;
    switch (_model) {
    case DJIMotorModel_e::GM6020:
        motor_ = new DJIMOTOR::GM6020("gm6020", configDJI, DJIMOTOR::WorkMode_e::QUAD_CURR);
        break;
    case DJIMotorModel_e::M3508:
        motor_ = new DJIMOTOR::M3508("m3508", configDJI, DJIMOTOR::WorkMode_e::QUAD_CURR);
        break;
    }
}

void TestDJI::testRot(float _elec) { motor_->cmdElec(_elec); }

void TestDJI::testON() { motor_->cmd(MotorCmdType_e::ON); }

void TestDJI::testOFF() { motor_->cmd(MotorCmdType_e::OFF); }