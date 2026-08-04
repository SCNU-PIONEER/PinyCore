#include "TestLK.hpp"
#include "FreeRTOS.h"
#include "MF9025.hpp"
#include "MG4005.hpp"
#include "Projdefs.hpp"
#include "Soc.hpp"
#include "StmLog.hpp"
#include "magic_enum/magic_enum.hpp"
#include "task.h"
#include <cmath>
#include <cstdint>
using namespace TEST;
using namespace PINYMOTOR;
using namespace LKMOTOR;

InitConfig_s configLK = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
    .comType = ComType_e::CAN,
    .motorType = SupportMotor_e::LK,
    .offsetId = 0,
    .txFreq = 1000.0f,
};

void TestLK::test()
{
    testON();
    vTaskDelay(100);

    testTorq(0.1f);
    vTaskDelay(3000);

    testTorq(-0.1f);
    vTaskDelay(3000);

    testOFF();
    vTaskDelay(100);
    vTaskSuspend(getTaskHandler());
}

void TestLK::addMotor(LkMotorModel_e _model, uint8_t _id)
{
    if (motorCount_ >= MAX_MOTORS) {
        LOG::error("TestModule", "LK motor count exceeds MAX_MOTORS(%d)", MAX_MOTORS);
        return;
    }

    configLK.offsetId = _id;

    switch (_model) {
    case LkMotorModel_e::MG4005:
        motors_[motorCount_] = new MG4005("mg4005", configLK, WorkMode_e::QUAD_CURR);
        break;
    case LkMotorModel_e::MF9025:
        motors_[motorCount_] = new MF9025("mf9025", configLK, WorkMode_e::QUAD_CURR);
        break;
    }
    LOG::info("TestModule", "LK motor added: %s (id: %d, index: %d)", magic_enum::enum_name(_model).data(), _id,
              motorCount_);
    motorCount_++;
}

void TestLK::testON()
{
    for (uint8_t i = 0; i < motorCount_; ++i) {
        motors_[i]->cmd(MotorCmdType_e::ON);
    }
    vTaskDelay(100);

    for (uint8_t i = 0; i < motorCount_; ++i) {
        if (motors_[i]->globalState == GlobalState_e::ONLINE) {
            LOG::info("TestModule", "LK motor %d is enabled", i + 1);
        } else {
            LOG::error("TestModule", "LK motor %d enable failed, state: %s", i + 1,
                       magic_enum::enum_name(motors_[i]->globalState).data());
        }
    }
}

void TestLK::testOFF()
{
    for (uint8_t i = 0; i < motorCount_; ++i) {
        motors_[i]->cmd(MotorCmdType_e::OFF);
    }
    vTaskDelay(100);

    for (uint8_t i = 0; i < motorCount_; ++i) {
        if (motors_[i]->globalState == GlobalState_e::OFFLINE) {
            LOG::info("TestModule", "LK motor %d is disabled", i + 1);
        } else {
            LOG::error("TestModule", "LK motor %d disable failed, state: %s", i + 1,
                       magic_enum::enum_name(motors_[i]->globalState).data());
        }
    }
}

void TestLK::testTorq(float _torq)
{
    for (uint8_t i = 0; i < motorCount_; ++i) {
        motors_[i]->cmdTorq(_torq);
    }
    vTaskDelay(100);

    for (uint8_t i = 0; i < motorCount_; ++i) {
        float curTorq = motors_[i]->data().torq;
        LOG::info("TestModule", "LK motor %d torq: %f", i + 1, curTorq);
    }
}

void TestLK::testTorq(uint8_t _index, float _torq)
{
    if (_index >= motorCount_) {
        LOG::error("TestModule", "LK motor index %d out of range (count: %d)", _index, motorCount_);
        return;
    }
    motors_[_index]->cmdTorq(_torq);
    vTaskDelay(100);
    float curTorq = motors_[_index]->data().torq;
    LOG::info("TestModule", "LK motor %d torq: %f", _index + 1, curTorq);
}

PINYMOTOR::LKMOTOR::LKMotor *TestLK::getMotor(uint8_t _index)
{
    if (_index >= motorCount_) {
        return nullptr;
    }
    return motors_[_index];
}
