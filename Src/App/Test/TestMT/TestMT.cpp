#include "TestMT.hpp"
#include "MTMotor.hpp"
#include "RMD-X2-7.hpp"
#include "RMD-X4-10.hpp"
#include "RMD-X4-36.hpp"

#include "FreeRTOS.h"
#include "Projdefs.hpp"
#include "Soc.hpp"
#include "StmLog.hpp"
#include "task.h"
#include "magic_enum/magic_enum.hpp"
#include <cmath>
#include <cstdint>

using namespace TEST;
using namespace PINYMOTOR;

InitConfig_s configMT = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .motorType = SupportMotor_e::MAI_TA,
    .offsetId = 1,
    .txFreq = 500.0f,
    .isReverse = false,
};

void TestMT::buildMotor(MTMotorModel_e _model, uint8_t _id)
{
    configMT.offsetId = _id;
    switch (_model) {
    case MTMotorModel_e::RMD_X2_7:
        motor_ = new MTMOTOR::RMDX27("MT_RMDX27", configMT, MTMOTOR::WorkMode_e::TORQ);
        break;
    case MTMotorModel_e::RMD_X4_10:
        motor_ = new MTMOTOR::RMDX410("MT_RMDX410", configMT, MTMOTOR::WorkMode_e::TORQ);
        break;
    case MTMotorModel_e::RMD_X4_36:
        motor_ = new MTMOTOR::RMDX436("MT_RMDX436", configMT, MTMOTOR::WorkMode_e::TORQ);
        break;
    }

    /*set test*/
};

void TestMT::test()
{
    /*set test*/

    // switchMTWorkMode(MTMOTOR::WorkMode_e::SETTING);
    // motor_->setMotorZeroAng();
    // vTaskDelay(100);

    testON();
    vTaskDelay(100);
    switchMTWorkMode(MTMOTOR::WorkMode_e::TORQ);
    vTaskDelay(100);

    /*error test*/
    // motor_->readErrCode();

    /*workmode test*/

    // testON();
    // vTaskDelay(100);
    // switchMTWorkMode(MTMOTOR::WorkMode_e::TORQ);
    // testTorq(0.4f);
    // vTaskDelay(1000);

    // testON();
    // vTaskDelay(100);
    // switchMTWorkMode(MTMOTOR::WorkMode_e::SPEED);
    // testSpeed(-1.f);
    // vTaskDelay(1000);

    // testON();
    // vTaskDelay(100);
    // switchMTWorkMode(MTMOTOR::WorkMode_e::ABS_POS);
    // testAbsPos(5.f, 4.f);
    // vTaskDelay(1000);

    // testON();
    // vTaskDelay(100);
    // switchMTWorkMode(MTMOTOR::WorkMode_e::SINGLE_POS);
    // testSinglePos(5.f, 2.f);
    // vTaskDelay(1000);

    // testON();
    // vTaskDelay(100);
    // switchMTWorkMode(MTMOTOR::WorkMode_e::INC_POS);
    // testIncPos(2.f);
    // vTaskDelay(1000);

    // testON();
    // vTaskDelay(100);
    // switchMTWorkMode(MTMOTOR::WorkMode_e::FORCE_POS);
    // testForcePos(1.f, 5.f, 1.f);
    // vTaskDelay(1000);

    // testOFF();
    // vTaskDelay(100);
    vTaskSuspend(getTaskHandler());
}

void TestMT::testON() { motor_->cmd(MotorCmdType_e::ON); };

void TestMT::testOFF() { motor_->cmd(MotorCmdType_e::OFF); }

void TestMT::testTorq(float _torq)
{
    motor_->cmdTorq(_torq);
    vTaskDelay(100);
}

void TestMT::testSpeed(float _vel)
{
    motor_->cmdVel(_vel);
    vTaskDelay(100);
}
void TestMT::testAbsPos(float _velLimit, float _pos)
{
    motor_->cmdPosVel(_pos, _velLimit);
    vTaskDelay(100);
};
void TestMT::testSinglePos(float _velLimit, float _pos)
{
    motor_->cmdPosVel(_pos, _velLimit);
    vTaskDelay(100);
}
void TestMT::testIncPos(float _deltaPos)
{
    motor_->cmdPos(_deltaPos);
    vTaskDelay(100);
}
void TestMT::testForcePos(float _torqLimit, float _velLimit, float _pos)
{
    motor_->cmdMIT(_pos, _velLimit, _torqLimit, 0, 0);
    vTaskDelay(100);
}

void TestMT::switchMTWorkMode(PINYMOTOR::MTMOTOR::WorkMode_e _mode) { motor_->switchCtrlMode(_mode); }
