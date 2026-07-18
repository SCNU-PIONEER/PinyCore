#include "TestDM.hpp"
#include "DM4310.hpp"
#include "DM3507.hpp"
#include "DM3519.hpp"
#include "DM4340.hpp"
#include "DM6006.hpp"
#include "DM8009.hpp"
#include "DM10010L.hpp"
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

InitConfig_s configDM = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .workMode = WorkMode_e::EMIT,
    .offsetId = 2,
    .txFreq = 500.0f,
};

DMMOTOR::Reg_s regDM = {
    .regId = DMMOTOR::RegId_e::DM_REG_CTRL_MODE,
    .dat = { 0x00, 0x00, 0x00, 0x00 },
    .isWrite = false,
    .isRead = false,
    .isStorage = false,
};

DMMOTOR::RegValue_u regValueDM = {
    .dat = { 0x00, 0x00, 0x00, 0x00 },
};

void TestDM::test()
{
    testON();
    vTaskDelay(100);
    switchdmWorkMode(WorkMode_e::VDES);
    testVdes(2.f);
    vTaskDelay(3000);

    switchdmWorkMode(WorkMode_e::PDESVDES);
    testPdesVdes(2.f, 4.f);

    switchdmWorkMode(WorkMode_e::MIT_TT);
    testMitTt(0.2f);
    vTaskDelay(3000);

    switchdmWorkMode(WorkMode_e::MIT_VDES);
    testMITVdes(1.f, 1.f);
    vTaskDelay(3000);

    switchdmWorkMode(WorkMode_e::MIT_VDESPDES);
    testMitVdesPdes(2.f, 0.f, 5.f, 1.f);

    switchdmWorkMode(WorkMode_e::EMIT);
    testEmit(1.f, 4.f, 1.f);
    vTaskDelay(3000);

    testOFF();
    vTaskDelay(100);
    vTaskSuspend(getTaskHandler());
}

void TestDM::rebuildMotor(DmMotorModel_e _model, uint8_t _id, PINYMOTOR::WorkMode_e _mode)
{
    curModel_ = _model;
    configDM.workMode = _mode;
    configDM.offsetId = _id;
    switch (_model) {
    case DmMotorModel_e::DM4310:
        motor_ = new DMMOTOR::DM4310("dm4310", configDM);
        break;
    case DmMotorModel_e::DM3507:
        motor_ = new DMMOTOR::DM3507("dm3507", configDM);
        break;
    case DmMotorModel_e::DM3519:
        motor_ = new DMMOTOR::DM3519("dm3519", configDM);
        break;
    case DmMotorModel_e::DM4340:
        motor_ = new DMMOTOR::DM4340("dm4340", configDM);
        break;
    case DmMotorModel_e::DM6006:
        motor_ = new DMMOTOR::DM6006("dm6006", configDM);
        break;
    case DmMotorModel_e::DM8009:
        motor_ = new DMMOTOR::DM8009("dm8009", configDM);
        break;
    case DmMotorModel_e::DM10010L:
        motor_ = new DMMOTOR::DM10010L("dm10010l", configDM);
        break;
    }
    motor_->registerReg(&regDM, &regValueDM);
}

void TestDM::testON()
{
    motor_->cmd(MotorCmdType_e::ON);
    vTaskDelay(100);
    DMMOTOR::ErrorCode_e errorCode2 = motor_->getErrorcode();
    if (errorCode2 != DMMOTOR::ErrorCode_e::MOTOR_ENABLE) {
        LOG::error("TestModule", "test on, %s error code: %s,", magic_enum::enum_name(curModel_).data(),
                   magic_enum::enum_name(errorCode2).data());
    } else {
        LOG::info("TestModule", " %s is enabled", magic_enum::enum_name(curModel_).data());
    }
}

void TestDM::testOFF()
{
    motor_->cmd(MotorCmdType_e::OFF);
    vTaskDelay(100);
    DMMOTOR::ErrorCode_e errorCode2 = motor_->getErrorcode();
    if (errorCode2 != DMMOTOR::ErrorCode_e::MOTOR_DISABLE) {
        LOG::error("TestModule", "test off, %s error code: %s,", magic_enum::enum_name(curModel_).data(),
                   magic_enum::enum_name(errorCode2).data());
    } else {
        LOG::info("TestModule", " DM4310 is disabled");
    }
}

void TestDM::testMitTt(float _torq)
{
    motor_->cmdTorq(_torq);
    vTaskDelay(100);
    float curtorq = motor_->data().torq;
    if (fabsf(curtorq) - _torq > 0.1) {
        LOG::error("TestModule", "test MIT_TT, %s torq is %f, beyond %f", magic_enum::enum_name(curModel_).data(),
                   curtorq, _torq);
    } else {
        LOG::info("TestModule", "test MIT_TT, %s torq is %f, around %f", magic_enum::enum_name(curModel_).data(),
                  curtorq, _torq);
    }
}

void TestDM::testMitVdesPdes(float _pos, float _vel, float _kp, float _kd)
{
    motor_->setMITKp(_kp);
    motor_->setMITKd(_kd);
    motor_->cmdPosVel(_pos, _vel);
    vTaskDelay(5000);
    float curPos = motor_->data().ang;
    if (fabsf(curPos - _pos) > 0.1) {
        LOG::error("TestModule", "test MIT_VDESPDES, %s pos is %f, beyond %f", magic_enum::enum_name(curModel_).data(),
                   curPos, _pos);
    } else {
        LOG::info("TestModule", "test MIT_VDESPDES, %s pos is %f, around %f", magic_enum::enum_name(curModel_).data(),
                  curPos, _pos);
    }
}

void TestDM::testMITVdes(float _vel, float _kd)
{
    motor_->setMITKd(_kd);
    motor_->cmdVel(_vel);
    vTaskDelay(300);
    float curVel = motor_->data().spdRadps;
    if (fabsf(curVel - _vel) > 0.1) {
        LOG::error("TestModule", "test MIT_VDES, %s vel is %f, beyond %f", magic_enum::enum_name(curModel_).data(),
                   curVel, _vel);
    } else {
        LOG::info("TestModule", "test MIT_VDES, %s vel is %f, around %f", magic_enum::enum_name(curModel_).data(),
                  curVel, _vel);
    }
}

void TestDM::testPdesVdes(float _pos, float _vel)
{
    motor_->cmdPosVel(_pos, _vel);
    vTaskDelay(5000);
    float curPos = motor_->data().ang;
    if (fabsf(curPos - _pos) > 0.1) {
        LOG::error("TestModule", "test PDESVDES, %s pos is %f, beyond %f", magic_enum::enum_name(curModel_).data(),
                   curPos, _pos);
    } else {
        LOG::info("TestModule", "test PDESVDES, %s pos is %f, around %f", magic_enum::enum_name(curModel_).data(),
                  curPos, _pos);
    }
}

void TestDM::testVdes(float _vel)
{
    motor_->cmdVel(_vel);
    vTaskDelay(100);
    float curVel = motor_->data().spdRadps;
    if (fabsf(curVel - _vel) > 0.1) {
        LOG::error("TestModule", "test VDES, %s vel is %f, beyond %f", magic_enum::enum_name(curModel_).data(), curVel,
                   _vel);
    } else {
        LOG::info("TestModule", "test VDES, %s vel is %f, around %f", magic_enum::enum_name(curModel_).data(), curVel,
                  _vel);
    }
}

void TestDM::testEmit(float _pos, float _vel, float _torq)
{
    motor_->cmdMIT(_pos, _vel, _torq);
    vTaskDelay(5000);
    float curPos = motor_->data().ang;
    if (fabsf(curPos - _pos) > 0.1) {
        LOG::error("TestModule", "test EMIT, %s pos is %f, beyond %f", magic_enum::enum_name(curModel_).data(), curPos,
                   _pos);
    } else {
        LOG::info("TestModule", "test EMIT, %s pos is %f, around %f", magic_enum::enum_name(curModel_).data(), curPos,
                  _pos);
    }
}

void TestDM::switchdmWorkMode(PINYMOTOR::WorkMode_e _mode)
{
    motor_->switchCtrlMode(_mode);
    regDM.regId = DMMOTOR::RegId_e::DM_REG_CTRL_MODE;
    if (_mode == WorkMode_e::MIT_TT || _mode == WorkMode_e::MIT_VDESPDES || _mode == WorkMode_e::MIT_VDES) {
        regDM.dat[0] = 0x01;
        motor_->writeOneReg(regDM.regId, regDM.dat);
    } else if (_mode == WorkMode_e::PDESVDES) {
        regDM.dat[0] = 0x02;
        motor_->writeOneReg(regDM.regId, regDM.dat);
    } else if (_mode == WorkMode_e::VDES) {
        regDM.dat[0] = 0x03;
        motor_->writeOneReg(regDM.regId, regDM.dat);
    } else if (_mode == WorkMode_e::EMIT) {
        regDM.dat[0] = 0x04;
        motor_->writeOneReg(regDM.regId, regDM.dat);
    }
    motor_->readOneReg(regDM.regId);
    LOG::info("TestModule", "switch work mode to %s, reg read dat: %02X", magic_enum::enum_name(_mode).data(),
              regDM.dat[0]);
}
