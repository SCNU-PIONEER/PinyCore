#include "TestDM.hpp"
#include "DM4310.hpp"
#include "DM3507.hpp"
#include "DM3519.hpp"
#include "DM4340.hpp"
#include "DM6006.hpp"
#include "DM6248.hpp"
#include "DM8009.hpp"
#include "DM10010L.hpp"
#include "Bsp_can.hpp"
#include "FreeRTOS.h"
#include "Projdefs.hpp"
#include "sdkconfig.h"
#include "Soc.hpp"
#include "StmLog.hpp"
#include "task.h"
#include "magic_enum/magic_enum.hpp"
#include <cmath>
#include <cstdint>

using namespace TEST;
using namespace PINYMOTOR;
using namespace PINYMOTOR::DMMOTOR;

InitConfig_s configDM = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
    .comType = ComType_e::CAN,
    .motorType = SupportMotor_e::DA_MIAO,
    .offsetId = 2,
    .txFreq = 500.0f,
};

Reg_s regDM = {
    .regId = RegId_e::DM_REG_CTRL_MODE,
    .dat = { 0x00, 0x00, 0x00, 0x00 },
    .isWrite = false,
    .isRead = false,
    .isStorage = false,
};

RegValue_u regValueDM = {
    .dat = { 0x00, 0x00, 0x00, 0x00 },
};

void TestDM::test()
{
    testON();
    // switchdmWorkMode(WorkMode_e::VDES, false);
    // testVdes(3.f);
    // vTaskDelay(3000);

    // switchdmWorkMode(WorkMode_e::PDESVDES, false);
    // testPdesVdes(4.f, 4.f);
    // vTaskDelay(6000);

    // switchdmWorkMode(WorkMode_e::MIT_TT, false);
    // testMitTt(0.5f);
    // vTaskDelay(3000);

    // switchdmWorkMode(WorkMode_e::MIT_VDES, false);
    // testMITVdes(2.f, 1.f);
    // vTaskDelay(3000);

    // switchdmWorkMode(WorkMode_e::MIT_VDESPDES, false);
    // testMitVdesPdes(2.f, 0.f, 5.f, 1.f);

    // switchdmWorkMode(WorkMode_e::EMIT, false);
    // testEmit(1.f, 2.f, 1.f);
    // vTaskDelay(3000);

    testOFF();
    vTaskSuspend(getTaskHandler());
}

void TestDM::rebuildMotor(DmMotorModel_e _model, uint8_t _id)
{
    curModel_ = _model;
    configDM.offsetId = _id;
    switch (_model) {
#if MOTOR_DM4310
    case DmMotorModel_e::DM4310:
        motor_ = new DM4310("dm4310", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM3507
    case DmMotorModel_e::DM3507:
        motor_ = new DM3507("dm3507", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM3519
    case DmMotorModel_e::DM3519:
        motor_ = new DM3519("dm3519", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM4340
    case DmMotorModel_e::DM4340:
        motor_ = new DM4340("dm4340", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM6006
    case DmMotorModel_e::DM6006:
        motor_ = new DM6006("dm6006", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM6248
    case DmMotorModel_e::DM6248:
        motor_ = new DM6248("dm6248", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM8009
    case DmMotorModel_e::DM8009:
        motor_ = new DM8009("dm8009", configDM, WorkMode_e::EMIT);
        break;
#endif
#if MOTOR_DM10010L
    case DmMotorModel_e::DM10010L:
        motor_ = new DM10010L("dm10010l", configDM, WorkMode_e::EMIT);
        break;
#endif
    default:
        LOG::error("TestModule", "DM motor model %s is not enabled in the configuration",
                   magic_enum::enum_name(_model).data());
        return;
    }
    motor_->registerReg(&regDM, &regValueDM);
}

void TestDM::testON()
{
    motor_->cmd(MotorCmdType_e::ON);
    vTaskDelay(20);
    ErrorCode_e errorCode2 = motor_->getErrorcode();
    if (errorCode2 != ErrorCode_e::MOTOR_ENABLE) {
        LOG::error("TestModule", "test on, %s error code: %s,", magic_enum::enum_name(curModel_).data(),
                   magic_enum::enum_name(errorCode2).data());
    } else {
        LOG::info("TestModule", " %s is enabled", magic_enum::enum_name(curModel_).data());
    }
}

void TestDM::testOFF()
{
    motor_->cmd(MotorCmdType_e::OFF);
    vTaskDelay(20);
    ErrorCode_e errorCode2 = motor_->getErrorcode();
    if (errorCode2 != ErrorCode_e::MOTOR_DISABLE) {
        LOG::error("TestModule", "test off, %s error code: %s,", magic_enum::enum_name(curModel_).data(),
                   magic_enum::enum_name(errorCode2).data());
    } else {
        LOG::info("TestModule", " DMMotor is disabled");
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

void TestDM::switchdmWorkMode(WorkMode_e _mode, bool _store)
{
    testOFF();
    motor_->switchCtrlMode(_mode);
    const uint8_t code = regModeCode(_mode);
    const uint16_t id = motor_->canId();
    const uint8_t idL = static_cast<uint8_t>(id);
    const uint8_t idH = static_cast<uint8_t>(id >> 8u);
    uint8_t writeData[8] = { idL, idH, 0x55, static_cast<uint8_t>(RegId_e::DM_REG_CTRL_MODE), code, 0, 0, 0 };
    sendDmFrame(0x7FF, writeData);
    vTaskDelay(20);
    if (_store) {
        uint8_t saveData[8] = { idL, idH, 0xAA, 0x01, 0, 0, 0, 0 };
        sendDmFrame(0x7FF, saveData);
        vTaskDelay(20);
    }
    testON();
    LOG::info("TestModule", "switch work mode to %s", magic_enum::enum_name(_mode).data());
}

uint8_t TestDM::regModeCode(WorkMode_e _mode)
{
    switch (_mode) {
    case WorkMode_e::EMIT:
        return 4;
    case WorkMode_e::PDESVDES:
        return 2;
    case WorkMode_e::VDES:
        return 3;
    default:
        return 1;
    }
}

void TestDM::sendDmFrame(uint16_t _id, uint8_t *_dat)
{
    canHandle *pHandle = reinterpret_cast<canHandle *>(configDM.pComHandle);
    const uint32_t start = HAL_GetTick();
#if defined(SOC_FDCAN)
    while (HAL_FDCAN_GetTxFifoFreeLevel(pHandle) == 0 && (HAL_GetTick() - start) < 5) {
    }
#elif defined(SOC_CAN)
    while (HAL_CAN_GetTxMailboxesFreeLevel(pHandle) == 0 && (HAL_GetTick() - start) < 5) {
    }
#endif
    Can::instance().transmitData(pHandle, _id, _dat, 8);
}
