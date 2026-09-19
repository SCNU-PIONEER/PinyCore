#include "TestLKMotor.hpp"
#include "MG4005.hpp"
#include "MF9025.hpp"
#include "FreeRTOS.h"

#include "MotorCommonMacros.hpp"
#include "Projdefs.hpp"
#include "Soc.hpp"
#include "StmLog.hpp"
#include "task.h"
#include "magic_enum/magic_enum.hpp"
#include <cmath>
#include <cstdint>

using namespace TEST;
using namespace PINYMOTOR;
using namespace LKMOTOR;

InitConfig_s configLK = {
    .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
    .comType = ComType_e::CAN,
    .motorType = SupportMotor_e::LK,
    .offsetId = 1,
    .txFreq = 300.0f,
};

LKMotorCAN *TestLKMotor::getMotor() { return motorCAN_; }

const char *TestLKMotor::getModelName() { return magic_enum::enum_name(curModel_).data(); }

void TestLKMotor::logError(const char *_test, const char *_detail)
{
    LOG::error("TestLKMotor", "[%s] %s: %s", getModelName(), _test, _detail);
}

void TestLKMotor::logInfo(const char *_test, const char *_detail)
{
    LOG::info("TestLKMotor", "[%s] %s: %s", getModelName(), _test, _detail);
}

void TestLKMotor::addMotor(LkMotorModel_e _model, uint8_t _id)
{
    configLK.offsetId = _id;

    switch (_model) {
    case LkMotorModel_e::MG4005:
        motorCAN_ = new MG4005("mg4005", configLK, WorkMode_e::PDESVDES);
        curModel_ = LkMotorModel_e::MG4005;
        break;
    case LkMotorModel_e::MF9025:
        motorCAN_ = new MF9025("mf9025", configLK, WorkMode_e::PDESVDES);
        curModel_ = LkMotorModel_e::MF9025;
        break;
    }
    LOG::info("TestModule", "LK motor added: %s (id: %d)", getModelName(), _id);
}

void TestLKMotor::test()
{
    LOG::info("TestLKMotor", "========== LK Motor Test Start ==========");
    vTaskDelay(1000);

    testON();
    vTaskDelay(100);

    // //状态读取
    // testReadState1();
    // vTaskDelay(50);
    // testReadState2();
    // vTaskDelay(50);
    // testReadEncoder();
    // vTaskDelay(50);

    // //速度
    // LOG::info("TestLKMotor", "--- Speed Control Test ---");
    // switchWorkMode(WorkMode_e::VDES);
    // testVdes(2.f); //rad/s
    // vTaskDelay(10000);

    // //多圈位置速度
    // LOG::info("TestLKMotor", "--- Multi-Position Control Test ---");
    // switchWorkMode(WorkMode_e::MULTI_PDESVDES);
    // testMultiPdesVdes(6.1f, 6.f); //rad, rad/s
    // vTaskDelay(40000);

    // //单圈位置速度
    // LOG::info("TestLKMotor", "--- Single-Position Control Test ---");
    // switchWorkMode(WorkMode_e::SINGLE_PDESVDES);
    // testSinglePdesVdes(1.57f, 2.0f);
    // vTaskDelay(10000);

    // //增量位置速度
    // LOG::info("TestLKMotor", "--- Incremental Position Control Test ---");
    // switchWorkMode(WorkMode_e::INC_PDESVDES);
    // testIncPdesVdes(0.01f, 4.0f);
    // vTaskDelay(30000);

    //力矩控制(空载电流变化较大)
    LOG::info("TestLKMotor", "--- Torque Control Test ---");
    switchWorkMode(WorkMode_e::MIT_TT);
    testMitTt(0.012f); //对应输出轴力矩0.012*减速比
    vTaskDelay(5000);

    // //编码器
    // LOG::info("TestLKMotor", "--- Encoder Test ---");
    // testReadMultiPos();
    // vTaskDelay(50);
    // testReadSinglePos();
    // vTaskDelay(50);

    // //设置当前位置为零点
    // testSetZeroPos();

    testStop();
    vTaskDelay(100);
    // testOFF();
    // vTaskDelay(2000);

    LOG::info("TestLKMotor", "========== LK Motor Test End ==========");
    vTaskSuspend(getTaskHandler());
}

void TestLKMotor::testON()
{
    if (!motorCAN_) {
        logError("testON", "Motor not initialized");
        return;
    }

    motorCAN_->cmd(MotorCmdType_e::ON);
    vTaskDelay(100);
}

void TestLKMotor::testOFF()
{
    if (!motorCAN_) {
        logError("testOFF", "Motor not initialized");
        return;
    }

    motorCAN_->cmd(MotorCmdType_e::OFF);
    vTaskDelay(100);

    logInfo("testOFF", "Motor disabled successfully");
}

void TestLKMotor::testStop()
{
    if (!motorCAN_) {
        logError("testStop", "Motor not initialized");
        return;
    }

    motorCAN_->stop();
    vTaskDelay(100);

    float curVel = motorCAN_->data().spdRadps;
    if (fabsf(curVel) < 0.5f) {
        logInfo("testStop", "Motor stopped successfully");
    } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "Motor still moving, vel=%.2f", curVel);
        logError("testStop", buf);
    }
}

void TestLKMotor::testVdes(float _vel)
{
    if (!motorCAN_)
        return;
    //4005 :Rated Torque = 1 N*m;Max Torque = 2.5N*m
    motorCAN_->cmdMIT(0.f, _vel, 1.f);
    vTaskDelay(500);

    float curVel = motorCAN_->data().spdRadps;
    float error = fabsf(curVel - _vel);

    char buf[64];
    if (error > 1.0f) {
        snprintf(buf, sizeof(buf), "vel=%.2f, target=%.2f, error=%.2f", curVel, _vel, error);
        logError("testVdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "vel=%.2f, target=%.2f", curVel, _vel);
        logInfo("testVdes", buf);
    }
}

void TestLKMotor::testPdesVdes(float _pos, float _vel)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdPosVel(_pos, _vel);
    vTaskDelay(3000);

    float curPos = motorCAN_->data().ang;
    float error = fabsf(curPos - _pos);

    char buf[64];
    if (error > 0.2f) {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f, error=%.2f", curPos, _pos, error);
        logError("testPdesVdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f", curPos, _pos);
        logInfo("testPdesVdes", buf);
    }
}

void TestLKMotor::testSinglePdes(float _pos)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdPos(_pos);
    vTaskDelay(3000);

    float curPos = motorCAN_->data().ang;
    float error = fabsf(curPos - _pos);

    char buf[64];
    if (error > 0.2f) {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f, error=%.2f", curPos, _pos, error);
        logError("testSinglePdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f", curPos, _pos);
        logInfo("testSinglePdes", buf);
    }
}

void TestLKMotor::testSinglePdesVdes(float _pos, float _vel)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdPosVel(_pos, _vel);
    vTaskDelay(3000);

    float curPos = motorCAN_->data().ang;
    float error = fabsf(curPos - _pos);

    char buf[64];
    if (error > 0.2f) {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f, error=%.2f", curPos, _pos, error);
        logError("testSinglePdesVdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f", curPos, _pos);
        logInfo("testSinglePdesVdes", buf);
    }
}

void TestLKMotor::testMultiPdes(float _pos)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdPos(_pos);
    vTaskDelay(3000);

    float curPos = motorCAN_->data().multipCirAng;
    float error = fabsf(curPos - _pos);

    char buf[64];
    if (error > 0.2f) {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f, error=%.2f", curPos, _pos, error);
        logError("testMultiPdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f", curPos, _pos);
        logInfo("testMultiPdes", buf);
    }
}

void TestLKMotor::testMultiPdesVdes(float _pos, float _vel)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdPosVel(_pos, _vel);
    vTaskDelay(30000);

    float curPos = motorCAN_->data().multipCirAng;
    float error = fabsf(curPos - _pos);

    char buf[64];
    if (error > 0.2f) {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f, error=%.2f", curPos, _pos, error);
        logError("testMultiPdesVdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "pos=%.2f, target=%.2f", curPos, _pos);
        logInfo("testMultiPdesVdes", buf);
    }
}

void TestLKMotor::testIncPdes(float _pos)
{
    if (!motorCAN_)
        return;

    float startPos = motorCAN_->data().ang;
    motorCAN_->cmdPos(_pos);
    vTaskDelay(3000);

    float curPos = motorCAN_->data().ang;
    float expectedPos = startPos + _pos;
    float error = fabsf(curPos - expectedPos);

    char buf[64];
    if (error > 0.2f) {
        snprintf(buf, sizeof(buf), "pos=%.2f, expected=%.2f, error=%.2f", curPos, expectedPos, error);
        logError("testIncPdes", buf);
    } else {
        snprintf(buf, sizeof(buf), "pos=%.2f, expected=%.2f", curPos, expectedPos);
        logInfo("testIncPdes", buf);
    }
}

void TestLKMotor::testIncPdesVdes(float _pos, float _vel)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdPosVel(_pos, _vel);
    vTaskDelay(30000);
}

void TestLKMotor::testMitTt(float _torq)
{
    if (!motorCAN_)
        return;

    motorCAN_->cmdTorq(_torq);
    vTaskDelay(10000);

    float cur = motorCAN_->data().torq;
    float error = fabsf(cur - (_torq));

    char buf[64];
    if (error > 0.005f) {
        snprintf(buf, sizeof(buf), "_torq=%.3f, target=%.3f, error=%.3f", cur, _torq, error);
        logError("testMitTt", buf);
    } else {
        snprintf(buf, sizeof(buf), "_torq=%.3f, target=%.3f", cur, _torq);
        logInfo("testMitTt", buf);
    }
}

void TestLKMotor::testReadState1()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readState1andError();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf), "temp=%.1f, curr=%.2f", motorCAN_->data().tempture, motorCAN_->data().curr);
        logInfo("testReadState1", buf);
    } else {
        logError("testReadState1", "Failed to read state1");
    }
}

void TestLKMotor::testReadState2()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readState2();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        char buf[80];
        snprintf(buf, sizeof(buf), "temp=%.1f, curr=%.2f, spd=%.2f, ang=%.2f", motorCAN_->data().tempture,
                 motorCAN_->data().curr, motorCAN_->data().spdRadps, motorCAN_->data().ang);
        logInfo("testReadState2", buf);
    } else {
        logError("testReadState2", "Failed to read state2");
    }
}

void TestLKMotor::testReadState3()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readState3();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        logInfo("testReadState3", "Read state3 success");
    } else {
        logError("testReadState3", "Failed to read state3");
    }
}

void TestLKMotor::testReadEncoder()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readEncoder();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        logInfo("testReadEncoder", "Read encoder success");
    } else {
        logError("testReadEncoder", "Failed to read encoder");
    }
}

void TestLKMotor::testReadCtrlCmd(ParamID_e _id)
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readCtrlCmd(_id);
    }

    vTaskDelay(50);

    char buf[64];
    snprintf(buf, sizeof(buf), "ParamID=0x%02X, result=%d", static_cast<uint8_t>(_id), result);
    if (result == STM_OK) {
        logInfo("testReadCtrlCmd", buf);
    } else {
        logError("testReadCtrlCmd", buf);
    }
}

void TestLKMotor::testWriteCtrlCmd(ParamID_e _id, std::array<uint8_t, 6> _data)
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->writeCtrlCmd(_id, _data);
    }

    vTaskDelay(50);

    char buf[64];
    snprintf(buf, sizeof(buf), "ParamID=0x%02X, result=%d", static_cast<uint8_t>(_id), result);
    if (result == STM_OK) {
        logInfo("testWriteCtrlCmd", buf);
    } else {
        logError("testWriteCtrlCmd", buf);
    }
}
//测试设置零点
void TestLKMotor::testSetZeroPos()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->setZeroPos(); //断电重启生效
    }
    vTaskDelay(100);

    if (result == STM_OK) {
        logInfo("testSetZeroPos", "Zero position set successfully");
    } else {
        logError("testSetZeroPos", "Failed to set zero position");
    }
}

void TestLKMotor::testReadMultiPos()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readMultiPos();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf), "multiPos=%.2f rad", motorCAN_->data().multipCirAng);
        logInfo("testReadMultiPos", buf);
    } else {
        logError("testReadMultiPos", "Failed to read multi position");
    }
}

void TestLKMotor::testReadSinglePos()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->readSinglePos();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        char buf[64];
        snprintf(buf, sizeof(buf), "singlePos=%.2f rad", motorCAN_->data().ang);
        logInfo("testReadSinglePos", buf);
    } else {
        logError("testReadSinglePos", "Failed to read single position");
    }
}

void TestLKMotor::testClearPosCircle()
{
    MotorTypeDef_e result = STM_OK;

    if (motorCAN_) {
        result = motorCAN_->clearPosCircle();
    }

    vTaskDelay(50);

    if (result == STM_OK) {
        logInfo("testClearPosCircle", "Position circle cleared");
    } else {
        logError("testClearPosCircle", "Failed to clear position circle");
    }
}

void TestLKMotor::switchWorkMode(WorkMode_e _mode)
{
    if (motorCAN_) {
        motorCAN_->switchCtrlMode(_mode);
        LOG::info("TestLKMotor", "Switch work mode to %s", getModelName());
    }
}