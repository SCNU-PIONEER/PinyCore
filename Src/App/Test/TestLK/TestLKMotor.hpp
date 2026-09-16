#pragma once

#include "TestModule.hpp"
#include "LKMotorCan.hpp"
#include <cstdint>

namespace TEST {

enum class LkMotorModel_e : uint8_t { MG4005, MF9025 };

class TestLKMotor : public TestModule {
public:
    void addMotor(LkMotorModel_e _model, uint8_t _id);

    void test();

    //  直接发送命令 (调用一次发送一次)
    void testON();
    void testOFF();
    void testStop();
    void testReadState1();
    void testReadState2();
    void testReadState3();
    void testReadEncoder();
    void testReadMultiPos();
    void testReadSinglePos();
    void testClearPosCircle();
    void testReadCtrlCmd(PINYMOTOR::LKMOTOR::ParamID_e _id);
    void testWriteCtrlCmd(PINYMOTOR::LKMOTOR::ParamID_e _id, std::array<uint8_t, 6> _data);
    void testSetZeroPos();

    // 统一发送命令 (设置cmd_，由ctrl()周期性发送)
    void testVdes(float _vel);
    void testMitTt(float _torq);
    void testPdesVdes(float _pos, float _vel);
    void testSinglePdes(float _pos);
    void testSinglePdesVdes(float _pos, float _vel);
    void testMultiPdes(float _pos);
    void testMultiPdesVdes(float _pos, float _vel);
    void testIncPdes(float _pos);
    void testIncPdesVdes(float _pos, float _vel);

    void switchWorkMode(PINYMOTOR::LKMOTOR::WorkMode_e _mode);

private:
    PINYMOTOR::LKMOTOR::LKMotorCAN *motorCAN_ = nullptr;
    PINYMOTOR::LKMOTOR::WorkMode_e currentWorkMode_;
    LkMotorModel_e curModel_;

    PINYMOTOR::LKMOTOR::LKMotorCAN *getMotor();
    const char *getModelName();
    void logError(const char *_test, const char *_detail);
    void logInfo(const char *_test, const char *_detail);
};

} // namespace TEST