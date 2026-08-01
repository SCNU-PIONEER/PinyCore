#pragma once

#include "TestModule.hpp"
#include <cstdint>
#include "Projdefs.hpp"
#include "MTMotor.hpp"

namespace TEST {

enum class MTMotorModel_e : uint8_t {
    RMD_X2_7,
    RMD_X4_10,
    RMD_X4_36,
};

class TestMT : public TestModule {
public:
    void buildMotor(MTMotorModel_e _model, uint8_t _id);

    void test();
    void testON();
    void testOFF();
    void testTorq(float _torq);
    void testSpeed(float _vel);
    void testAbsPos(float _velLimit, float _pos);
    void testSinglePos(float _velLimit, float _pos);
    void testIncPos(float _pos);
    void testForcePos(float _torqLimit, float _velLimit, float _pos);
    void switchMTWorkMode(PINYMOTOR::MTMOTOR::WorkMode_e _mode);

private:
    PINYMOTOR::MTMOTOR::MTMotor *motor_;
};

} // namespace TEST