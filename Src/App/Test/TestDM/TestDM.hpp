#pragma once

#include "TestModule.hpp"
#include <cstdint>
#include "Projdefs.hpp"
#include "DMMotor.hpp"

namespace TEST {

class TestDM : public TestModule {
public:
    void rebuildMotor(DmMotorModel_e _model, uint8_t _id);

    void test();
    void testON();
    void testOFF();
    void testMitTt(float _torq);
    void testMitVdesPdes(float _pos, float _vel, float _kp, float _kd);
    void testMITVdes(float _vel, float _kd);
    void testPdesVdes(float _pos, float _vel);
    void testVdes(float _vel);
    void testEmit(float _pos, float _vel, float _torq);
    void switchdmWorkMode(PINYMOTOR::DMMOTOR::WorkMode_e _mode);

private:
    PINYMOTOR::DMMOTOR::DMMotor *motor_;
    PINYMOTOR::DMMOTOR::WorkMode_e currentWorkMode_;
    DmMotorModel_e curModel_;
};

} // namespace TEST