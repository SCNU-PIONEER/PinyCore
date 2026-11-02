#pragma once

#include "TestModule.hpp"
#include <cstdint>
#include "DJIMotor.hpp"

namespace TEST {

enum class DJIMotorModel_e : uint8_t { GM6020, M3508 };

class TestDJI : public TestModule {
public:
    void rebuildMotor(DJIMotorModel_e _model, uint8_t _id);

    void test();
    void testON();
    void testOFF();
    void testRot(float _elec);

private:
    PINYMOTOR::DJIMOTOR::DJIMotor *motor_;
    PINYMOTOR::DJIMOTOR::WorkMode_e currentWorkMode_;
    DJIMotorModel_e curModel_;
};

} // namespace TEST