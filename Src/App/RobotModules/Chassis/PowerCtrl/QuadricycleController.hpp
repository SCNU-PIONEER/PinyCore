#pragma once
#include "PowerController.hpp"

class QuadricycleController : public PowerController {
public:
    QuadricycleController(ChassisType_e _chassisType);

    void cmdPowerCalc(const float *_motorSpeed, PINYMOTOR::IMotor **_motor,
                      const float *_cmd) final;
    void relPowerCalc(PINYMOTOR::IMotor **_motor) final;
    void currentCalc(PINYMOTOR::IMotor **_motor, const float *_cmd) final;
    void rlsUpdate(PINYMOTOR::IMotor **_motor) final;

    std::vector<float> powerCtrl(const float *_motorSpeed,
                                 PINYMOTOR::IMotor **_motor, const float *_cmd,
                                 RefereeMsg_s _msg) final;

private:
    MotorParam_s M3508 = { .KN = 0.0001f,
                           .MLC = 0.0001f,
                           .ESR = 0.0001f,
                           .LeakagePower = 4.7f / (float)motorNum_ };
    RLS<3> wheelRLS_ = RLS<3>(0.999f);
};
