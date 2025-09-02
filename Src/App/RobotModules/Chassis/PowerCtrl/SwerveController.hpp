#pragma once
#include "PowerController.hpp"

class SwerveController : public PowerController {
public:
    SwerveController(ChassisType_e _chassisType);

    void cmdPowerCalc(const float *_motorSpeed, PINYMOTOR::IMotor **_motor,
                      const float *_cmd) final;

    void relPowerCalc(PINYMOTOR::IMotor **_motor) final;

    void currentCalc(PINYMOTOR::IMotor **_motor) final;

    void rlsUpdate(PINYMOTOR::IMotor **_motor) final;

    std::vector<float> powerCtrl(const float *_motorSpeed,
                                 PINYMOTOR::IMotor **_motor,
                                 const float *_cmd) final;

private:
    float steerPowerRatio_ = 0.0f;
    float wheelPowerRatio_ = 0.0f;

    float steerCmdPower_ = 0.0f;
    float wheelCmdPower_ = 0.0f;
    float steerRelPower_ = 0.0f;
    float wheelRelPower_ = 0.0f;

    MotorParam_s M3508 = { .KN = 0.0f,
                           .MLC = 0.0f,
                           .ESR = 0.0f,
                           .LeakagePower = 0.0f };
    MotorParam_s GM6020 = { .KN = 0.0f,
                            .MLC = 0.0f,
                            .ESR = 0.0f,
                            .LeakagePower = 0.0f };

    RLS<3> wheelRLS_ = RLS<3>(0.99999f);
    RLS<3> steerRLS_ = RLS<3>(0.99999f);
};
