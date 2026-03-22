
#pragma once
#include "PowerController.hpp"

class SwerveController : public PowerController {
public:
    SwerveController(ChassisType_e _chassisType, CAP *_cap);

    std::vector<float> powerCtrl(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[8], const float *_cmd,
                                 RefereeMsg_s _msg);

private:
    static constexpr float VEL_THESHOLD = 10.f;
    float steerPowerRatio_ = 0.0f;
    float wheelPowerRatio_ = 0.0f;
    float wheelRelPower_ = 0.0f;
    float steerRelPower_ = 0.0f;
    float wheelCmdPower_ = 0.0f;
    float steerCmdPower_ = 0.0f;

    PowerModel_s Wheel{};
    PowerModel_s Steer{};
    PowerModel_s::ModelParam_s WheelLaunchMotion = { .K0 = 0.0409562f,
                                                     .MLC = 9.99004e-05f,
                                                     .ESR = 1.78819f,
                                                     .LeakagePower = 3.f / ((float)motorNum_ / 2.f) };
    PowerModel_s::ModelParam_s WheelUniformMotion = { .K0 = 0.0001f,
                                                      .MLC = 0.0001f,
                                                      .ESR = 0.0001f,
                                                      .LeakagePower = 3.f / ((float)motorNum_ / 2.f) };
    PowerModel_s::ModelParam_s SteerLaunchMotion = { .K0 = 0.388717f,
                                                     .MLC = 0.0222961f,
                                                     .ESR = 12.6557f,
                                                     .LeakagePower = 6.0f / ((float)motorNum_ / 2.f) };
    RLS<PowerModel_s::FIT_RANK> *wheelRLS_;
#if ENABLE_STEER_RLS
    RLS<PowerModel_s::FIT_RANK> *steerRLS_;
#endif
    void cmdPowerCalc(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[8], const float *_cmd);
    void relPowerCalc(PINYMOTOR::IMotor *_motor[8]);
    void torqueCalc(PINYMOTOR::IMotor *_motor[8], const float *_cmd);
    void rlsUpdate(PINYMOTOR::IMotor *_motor[8]);
};
