
#pragma once
#include "PowerController.hpp"

class SwerveController : public PowerController {
public:
    SwerveController(ChassisType_e _chassisType, SuperCap *_cap);

    std::vector<float> powerCtrl(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[8], const float *_cmd,
                                 RefereeMsg_s _msg, const float _sVel[4]);

private:
    static constexpr float VEL_THESHOLD = 10.f;
    float steerPowerRatio_ = 0.0f;
    float wheelPowerRatio_ = 0.0f;
    float wheelFitPower_ = 0.0f;
    float steerFitPower_ = 0.0f;
    float wheelCmdPower_ = 0.0f;
    float steerCmdPower_ = 0.0f;

    PowerModel_s Wheel{};
    PowerModel_s Steer{};
    PowerModel_s::ModelParam_s WheelLaunchMotion = { .K0 = 0.0456955f,
                                                     .MLC = 9.59024e-05f,
                                                     .ESR = 3.30419f,
                                                     .LeakagePower = 3.f / ((float)motorNum_ / 2.f) };
    PowerModel_s::ModelParam_s WheelUniformMotion = { .K0 = 0.0353186f,
                                                      .MLC = 5.76333e-05f,
                                                      .ESR = 4.17215f,
                                                      .LeakagePower = 3.f / ((float)motorNum_ / 2.f) };
    PowerModel_s::ModelParam_s SteerLaunchMotion = { .K0 = 0.795305f,
                                                     .MLC = 0.031172f,
                                                     .ESR = 9.54849f,
                                                     .LeakagePower = 4.f / ((float)motorNum_ / 2.f) };
    RLS<PowerModel_s::FIT_RANK> *wheelRLS_;
#if ENABLE_STEER_RLS
    RLS<PowerModel_s::FIT_RANK> *steerRLS_;
#endif
    void cmdPowerCalc(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[8], const float *_cmd, const float _sVel[4]);
    void relPowerCalc(PINYMOTOR::IMotor *_motor[8], const float _sVel[4]);
    void torqueCalc(PINYMOTOR::IMotor *_motor[8], const float *_cmd, const float _sVel[4]);
    void rlsUpdate(PINYMOTOR::IMotor *_motor[8], const float _sVel[4]);
};
