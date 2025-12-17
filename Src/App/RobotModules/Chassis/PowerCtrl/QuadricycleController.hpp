#pragma once
#include "PowerController.hpp"

class QuadricycleController : public PowerController {
public:
    QuadricycleController(ChassisType_e _chassisType, CAP *_cap);

    std::vector<float> powerCtrl(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[4], const float *_cmd,
                                 const RefereeMsg_s &_msg);

private:
    static constexpr float VEL_THESHOLD = 200.f;
    PowerModel_s Wheel{};
    PowerModel_s::ModelParam_s LaunchMotion = { .K0 = 0.001f,
                                                .MLC = 0.001f,
                                                .ESR = 0.001f,
                                                .LeakagePower = 2.5f / (float)motorNum_ };
    PowerModel_s::ModelParam_s UniformMotion = { .K0 = 0.001f,
                                                 .MLC = 0.001f,
                                                 .ESR = 0.001f,
                                                 .LeakagePower = 2.5f / (float)motorNum_ };
    RLS<PowerModel_s::FIT_RANK> *wheelRLS_;
    void cmdPowerCalc(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[4], const float *_cmd);
    void relPowerCalc(PINYMOTOR::IMotor *_motor[4]);
    void torqueCalc(PINYMOTOR::IMotor *_motor[4], const float *_cmd);
    void rlsUpdate(PINYMOTOR::IMotor *_motor[4]);
};
