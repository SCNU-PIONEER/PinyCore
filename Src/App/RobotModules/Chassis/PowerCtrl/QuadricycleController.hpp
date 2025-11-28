#pragma once
#include "PowerController.hpp"

class QuadricycleController : public PowerController {
public:
    QuadricycleController(ChassisType_e _chassisType, CAP *_cap);

    void cmdPowerCalc(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[4], const float *_cmd);
    void relPowerCalc(PINYMOTOR::IMotor *_motor[4]);
    void currentCalc(PINYMOTOR::IMotor *_motor[4], const float *_cmd);
    void rlsUpdate(PINYMOTOR::IMotor *_motor[4]);

    std::vector<float> powerCtrl(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[4], const float *_cmd,
                                 RefereeMsg_s _msg);

private:
    MotorParam_s M3508 = { .KN = 0.0001f, .MLC = 0.0001f, .ESR = 0.0001f, .LeakagePower = 4.7f / (float)motorNum_ };
    RLS<3> wheelRLS_ = RLS<3>(0.999f);
};
