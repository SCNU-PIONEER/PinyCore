#pragma once
#include "PowerController.hpp"

class QuadricycleController : public PowerController {
public:
    QuadricycleController(ChassisType_e _chassisType);

    void cmdPowerCalc(float *_motorSpeed) final;

    void relPowerCalc() final;

    void currentCalc() final;

    void rlsUpdate() final;

    std::vector<float> powerCtrl(float *_motorSpeed) final;

private:
    float powerRatio_ = 0.0f;

    MotorParam_s M3508 = { .k0 = 0.0f,
                           .MLC = 0.0f,
                           .ESR = 0.0f,
                           .LeakagePower = 0.0f };

    RLS<3> wheelRLS_ = RLS<3>(0.99999f);
};
