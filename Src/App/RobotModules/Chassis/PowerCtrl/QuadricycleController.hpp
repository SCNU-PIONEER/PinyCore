#pragma once
#include "PowerController.hpp"

class QuadricycleController : public PowerController {
public:
    QuadricycleController(ChassisType_e chassisType);

    void cmdPowerCalc(float *motorSpeed) override final;

    void relPowerCalc() override final;

    void currentCalc() override final;

    std::vector<float> powerCtrl(float *motorSpeed) override final;

private:
    float powerRatio_ = 0.0f;

    MotorParam_s M3508 = { 0.0f, 0.0f, 0.0f, 0.0f };
};
