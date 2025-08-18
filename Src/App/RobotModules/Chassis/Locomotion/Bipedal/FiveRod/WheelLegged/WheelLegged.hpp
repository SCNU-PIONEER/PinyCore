#pragma once

#include "VMC.hpp"
#include "IMotor.hpp"

namespace CHASSIS::BIPEDAL {

class WheelLegged {
    static constexpr VMCConfig_s VMC_CONFIG = { .L1 = 0.15f,
                                                .L2 = 0.25f,
                                                .L3 = 0.25f,
                                                .L4 = 0.15f,
                                                .L5 = 0.12f };

public:
    WheelLegged(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4,
                PINYMOTOR::IMotor *_wheel);

    void update();

    void ctrl();

    const LegState_s &state() const { return legState_; }

private:
    LegState_s legState_;

    float tp_ = 0.f;
    float fd_ = 0.f;
    float tw_ = 0.f;

    VMC vmc_ = VMC(VMC_CONFIG);

    PINYMOTOR::IMotor *hip1_;
    PINYMOTOR::IMotor *hip4_;
    PINYMOTOR::IMotor *wheel_;
};

} // namespace CHASSIS::BIPEDAL
