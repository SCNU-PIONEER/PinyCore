#pragma once

#include "VMC.hpp"
#include "IMotor.hpp"
#include "INS.hpp"
#include "PidBasic.hpp"

namespace CHASSIS::BIPEDAL {

class WheelLegged {
public:
    static constexpr float GRAVITY = 9.7833f;

    static constexpr float MASS_WHEEL = 0.880f + 0.500f;

    static constexpr float FN_THRESHOLD = MASS_WHEEL * GRAVITY;

    static constexpr float MAX_L = 0.3f;

    static constexpr VMCConfig_s VMC_CONFIG = { .L1 = 0.15f,
                                                .L2 = 0.25f,
                                                .L3 = 0.25f,
                                                .L4 = 0.15f,
                                                .L5 = 0.12f };

    WheelLegged(PINYMOTOR::IMotor *_hip1, PINYMOTOR::IMotor *_hip4,
                PINYMOTOR::IMotor *_wheel);

    void stateUpdate(float _dt);

    bool detectFloating(const INS_SYS::INSData_s &_ins);
    bool detectFloating(float _az);

    void setL(float _l);

    void setFd(float _fd);
    void setTp(float _tp);
    void setTw(float _tw);

    void ctrl();

    const LegState_s &state() const { return legState_; }

    bool isFloating = false;

private:
    LegState_s legState_;

    float tp_ = 0.f;
    float fd_ = 0.f;
    float tw_ = 0.f;

    float fn_ = 0.f;

    VMC vmc_ = VMC(VMC_CONFIG);

    float setL_;

    PositonalPid LengthAngPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };
    PositonalPid LengthVelPid_{ 0, 0, 0, 0.001f, 0, 0, 0 };

    PINYMOTOR::IMotor *hip1_;
    PINYMOTOR::IMotor *hip4_;
    PINYMOTOR::IMotor *wheel_;

    float minL(float _phi);
};

} // namespace CHASSIS::BIPEDAL
