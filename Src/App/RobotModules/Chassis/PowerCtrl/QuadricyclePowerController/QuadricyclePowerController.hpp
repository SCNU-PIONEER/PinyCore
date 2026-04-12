#pragma once
#include "PowerController.hpp"
#include "IMotor.hpp"

class QuadricyclePowerController final : public PowerController {
public:
    QuadricyclePowerController() = default;
    struct PowerCtrlMsg_s {
        float powerLimit;
        const PINYMOTOR::IMotor *const (&motor)[4];
        const float (&cmdTorq)[4];
        const float (&cmdVel)[4];

        PowerCtrlMsg_s(float _powerLimit, const PINYMOTOR::IMotor *const (&_motor)[4], const float (&_cmdTorq)[4],
                       const float (&_cmdVel)[4])
                : powerLimit(_powerLimit), motor(_motor), cmdTorq(_cmdTorq), cmdVel(_cmdVel)
        {
        }
    };
    void powerCtrl(float (&_setTorq)[4], const PowerCtrlMsg_s &_msg);

private:
    static constexpr float VEL_THESHOLD = 200.f;

    MotorPowerModel_s::ModelParam_s LaunchMotion = { .K0 = 0.001f,
                                                     .MLC = 0.001f,
                                                     .ESR = 0.001f,
                                                     .LeakagePower = 2.5f / 4.f };
    MotorPowerModel_s::ModelParam_s UniformMotion = { .K0 = 0.001f,
                                                      .MLC = 0.001f,
                                                      .ESR = 0.001f,
                                                      .LeakagePower = 2.5f / 4.f };
    MotorPowerModel_s wheelModel_{ LaunchMotion };
    RLS<PowerController::FIT_RANK> wheelRLS_{ 0.99999999f, Matrix<PowerController::FIT_RANK, 1>({
                                                                   { UniformMotion.K0 },  //
                                                                   { UniformMotion.MLC }, //
                                                                   { UniformMotion.ESR }  //
                                                           }) };
    float powerLimitRatio_ = 0.f;

    float motorVel_[4]{};
    float motorTorq_[4]{};

    void updateRLS();
};
