
#pragma once
#include "PowerController.hpp"
#include "IMotor.hpp"

class SwerveController : public PowerController {
public:
    SwerveController() = default;
    struct PowerCtrlMsg_s {
        float powerLimit;
        const PINYMOTOR::IMotor *const (&wheelMotor)[4];
        const float (&cmdWheelTorq)[4];
        const float (&cmdWheelVel)[4];
        const PINYMOTOR::IMotor *const (&steerMotor)[4];
        const float (&cmdSteerTorq)[4];
        const float (&cmdSteerVel)[4];

        PowerCtrlMsg_s(float _powerLimit, const PINYMOTOR::IMotor *const (&_wheelMotor)[4],
                       const float (&_cmdWheelTorq)[4], const float (&_cmdWheelVel)[4],
                       const PINYMOTOR::IMotor *const (&_steerMotor)[4], const float (&_cmdSteerTorq)[4],
                       const float (&_cmdSteerVel)[4])
                : powerLimit(_powerLimit)
                , wheelMotor(_wheelMotor)
                , cmdWheelTorq(_cmdWheelTorq)
                , cmdWheelVel(_cmdWheelVel)
                , steerMotor(_steerMotor)
                , cmdSteerTorq(_cmdSteerTorq)
                , cmdSteerVel(_cmdSteerVel)
        {
        }
    };
    void powerCtrl(float (&_setWheelTorq)[4], float (&_setSteerTorq)[4], const PowerCtrlMsg_s &_msg);

private:
    static constexpr float VEL_THESHOLD = 10.f;

    MotorPowerModel_s::ModelParam_s wheelLaunchMotion = { .K0 = 0.0001f,
                                                          .MLC = 0.0001f,
                                                          .ESR = 0.0001f,
                                                          .LeakagePower = 2.5f / 4.f };
    MotorPowerModel_s::ModelParam_s wheelUniformMotion = { .K0 = 0.0001f,
                                                           .MLC = 0.0001f,
                                                           .ESR = 0.0001f,
                                                           .LeakagePower = 2.5f / 4.f };
    MotorPowerModel_s wheelModel_{ wheelLaunchMotion };
    RLS<PowerController::FIT_RANK> wheelRLS_{ 0.9999f, Matrix<PowerController::FIT_RANK, 1>({
                                                               { wheelUniformMotion.K0 },  //
                                                               { wheelUniformMotion.MLC }, //
                                                               { wheelUniformMotion.ESR }  //
                                                       }) };

    MotorPowerModel_s steerModel_{ steerLaunchMotion };
    MotorPowerModel_s::ModelParam_s steerLaunchMotion = { .K0 = 0.0001f,
                                                          .MLC = 0.0001f,
                                                          .ESR = 0.0001f,
                                                          .LeakagePower = 2.5f / 4.f };
#if POWERCTRL_ENABLE_STEER_RLS
    RLS<PowerController::FIT_RANK> steerRLS_{ 0.999999999f, Matrix<PowerController::FIT_RANK, 1>({
                                                                    { steerLaunchMotion.K0 },  //
                                                                    { steerLaunchMotion.MLC }, //
                                                                    { steerLaunchMotion.ESR }  //
                                                            }) };
#endif

    float wheelPowerLimitRatio_ = 0.f;
    float steerPowerLimitRatio_ = 0.f;

    float wheelVel_[4]{};
    float wheelTorq_[4]{};
    float steerVel_[4]{};
    float steerTorq_[4]{};

    void updateRLS();
};
