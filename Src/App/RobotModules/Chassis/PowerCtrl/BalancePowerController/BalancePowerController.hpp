#pragma once
#include "PowerController.hpp"
#include "IMotor.hpp"

class BalancePowerController final : public PowerController {
public:
    BalancePowerController() = default;
    struct PowerWBRCtrlMsg_s {
        float powerLimit;
        float cmdCapRatio;
        const PINYMOTOR::IMotor *wheelL;
        const PINYMOTOR::IMotor *wheelR;
        const Matrix<4, 10> &k;
        const Matrix<10, 1> &errX;

        PowerWBRCtrlMsg_s(float _powerLimit, float _cmdCapRatio, const PINYMOTOR::IMotor *_wheelL,
                          const PINYMOTOR::IMotor *_wheelR, const Matrix<4, 10> &_k, const Matrix<10, 1> &_errX)
                : powerLimit(_powerLimit)
                , cmdCapRatio(_cmdCapRatio)
                , wheelL(_wheelL)
                , wheelR(_wheelR)
                , k(_k)
                , errX(_errX)
        {
        }
    };
    void powerWBRCtrl(Matrix<4, 1> &_u, const PowerWBRCtrlMsg_s &_msg);

    // TODO: 单侧腿模型功率控制
    struct PowerWLCtrlMsg_s {};
    void powerWLCtrl(Matrix<2, 1> &_uL, Matrix<2, 1> &_uR, const PowerWLCtrlMsg_s &_msg);

    // TODO: 板凳模型功率控制
    struct PowerSTCtrlMsg_s {};
    void powerSTCtrl(Matrix<1, 1> &_u, const PowerSTCtrlMsg_s &_msg);

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

    float motorVel_[2]{};
    float motorTorq_[2]{};

    void updateRLS();
};