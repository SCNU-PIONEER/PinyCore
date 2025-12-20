#include "QuadricycleController.hpp"
#include <algorithm>
#include <array>

using namespace PINYMOTOR;

QuadricycleController::QuadricycleController(ChassisType_e _chassisType, CAP *_cap)
        : PowerController(_chassisType, _cap)
{
    float estVector[PowerModel_s::FIT_RANK] = { UniformMotion.K0, UniformMotion.MLC, UniformMotion.ESR };
    wheelRLS_ = new RLS<PowerModel_s::FIT_RANK>(0.99999999f, Matrix<PowerModel_s::FIT_RANK, 1>(estVector));
    Wheel.overrideParams(LaunchMotion);
}

void QuadricycleController::cmdPowerCalc(const float *_motorSpeed, IMotor *_motor[4], const float *_cmd)
{
    chassisRawPower_ = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        const float cmdVel = _motorSpeed[i] * _motor[i]->rr();
        cmdPower_[i] = Wheel.power(_cmd[i], cmdVel);
        chassisRawPower_ += cmdPower_[i];
    }
}

void QuadricycleController::relPowerCalc(IMotor *_motor[4])
{
    chassisRealPower_ = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        const float relVel = _motor[i]->vel() * _motor[i]->rr();
        relPower_[i] = Wheel.power(_motor[i]->torq(), relVel);
        chassisRealPower_ += relPower_[i];
    }
}

void QuadricycleController::torqueCalc(IMotor *_motor[4], const float *_cmd)
{
    for (uint8_t i = 0; i < motorNum_; i++) {
        const float cmdVel = _motor[i]->vel() * _motor[i]->rr();
        float discriminant = std::max(Wheel.delta(cmdVel, setPower_[i]), 0.f);
        float sign = (_cmd[i] == 0) ? 0.f : ((_cmd[i] > 0) ? 1.f : -1.f);
        setTorq_[i] = (sign >= 0) ? (std::clamp((-(Wheel.modelParams.K0 * cmdVel) + sign * sqrtf(discriminant)) /
                                                        (2 * Wheel.modelParams.ESR),
                                                0.f, _cmd[i])) :
                                    std::clamp((-(Wheel.modelParams.K0 * cmdVel) + sign * sqrtf(discriminant)) /
                                                       (2 * Wheel.modelParams.ESR),
                                               _cmd[i], 0.f);
    }
}

void QuadricycleController::rlsUpdate(IMotor *_motor[4])
{
    std::array<float, 4> vel = {};
    float vectorValue[PowerModel_s::FIT_RANK] = {};
    for (uint8_t i = 0; i < motorNum_; i++) {
        vel[i] = _motor[i]->vel() * _motor[i]->rr();
        vectorValue[0] += _motor[i]->torq() * vel[i];
        vectorValue[1] += vel[i] * vel[i];
        vectorValue[2] += _motor[i]->torq() * _motor[i]->torq();
    }
    if (std::ranges::any_of(vel, [](float _vel) { return _vel > VEL_THESHOLD; })) {
        Matrix<PowerModel_s::FIT_RANK, 1> inputVector(vectorValue);
        wheelRLS_->update(inputVector, capFeedbackPower_ - (Wheel.modelParams.LeakagePower * 4.f));
        Matrix<PowerModel_s::FIT_RANK, 1> params = wheelRLS_->getEstVector();
        Wheel.overrideParams({ .K0 = params(0, 0), .MLC = params(1, 0), .ESR = params(2, 0) });
    } else
        Wheel.overrideParams(LaunchMotion);
}

std::vector<float> QuadricycleController::powerCtrl(const float *_motorSpeed, IMotor *_motor[4], const float *_cmd,
                                                    const RefereeMsg_s &_msg)
{
#if POWERCTRL_USE_RLS
    rlsUpdate(_motor);
#endif
    update(_msg);

    cmdPowerCalc(_motorSpeed, _motor, _cmd);
    if (chassisRawPower_ > maxPower_) //限制最大输出功率
        powerRatio_ = maxPower_ / chassisRawPower_;
    else
        powerRatio_ = 1.f;

    chassisSetPower = 0;
    static float lastSetPower = 0;
    for (uint8_t i = 0; i < motorNum_; i++) {
        setPower_[i] = cmdPower_[i] * powerRatio_;
        chassisSetPower += setPower_[i];
    }
    torqueCalc(_motor, _cmd);
    relPowerCalc(_motor);

    static uint32_t taskTick = 0;
    float setPowerDot = (chassisSetPower - lastSetPower) / static_cast<float>(xTaskGetTickCount() - taskTick) * 1000.f;
    lastSetPower = chassisSetPower;
    taskTick = xTaskGetTickCount();
    if (cap_ != nullptr)
        cap_->chargeCmdPower = std::clamp(limitPower_ - (0.01f * setPowerDot), 30.f, 120.f);

    return setTorq_;
}
