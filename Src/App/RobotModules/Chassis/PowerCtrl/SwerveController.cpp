#include "SwerveController.hpp"
#include <algorithm>
#include <array>

using namespace PINYMOTOR;

SwerveController::SwerveController(ChassisType_e _chassisType, CAP *_cap) : PowerController(_chassisType, _cap)
{
    Wheel.overrideParams(WheelLaunchMotion);
    float wheelEstVector[PowerModel_s::FIT_RANK] = { WheelUniformMotion.K0, WheelUniformMotion.MLC,
                                                     WheelUniformMotion.ESR };
    wheelRLS_ = new RLS<PowerModel_s::FIT_RANK>(0.999f, Matrix<PowerModel_s::FIT_RANK, 1>(wheelEstVector));

    Steer.overrideParams(SteerLaunchMotion);
#if ENABLE_STEER_RLS
    float steerEstVector[PowerModel_s::FIT_RANK] = { SteerLaunchMotion.K0, SteerLaunchMotion.MLC,
                                                     SteerLaunchMotion.ESR };
    steerRLS_ = new RLS<PowerModel_s::FIT_RANK>(0.999f, Matrix<PowerModel_s::FIT_RANK, 1>(steerEstVector));
#endif
}
float motorCmdRads[8] = {};
void SwerveController::cmdPowerCalc(const float *_motorSpeed, IMotor *_motor[8], const float *_cmd)
{
    wheelCmdPower_ = 0.f;
    steerCmdPower_ = 0.f;
    for (uint8_t i = 0; i < motorNum_ / 2; i++) {
        const float wheelCmdVel = _motorSpeed[i] * _motor[i]->rr();
        cmdPower_[i] = Wheel.power(_cmd[i], wheelCmdVel);
        wheelCmdPower_ += cmdPower_[i];

        const float steerCmdVel = _motor[i + 4]->vel() * _motor[i + 4]->rr();
        cmdPower_[i + 4] = Steer.power(_cmd[i + 4], steerCmdVel);
        steerCmdPower_ += cmdPower_[i + 4];
        motorCmdRads[i] = wheelCmdVel;
        motorCmdRads[i + 4] = steerCmdVel;
    }
    chassisRawPower_ = wheelCmdPower_ + steerCmdPower_;
}
float motorRelRads[8] = {};
void SwerveController::relPowerCalc(IMotor *_motor[8])
{
    wheelRelPower_ = 0;
    steerRelPower_ = 0;
    for (uint8_t i = 0; i < motorNum_ / 2; i++) {
        const float wheelRelVel = _motor[i]->vel() * _motor[i]->rr();
        relPower_[i] = Wheel.power(_motor[i]->torq(), wheelRelVel);
        wheelRelPower_ += relPower_[i];

        const float steerRelVel = _motor[i + 4]->vel() * _motor[i + 4]->rr();
        relPower_[i + 4] = Steer.power(_motor[i + 4]->torq(), steerRelVel);
        steerRelPower_ += relPower_[i + 4];
        motorRelRads[i] = wheelRelVel;
        motorRelRads[i + 4] = steerRelVel;
    }
    chassisRealPower_ = wheelRelPower_ + steerRelPower_;
}

void SwerveController::torqueCalc(IMotor *_motor[8], const float *_cmd)
{
    for (uint8_t i = 0; i < motorNum_ / 2; i++) {
        const float wheelRelVel = _motor[i]->vel() * _motor[i]->rr();
        float discriminant = std::max(Wheel.delta(wheelRelVel, setPower_[i]), 0.f);
        float sign = (_cmd[i] == 0) ? 0.f : ((_cmd[i] > 0) ? 1.f : -1.f);
        setTorq_[i] = (sign >= 0) ? (std::clamp((-(Wheel.modelParams.K0 * wheelRelVel) + sign * sqrtf(discriminant)) /
                                                        (2 * Wheel.modelParams.ESR),
                                                0.f, _cmd[i])) :
                                    std::clamp((-(Wheel.modelParams.K0 * wheelRelVel) + sign * sqrtf(discriminant)) /
                                                       (2 * Wheel.modelParams.ESR),
                                               _cmd[i], 0.f);

        const float steerRelVel = _motor[i + 4]->vel() * _motor[i + 4]->rr();
        discriminant = std::max(Steer.delta(steerRelVel, setPower_[i + 4]), 0.f);
        sign = (_cmd[i + 4] == 0) ? 0.f : ((_cmd[i + 4] > 0) ? 1.f : -1.f);
        setTorq_[i + 4] = (sign >= 0) ?
                                  (std::clamp((-(Steer.modelParams.K0 * steerRelVel) + sign * sqrtf(discriminant)) /
                                                      (2 * Steer.modelParams.ESR),
                                              0.f, _cmd[i + 4])) :
                                  std::clamp((-(Steer.modelParams.K0 * steerRelVel) + sign * sqrtf(discriminant)) /
                                                     (2 * Steer.modelParams.ESR),
                                             _cmd[i + 4], 0.f);
    }
}
float kn = 0;
float mlc = 0;
float esr = 0;
void SwerveController::rlsUpdate(IMotor *_motor[8])
{
    std::array<float, 4> vel = {};
    float vectorValue[PowerModel_s::FIT_RANK] = {};
    for (uint8_t i = 0; i < motorNum_ / 2; i++) {
        vel[i] = _motor[i]->vel() * _motor[i]->rr();
        vectorValue[0] += _motor[i]->torq() * vel[i];
        vectorValue[1] += vel[i] * vel[i];
        vectorValue[2] += _motor[i]->torq() * _motor[i]->torq();
    }
    if (std::ranges::any_of(vel, [](float _vel) { return _vel > VEL_THESHOLD; })) {
        Matrix<PowerModel_s::FIT_RANK, 1> inputVector(vectorValue);
        float fitPower = capFeedbackPower_ - steerRelPower_ - (Wheel.modelParams.LeakagePower * 4.f);
        wheelRLS_->update(inputVector, fitPower > 0.f ? fitPower : 0.f);
        Matrix<PowerModel_s::FIT_RANK, 1> params = wheelRLS_->getEstVector();
        Wheel.overrideParams({ .K0 = params(0, 0), .MLC = params(1, 0), .ESR = params(2, 0) });
    } else
        Wheel.overrideParams(WheelLaunchMotion);

#if ENABLE_STEER_RLS
    float steerVector[PowerModel_s::FIT_RANK] = {};
    for (uint8_t i = 0; i < motorNum_ / 2; i++) {
        const float vel = _motor[i + 4]->vel() * _motor[i + 4]->rr();
        steerVector[0] += _motor[i + 4]->torq() * vel;
        steerVector[1] += vel * vel;
        steerVector[2] += _motor[i + 4]->torq() * _motor[i + 4]->torq();
    }
    Matrix<PowerModel_s::FIT_RANK, 1> steerInput(steerVector);
    float steerFitPower = capFeedbackPower_ - (Steer.modelParams.LeakagePower * 4.f);
    steerRLS_->update(steerInput, steerFitPower > 0.f ? steerFitPower : 0.f);
    Matrix<PowerModel_s::FIT_RANK, 1> steerParams = steerRLS_->getEstVector();
    Steer.overrideParams({ .K0 = steerParams(0, 0), .MLC = steerParams(1, 0), .ESR = steerParams(2, 0) });
#endif
    kn = Wheel.modelParams.K0;
    mlc = Wheel.modelParams.MLC;
    esr = Wheel.modelParams.ESR;
}
float rawPower = 0;
float realPower = 0;
float clacPower = 0;
float setpower = 0;
float steerPower = 0;
std::vector<float> SwerveController::powerCtrl(const float *_motorSpeed, PINYMOTOR::IMotor *_motor[8],
                                               const float *_cmd, RefereeMsg_s _msg)
{
#if POWERCTRL_USE_RLS
    //rlsUpdate(_motor);
#endif
    update(_msg);

    cmdPowerCalc(_motorSpeed, _motor, _cmd);

    float steerMaxPower = maxPower_ * 0.8f;
    float wheelMaxPower = std::max(maxPower_ * 0.2f, maxPower_ - steerCmdPower_);
    steerPower = steerCmdPower_;
    wheelPowerRatio_ = (wheelCmdPower_ > wheelMaxPower) ? (wheelMaxPower / wheelCmdPower_) : 1.f;
    steerPowerRatio_ = (steerCmdPower_ > steerMaxPower) ? (steerMaxPower / steerCmdPower_) : 1.f;
    float chassisSetPower = 0;
    static float lastSetPower = 0;
    for (uint8_t i = 0; i < 4; i++) {
        setPower_[i] = cmdPower_[i] * wheelPowerRatio_;
        setPower_[i + 4] = cmdPower_[i + 4] * steerPowerRatio_;
        chassisSetPower += setPower_[i];
        chassisSetPower += setPower_[i + 4];
    }

    torqueCalc(_motor, _cmd);
    relPowerCalc(_motor);

    rawPower = chassisRawPower_;
    realPower = capFeedbackPower_;
    clacPower = chassisRealPower_;
    setpower = maxPower_;

    static uint32_t taskTick = 0;
    float setPowerDot = (chassisSetPower - lastSetPower) / static_cast<float>(xTaskGetTickCount() - taskTick) * 1000.f;
    lastSetPower = chassisSetPower;
    taskTick = xTaskGetTickCount();
    if (cap_ != nullptr)
        cap_->chargeCmdPower = std::clamp(limitPower_ - (0.01f * setPowerDot), 30.f, 120.f);

    return setTorq_;
}