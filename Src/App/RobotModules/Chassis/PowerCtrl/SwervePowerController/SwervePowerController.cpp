#include "SwervePowerController.hpp"

#include <numeric>

using namespace PINYMOTOR;

void SwervePowerController::powerCtrl(float (&_setWheelTorq)[4], float (&_setSteerTorq)[4], const PowerCtrlMsg_s &_msg)
{
    for (uint8_t i = 0; i < 4; ++i) {
        wheelVel_[i] = _msg.wheelMotor[i]->vel();
        wheelTorq_[i] = _msg.wheelMotor[i]->torq();
        steerVel_[i] = _msg.steerMotor[i]->vel();
        steerTorq_[i] = _msg.steerMotor[i]->torq();
    }

    // 拟合模型参数并估计功率
    float fitWheelPower[4]{};
    this->estimatePower(wheelTorq_, wheelVel_, fitWheelPower, wheelModel_);
    float fitWheelSumPower = std::accumulate(fitWheelPower, fitWheelPower + 4, 0.f);
    float fitSteerPower[4]{};
    this->estimatePower(steerTorq_, steerVel_, fitSteerPower, steerModel_);
    float fitSteerSumPower = std::accumulate(fitSteerPower, fitSteerPower + 4, 0.f);
    float fitTotalPower = fitWheelSumPower + fitSteerSumPower;
    updateChassisFitPower(fitTotalPower);

    // 更新底盘功率信息
#if EXTENSION_SUPERCAP
    if (this->cap_ != nullptr)
        updateChassisRealPower(this->cap_->getCapData().outputPower);
#else
    updateChassisRealPower(fitTotalPower);
#endif

#if POWERCTRL_USE_RLS
    updateRLS();
#endif

    // 计算功率上限
    float powerMax = this->updateAllowablePower(_msg.powerLimit, _msg.cmdCapRatio);
    float steerPowerMax = powerMax * 0.8f;
    float wheelPowerMax = std::max(powerMax * 0.2f, powerMax - fitSteerSumPower);

    // 计算原始输出
    float rawSetWheelPower[4]{};
    this->estimatePower(_msg.cmdWheelTorq, _msg.cmdWheelVel, rawSetWheelPower, wheelModel_);
    float rawSetWheelSumPower = std::accumulate(rawSetWheelPower, rawSetWheelPower + 4, 0.f);

    float rawSetSteerPower[4]{};
    this->estimatePower(_msg.cmdSteerTorq, _msg.cmdSteerVel, rawSetSteerPower, steerModel_);
    float rawSetSteerSumPower = std::accumulate(rawSetSteerPower, rawSetSteerPower + 4, 0.f);
    this->updateChassisRawSetPower(rawSetWheelSumPower + rawSetSteerSumPower);

    // 计算功率限制比例
    wheelPowerLimitRatio_ = (rawSetWheelSumPower > wheelPowerMax) ? wheelPowerMax / rawSetWheelSumPower : 1.f;
    steerPowerLimitRatio_ = (rawSetSteerSumPower > steerPowerMax) ? steerPowerMax / rawSetSteerSumPower : 1.f;

    // 计算最终输出
    float setWheelPower[4]{};
    this->limitRawSetPower(rawSetWheelPower, setWheelPower, wheelPowerLimitRatio_);
    this->solveEffectiveCmdTorq(_setWheelTorq, setWheelPower, _msg.cmdWheelTorq, wheelVel_, wheelModel_);
    float setSteerPower[4]{};
    this->limitRawSetPower(rawSetSteerPower, setSteerPower, steerPowerLimitRatio_);
    this->solveEffectiveCmdTorq(_setSteerTorq, setSteerPower, _msg.cmdSteerTorq, steerVel_, steerModel_);

    float setWheelSumPower = std::accumulate(setWheelPower, setWheelPower + 4, 0.f);
    float setSteerSumPower = std::accumulate(setSteerPower, setSteerPower + 4, 0.f);
    this->updateChassisSetPower(setWheelSumPower + setSteerSumPower);
}

void SwervePowerController::updateRLS()
{
    if (std::ranges::any_of(wheelVel_, [](float _vel) { return std::fabs(_vel) > VEL_THESHOLD; })) {
        this->fitting(wheelVel_, wheelTorq_, &wheelRLS_, wheelModel_.modelParams);
    } else
        wheelModel_.overrideParams(wheelLaunchMotion);

#if POWERCTRL_ENABLE_STEER_RLS
    this->fitting(steerVel_, steerTorq_, &steerRLS_, steerModel_.modelParams);
#endif
}