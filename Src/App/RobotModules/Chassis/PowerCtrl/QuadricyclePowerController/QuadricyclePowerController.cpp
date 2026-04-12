#include "QuadricyclePowerController.hpp"

#include <numeric>

using namespace PINYMOTOR;

void QuadricyclePowerController::powerCtrl(float (&_setTorq)[4], const PowerCtrlMsg_s &_msg)
{
    for (uint8_t i = 0; i < 4; ++i) {
        motorVel_[i] = _msg.motor[i]->vel();
        motorTorq_[i] = _msg.motor[i]->torq();
    }

    // 更新底盘功率信息
#if EXTENSION_SUPERCAP
    if (this->cap_ != nullptr)
        updateChassisRealPower(this->cap_->getCapData().outputPower);
#endif

    // 拟合模型参数并估计功率
    float fitPower[4]{};
    this->estimatePower(motorTorq_, motorVel_, fitPower, wheelModel_);
    updateChassisFitPower(std::accumulate(fitPower, fitPower + 4, 0.f));

#if POWERCTRL_USE_RLS
    updateRLS();
#endif

    // 计算功率上限
    float powerMax = this->updateAllowablePower(_msg.powerLimit);

    // 计算原始输出
    float rawSetPower[4]{};
    this->estimatePower(_msg.cmdTorq, _msg.cmdVel, rawSetPower, wheelModel_);
    float rawSetSumPower = std::accumulate(rawSetPower, rawSetPower + 4, 0.f);
    this->updateChassisRawSetPower(rawSetSumPower);

    // 计算功率限制比例
    powerLimitRatio_ = (rawSetSumPower > powerMax) ? powerMax / rawSetSumPower : 1.f;

    // 计算最终输出
    float setPower[4]{};
    this->limitRawSetPower(rawSetPower, setPower, powerLimitRatio_);
    this->solveEffectiveCmdTorq(_setTorq, setPower, _msg.cmdTorq, _msg.cmdVel, wheelModel_);

    this->updateChassisSetPower(std::accumulate(setPower, setPower + 4, 0.f));
}

void QuadricyclePowerController::updateRLS()
{
    if (std::ranges::any_of(motorVel_, [](float _vel) { return _vel > VEL_THESHOLD; })) {
        this->fitting(motorVel_, motorTorq_, &wheelRLS_, wheelModel_.modelParams);
    } else
        wheelModel_.overrideParams(LaunchMotion);
}
