#include "BalancePowerController.hpp"

#include <numeric>

void BalancePowerController::powerWBRCtrl(Matrix<4, 1> &_u, const PowerWBRCtrlMsg_s &_msg)
{
    motorVel_[0] = _msg.wheelL->vel();
    motorTorq_[0] = _msg.wheelL->torq();
    motorVel_[1] = _msg.wheelR->vel();
    motorTorq_[1] = _msg.wheelR->torq();

    // 拟合模型参数并估计功率
    float fitPower[2]{};
    this->estimatePower(motorTorq_, motorVel_, fitPower, wheelModel_);
    float fitTotalPower = std::accumulate(fitPower, fitPower + 2, 0.f);
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

    // 计算原始输出
    Matrix<4, 1> uSpeed = Matrix<4, 1>::zeros();
    Matrix<4, 1> uBalance = Matrix<4, 1>::zeros();
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            uSpeed(i, 0) += _msg.k(i, j) * _msg.errX(j, 0);
        }
        for (uint8_t j = 4; j < 10; j++) {
            uBalance(i, 0) += _msg.k(i, j) * _msg.errX(j, 0);
        }
    }
    _u = uBalance + uSpeed; // U = U_b + U_s
    float cmdTorq[2] = { _u(0, 0), _u(1, 0) };
    float rawSetPower[2]{};
    this->estimatePower(cmdTorq, motorVel_, rawSetPower, wheelModel_);
    float rawSetSumPower = std::accumulate(rawSetPower, rawSetPower + 2, 0.f);
    this->updateChassisRawSetPower(rawSetSumPower);

    // 计算功率限制比例
    powerLimitRatio_ = (rawSetSumPower > powerMax) ? powerMax / rawSetSumPower : 1.f;

    // 计算最终输出
    float setPower[2]{};
    this->limitRawSetPower(rawSetPower, setPower, powerLimitRatio_);
    float uWeaken[2]{};
    this->solveEffectiveCmdTorq(uWeaken, setPower, cmdTorq, motorVel_, wheelModel_);
    // U_s' = U' - U_b
    float uSpeedWeaken[2] = { std::max(0.f, uWeaken[0] - uBalance(0, 0)), std::max(0.f, uWeaken[1] - uBalance(1, 0)) };
    // factor = U_s' / U_s
    float attenuationFactor[2] = { uSpeedWeaken[0] / uSpeed(0, 0), uSpeedWeaken[1] / uSpeed(1, 0) };
    // U = U_b + U_s * factor
    _u(0, 0) = uBalance(0, 0) + uSpeedWeaken[0];
    _u(1, 0) = uBalance(1, 0) + uSpeedWeaken[1];
    _u(2, 0) = uBalance(2, 0) + uSpeed(2, 0) * attenuationFactor[0];
    _u(3, 0) = uBalance(3, 0) + uSpeed(3, 0) * attenuationFactor[1];
}

void BalancePowerController::updateRLS()
{
    if (std::ranges::any_of(motorVel_, [](float _vel) { return std::fabs(_vel) > VEL_THESHOLD; })) {
        this->fitting(motorVel_, motorTorq_, &wheelRLS_, wheelModel_.modelParams);
    } else
        wheelModel_.overrideParams(LaunchMotion);
}