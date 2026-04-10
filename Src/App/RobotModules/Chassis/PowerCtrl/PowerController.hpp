#pragma once
#include <algorithm>
#include "RLS.hpp"
#include "dsp/fast_math_functions.h"
#include "sdkconfig.h"

#if EXTENSION_SUPERCAP
#include "PidBasic.hpp"
#include "IIR.hpp"
#include "SuperCap.hpp"
#endif

struct MotorPowerModel_s {
    struct ModelParam_s {
        float K0;           // k0 Transmit constant（转化系数）
        float MLC;          // k1 Mechanical loss coefficient （机械损耗系数）
        float ESR;          // k2 Equivalent Series Resistance（等效串联电阻）
        float LeakagePower; // k3 静态功耗
    } modelParams;

    MotorPowerModel_s() = default;
    MotorPowerModel_s(const ModelParam_s &_params) : modelParams(_params) {}

    void overrideParams(const ModelParam_s &_params) { modelParams = _params; }

    float power(float _tau, float _omega) const
    {
        // P = k0 * τ * ω + k1 * ω² + k2 * τ² + k3
        return (modelParams.K0 * _tau * _omega) + (modelParams.MLC * _omega * _omega) +
               (modelParams.ESR * _tau * _tau) + modelParams.LeakagePower;
    }

    float delta(float _omega, float _p) const
    {
        // Δ = (k0 * ω)^2 - 4 * k2 * (k1 * ω^2 + k3 - P)
        return (modelParams.K0 * _omega * modelParams.K0 * _omega) -
               (4.f * modelParams.ESR * (modelParams.MLC * _omega * _omega + modelParams.LeakagePower - _p));
    }
};

class PowerController {
    static constexpr float REMAIN_POWER = 10.f;

public:
    static constexpr uint8_t FIT_RANK = 3; //拟合参数个数

    PowerController() = default;

#if EXTENSION_SUPERCAP
    void loadCap(SuperCap *_cap) { cap_ = _cap; }
#endif

protected:
    /**
     * @brief 拟合模型参数
     * 
     * @tparam N 参与拟合的电机数量
     * @param _rls RLS对象指针
     * @param _params 操作的模型参数对象引用
     */
    template <uint8_t N>
    void fitting(const float (&_vel)[N], const float (&_torq)[N], RLS<FIT_RANK> *_rls,
                 MotorPowerModel_s::ModelParam_s &_params)
    {
        float vectorValue[FIT_RANK] = {};
        for (uint8_t i = 0; i < N; ++i) {
            vectorValue[0] += _torq[i] * _vel[i];
            vectorValue[1] += _vel[i] * _vel[i];
            vectorValue[2] += _torq[i] * _torq[i];
        }
        Matrix<FIT_RANK, 1> inputVector(vectorValue);
        _rls->update(inputVector, chassisFitPower_ - _params.LeakagePower);
        Matrix<FIT_RANK, 1> params = _rls->getEstVector();
        _params.K0 = params(0, 0);
        _params.MLC = params(1, 0);
        _params.ESR = params(2, 0);
    }

    /**
     * @brief 通过给定模型估计电机功率
     * 
     * @tparam N 
     * @param _model 
     */
    template <uint8_t N>
    void estimatePower(const float (&_torq)[N], const float (&_vel)[N], float (&_power)[N],
                       const MotorPowerModel_s &_model)
    {
        for (uint8_t i = 0; i < N; ++i) {
            _power[i] = _model.power(_torq[i], _vel[i]);
        }
    }

    /**
     * @brief 更新底盘真实功率
     * 
     * @param _realPower 一般使用功率板反馈的数值作为输入
     */
    void updateChassisRealPower(float _realPower) { chassisRealPower_ = realPowerFilter_.process(_realPower); }

    /**
     * @brief 更新底盘估计功率
     * 
     * @param _fitPower
     */
    void updateChassisFitPower(float _fitPower) { chassisFitPower_ = _fitPower; }

    /**
     * @brief 更新原始期望输出功率
     * 
     * @param _chassisRawSetPower
     */
    void updateChassisRawSetPower(float _chassisRawSetPower) { chassisRawSetPower_ = _chassisRawSetPower; }

    /**
     * @brief 更新最终期望输出功率
     * @note chassisSetPower_仅为监视量，存在与否不会影响主要功能
     * 
     * @param _chassisSetPower 
     */
    void updateChassisSetPower(float _chassisSetPower) { chassisSetPower_ = _chassisSetPower; }

    /**
     * @brief 通过功率限制比例限制原始设定功率，得到最终设定功率
     * 
     * @tparam N 
     */
    template <uint8_t N> void limitRawSetPower(const float (&_rawSetPower)[N], float (&_setPower)[N], float _ratio)
    {
        for (uint8_t i = 0; i < N; ++i) {
            _setPower[i] = _rawSetPower[i] * _ratio;
        }
    }

    /**
     * @brief 解算一元二次方程，根据最终设定功率计算有效的设定转矩
     * 
     * @tparam N 
     * @param _model 
     */
    template <uint8_t N>
    void solveEffectiveCmdTorq(float (&_result)[N], const float (&_cmdPower)[N], const float (&_cmdTorq)[N],
                               const float (&_cmdVel)[N], const MotorPowerModel_s &_model)
    {
        for (uint8_t i = 0; i < N; ++i) {
            float discriminant = std::max(_model.delta(_cmdVel[i], _cmdPower[i]), 0.f);
            float sqrtDiscriminant;
            arm_sqrt_f32(discriminant, &sqrtDiscriminant);
            bool signBit = std::signbit(_cmdTorq[i]); // x < 0, return true
            float sign = std::copysignf(1.f, _cmdTorq[i]);
            _result[i] = signBit ? std::clamp((-(_model.modelParams.K0 * _cmdVel[i]) + sign * sqrtDiscriminant) /
                                                      (2 * _model.modelParams.ESR),
                                              _cmdTorq[i], 0.f) :
                                   std::clamp((-(_model.modelParams.K0 * _cmdVel[i]) + sign * sqrtDiscriminant) /
                                                      (2 * _model.modelParams.ESR),
                                              0.f, _cmdTorq[i]);
        }
    }

#if EXTENSION_SUPERCAP
    SuperCap *cap_;
    FILTER::LPFIIR3 realPowerFilter_{ 1000.f, 12.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE };
    PositionalPid powerPid_{ 250.f, 0, 0, 0.001f, 0, 200.f, 0 };
    /**
     * @brief 更新允许的最大输出功率
     * 
     * @param _chassisPowerLimit 
     * @param _capCmdRatio 
     */
    float updateAllowablePower(float _chassisPowerLimit, float _capCmdRatio = 0.f)
    {
        if (cap_ != nullptr) {
            CapData_s capData = cap_->getCapData();
            _capCmdRatio = std::clamp(_capCmdRatio, 0.f, 1.f);
            float offsetPower = powerPid_.calc(_capCmdRatio, capData.capEnergyRatio);
            offsetPower = std::max(offsetPower, REMAIN_POWER);
            allowablePower_ = _chassisPowerLimit - offsetPower;
        }
        return allowablePower_;
    }
#else
    float updateAllowablePower(float _chassisPowerLimit) { return allowablePower_ = _chassisPowerLimit; }
#endif

private:
    float allowablePower_ = 0.f;     // 允许最大输出功率
    float chassisRealPower_ = 0.f;   // 真实实际输出功率
    float chassisFitPower_ = 0.f;    // 估计实际输出功率
    float chassisRawSetPower_ = 0.f; // 原始期望输出功率
    float chassisSetPower_ = 0.f;    // 最终期望输出功率
};
