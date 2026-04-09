#pragma once
#include "PidBasic.hpp"
#include <vector>
#include "IMotor.hpp"
#include "SuperCap.hpp"
#include "RLS.hpp"
#include "sdkconfig.h"
#include "MsgImpl.hpp"
#include "IIR.hpp"

struct PowerModel_s {
    static constexpr const uint8_t FIT_RANK = 3; //拟合参数个数
    struct ModelParam_s {
        float K0;           // k0 Transmit constant（转化系数）
        float MLC;          // k1 Mechanical loss coefficient （机械损耗系数）
        float ESR;          // k2 Equivalent Series Resistance（等效串联电阻）
        float LeakagePower; // k3 静态功耗
    } modelParams;

    void overrideParams(const ModelParam_s &_params) { modelParams = _params; }

    float power(float _tau, float _omega)
    {
        // P = k0 * τ * ω + k1 * ω² + k2 * τ² + k3
        return (modelParams.K0 * _tau * _omega) + (modelParams.MLC * _omega * _omega) +
               (modelParams.ESR * _tau * _tau) + modelParams.LeakagePower;
    }

    float delta(float _omega, float _p)
    {
        // Δ = (k0 * ω)^2 - 4 * k2 * (k1 * ω^2 + k3 - P)
        return (modelParams.K0 * _omega * modelParams.K0 * _omega) -
               (4.f * modelParams.ESR * (modelParams.MLC * _omega * _omega + modelParams.LeakagePower - _p));
    }
};

enum class ChassisType_e : uint8_t {
    QUADRICYCLE = 4u,
    WHEELLEG = 6u,
    SWERVE = 8u,
};

class PowerController {
    static constexpr float REMAIN_POWER = 10.f;

public:
    PowerController(ChassisType_e _chassisType, SuperCap *_cap);

    //五个发送给超电的数据
    bool capEnable_ = true;          //超电使能
    bool systemRestart_ = false;     //系统重启
    bool clearError_ = false;        //清除错误
    bool enableCharge_ = true;       //启用主动充电限制
    uint8_t chargeRatioLimit_ = 255; //主动充电限制比例，0-255（无线充电时使用）

    float capCmdRatio_ = 0.f; //超电能量命令比例（0-1）
    float capRealRatio_ = 1.f;

protected:
    uint8_t motorNum_;     //电机数量
    float maxPower_ = 0.f; // 允许最大输出功率

    std::vector<float> cmdPower_; // 原闭环控制器所设定的功率
    std::vector<float> fitPower_; // 根据电机数据拟合出的实际功率

    float chassisRawPower_ = 0.f;  // 未经过功率控制的原始底盘功率
    float chassisFitPower_ = 0.f;  // 根据模型算出的实际输出功率（与超电反馈功率比较反映模型拟合程度）
    float chassisRealPower_ = 0.f; // 实际输出功率
    float chassisSetPower_ = 0.f;

    std::vector<float> setTorq_;  // 最终设定输出力矩
    std::vector<float> setPower_; // 功率控制后所得的功率
    float powerRatio_ = 1.f;

    SuperCap *cap_;
    void update(const RefereeMsg_s &_msg);

private:
    ChassisType_e chassisType_;
    PositionalPid powerPid_{ 250.f, 0, 0, 0.001f, 0, 200.f, 0 };
    FILTER::LPFIIR3 realPowerFilter{ 1000.f, 12.f, FILTER::FilterType_e::BUTTERWORTH, FILTER::Ripple_e::NONE };
};
