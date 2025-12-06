#pragma once
#include <vector>
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include "SuperCap.hpp"
#include "RLS.hpp"
#include "sdkconfig.h"
#include "MsgImpl.hpp"

struct PowerModel_s {
    float K0;           // k0 Transmit constant（转化系数）
    float MLC;          // k1 Mechanical loss coefficient （机械损耗系数）
    float ESR;          // k2 Equivalent Series Resistance（等效串联电阻）
    float LeakagePower; // k3 静态功耗

    float power(float _tau, float _omega)
    {
        // P = k0 * τ * ω + k1 * ω² + k2 * τ² + k3
        return (K0 * _tau * _omega) + (MLC * _omega * _omega) + (ESR * _tau * _tau) + LeakagePower;
    }

    float delta(float _omega, float _p)
    {
        // Δ = (k0 * ω)^2 - 4 * k2 * (k1 * ω^2 + k3 - P)
        return (K0 * _omega * K0 * _omega) - (4.f * ESR * (MLC * _omega * _omega + LeakagePower - _p));
    }
};

enum class ChassisType_e : uint8_t {
    QUADRICYCLE = 4u,
    WHEELLEG = 6u,
    SWERVE = 8u,
};

enum class ErrorCode_e : uint8_t {
    NO_ERROR = 0u,
    CAP_DISCONNECT = 1u,
    REFREEE_DISCONNECT = 2u,
    ALL_DISCONNECT = 3u,
};

class PowerController {
public:
    PowerController(ChassisType_e _chassisType, CAP *_cap);

    //四个发送给超电的数据
    float chargeCmdPower = 0.f; //期望电容充电功率
    bool capEnable_ = true;     //超电使能
    bool capCharge = true;      //超电充电使能
    float chassisSetPower = 0.f;

protected:
    static constexpr float VCAP_MAX = 26.f;
    static constexpr float VCAP_MIN = 5.f;
    static constexpr float VOLTAGE_RANGE = (VCAP_MAX * VCAP_MAX) - (VCAP_MIN * VCAP_MIN);

    uint8_t motorNum_; //电机数量

    float limitPower_ = 10.f;
    float maxPower_ = 0.f; // 允许最大输出功率（经过动态规划）
    float offsetPower_ = 0.f;
    float powerBuffer_ = 60.f;    // 实际缓冲能量值,从裁判系统读取,亦可软件设定
    float expPowerBuffer_ = 60.f; // 期望缓冲能量值

    std::vector<float> cmdPower_; // 原闭环控制器所设定的功率
    float chassisRawPower_ = 0.f; // 未经过功率控制的原始底盘功率

    std::vector<float> relPower_;  // 根据电机数据拟合出的实际功率
    float chassisRealPower_ = 0.f; // 根据模型算出的实际输出功率（与超电反馈功率比较反映模型拟合程度）
    float capFeedbackPower_ = 0.f; // 实际输出功率

    std::vector<float> setIq_;    // 最终设定输出电流
    std::vector<float> setPower_; // 功率控制后所得的功率
    float powerRatio_ = 1.f;

    float capCmdRatio_ = 0.9f;
    float capRealRatio_ = 1.f;

    CAP *cap_;

    void update(const RefereeMsg_s &_msg);

private:
    ChassisType_e chassisType_;
    PositionalPid energyPid_{ 0.1f, 0, 0, 0.001f, 0, 0, 0 };
    PositionalPid powerPid_{ 300.f, 0, 0, 0.001f, 0, 400.f, 0 };

    ErrorCode_e errorState_ = ErrorCode_e::NO_ERROR;

    void dynamicPower(float _capVoltage);
    void updateReferee(const RefereeMsg_s &_msg);
    void errorCheck(float _refereeFreq, float _capFreq);
};
