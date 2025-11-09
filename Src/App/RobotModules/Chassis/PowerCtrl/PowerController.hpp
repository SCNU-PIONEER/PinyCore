#pragma once
#include <vector>
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include "SuperCap.hpp"
#include "RLS.hpp"
#include "sdkconfig.h"
#include "MsgImpl.hpp"

struct MotorParam_s {
    float KN;           //Torque constant（扭矩常数）
    float MLC;          //Mechanical loss coefficient （机械损耗系数）
    float ESR;          //Equivalent Series Resistance（等效串联电阻）
    float LeakagePower; //静态功耗
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
    PowerController(ChassisType_e _chassisType);
    virtual ~PowerController() = default;

    virtual void relPowerCalc(PINYMOTOR::IMotor **_motor) = 0;
    virtual void cmdPowerCalc(const float *_motorSpeed,
                              PINYMOTOR::IMotor **_motor,
                              const float *_cmd) = 0;
    virtual void currentCalc(PINYMOTOR::IMotor **_motor, const float *_cmd) = 0;

    virtual std::vector<float> powerCtrl(const float *_motorSpeed,
                                         PINYMOTOR::IMotor **_motor,
                                         const float *_cmd,
                                         RefereeMsg_s _msg) = 0;
    virtual void rlsUpdate(PINYMOTOR::IMotor **_motor) = 0;

    void update(RefereeMsg_s _msg);
    void dynamicPower(float _capVoltage);
    void updateReferee(RefereeMsg_s _msg);
    void errorCheck(float _refereeFreq, float _capFreq);

#if EXTENSION_SUPERCAP
    CAP &getCap() { return cap_; }
#endif
    //四个发送给超电的数据
    float chargeCmdPower = 0.f; //期望电容充电功率
    bool capEnable_ = true;     //超电使能
    bool capCharge = true;      //超电充电使能
    float chassisSetPower = 0.f;

protected:
    static constexpr float VCAP_MAX = 26.f;
    static constexpr float VCAP_MIN = 5.f;
    static constexpr float VOLTAGE_RANGE =
            (VCAP_MAX * VCAP_MAX) - (VCAP_MIN * VCAP_MIN);

    uint8_t motorNum_; //电机数量

    float limitPower = 10.f;
    float maxPower = 0.f; // 允许最大输出功率（经过动态规划）
    float offsetPower = 0.f;
    float powerBuffer = 60.f;    // 实际缓冲能量值,从裁判系统读取,亦可软件设定
    float expPowerBuffer = 60.f; // 期望缓冲能量值

    std::vector<float> cmdPower; // 原闭环控制器所设定的功率
    float chassisRawPower = 0.f; // 未经过功率控制的原始底盘功率

    std::vector<float> relPower; // 根据电机数据拟合出的实际功率
    float chassisRealPower =
            0.f; // 根据模型算出的实际输出功率（与超电反馈功率比较反映模型拟合程度）
    float capFeedbackPower = 0.f; // 实际输出功率

    std::vector<float> setIq;    // 最终设定输出电流
    std::vector<float> setPower; // 功率控制后所得的功率
    float powerRatio = 1.f;

    float capCmdRatio = 0.9f;
    float capRealRatio = 1.f;

#if EXTENSION_SUPERCAP
    CAP cap_{ &HCAN1 };
#endif

private:
    ChassisType_e chassisType_;
    PositionalPid energyPid{ 0.1f, 0, 0, 0.002f, 0, 0, 0 };
    PositionalPid powerPid{ 300.f, 0, 0, 0.002f, 0, 400.f, 0 };

    ErrorCode_e errorState_ = ErrorCode_e::NO_ERROR;
};
