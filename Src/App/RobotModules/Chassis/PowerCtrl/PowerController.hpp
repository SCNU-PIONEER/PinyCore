#pragma once
#include <cstdint>
#include <vector>
#include "MotorManager.hpp"
#include "IMotor.hpp"
#include "PidBasic.hpp"
#include <memory>
#include "SuperCap.hpp"
#include "RLS.hpp"
#include "MotorCommonMacros.hpp"

#define USEConfidenceLevel 0
#define USELinearityRatio  1

struct MotorParam_s {
    float k0;           //扭矩常数
    float MLC;          //Mechanical loss coefficient （机械损耗系数）
    float ESR;          //Equivalent Series Resistance（等效串联电阻）
    float LeakagePower; //静态功耗
};

enum class ChassisType_e : uint8_t {
    QUADRICYCLE = 4u,
    WHEELLEG = 6u,
    SWERVE = 8u,
    BANNED = 1u,
};

class PowerController {
public:
    PowerController(ChassisType_e _chassisType);

    virtual ~PowerController() = default;

    virtual void relPowerCalc() = 0;

    virtual void cmdPowerCalc(float *_motorSpeed) = 0;

    virtual void currentCalc() = 0;

    virtual std::vector<float> powerCtrl(float *_motorSpeed) = 0;

    virtual void rlsUpdate() = 0;

    void refereeDataUpdate();

protected:
    ChassisType_e chassisType_;
    uint8_t motorNum_; //电机数量

    float limitPower = 0.f; // 最大输入功率（裁判系统读取）
    float maxPower = 0.f;   // 允许最大输出功率（经过动态规划）
    float offsetPower = 0.f;
    float powerBuffer = 60.f;    // 实际缓冲能量值,从裁判系统读取,亦可软件设定
    float expPowerBuffer = 60.f; // 期望缓冲能量值

    std::vector<float> cmdPower; // 原闭环控制器所设定的功率
    float chassisRawPower = 0.f; // 未经过功率控制的原始底盘功率

    std::vector<float> relPower; // 根据电机数据拟合出的实际功率
    float chassisRealPower =
            0.f; // 根据模型算出的实际输出功率（与超电反馈功率比较反映模型拟合程度）
    float capFeedbackPower =
            0.f; // 根据超电输出电流和输入电压算出的实际输出功率

    std::vector<float> setIq;    // 最终设定输出电流
    std::vector<float> setPower; // 最终均分后所得的功率
    float powerRatio = 1.f;      // 功率分配比例

    float capCmdRatio = 0.9f; // 期望电容剩余能量百分比
    float capRealRatio = 1.f; // 实际电容剩余能量百分比

    CAP cap_;
    bool capEnable_ = 1;
    bool capFeedforwardEnable_ = 1;
    float chargeCmdPower = 0.f; //期望电容充电功率

    std::unique_ptr<positonalPid> energyPid_ = nullptr;
    std::unique_ptr<positonalPid> powerPid_ = nullptr;
};
