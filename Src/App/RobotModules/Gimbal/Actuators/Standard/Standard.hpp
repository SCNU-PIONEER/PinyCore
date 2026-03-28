/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-18 00:12:48
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\Standard.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "Gimbal.hpp"
#include "IIR.hpp"
#include "LPF.hpp"
#include "Vision.hpp"

namespace GIMBAL {

union Motors_u {
    struct {
        PINYMOTOR::IMotor *yaw;
        PINYMOTOR::IMotor *trig;
    };
    PINYMOTOR::IMotor *_[2];
};

union States_u {
    struct {
        float pos;
        float vel;
    };
    float _[2];
};

class Standard : public Gimbal {
public:
    static constexpr bool YAW_REVERSE = false;
    Standard();//构造函数
    virtual ~Standard() {}
    
    PINYMOTOR::IMotor *yaw() const { return motors_.yaw; };
    PINYMOTOR::IMotor *trig() const { return motors_.trig; };

    void stop() final;
    void enter() final;
    void updateSelf() final;

    float cmd_yawPos = 0.f;//角度制
    float yaw_zeroAng = 0.f;//编码值
    float cmd_trigPos = 0.f;
    float trig_zeroCirNum = 0.f;//圈数

    States_u yawState() const { return yawState_; }
    States_u trigState() const { return trigState_; }

    void yawDualAngCtrl();
    void yawSingRpmCtrl(float _expRpm);
    void trigSingRpmCtrl(float _expRpm);
    void trigDualAngCtrl();
    void getyZeroPos();
    void gettZeroPos();
    Vision vision_{};

private:
    Motors_u motors_;
    States_u yawState_;
    States_u trigState_;

    // FILTER::IIR3 iirFilter[2];
    FILTER::LPF torqLpf[2]{ { 1000.f, 1.f }, { 1000.f, 5.f } };

    MotorCtrl_s YawMotorCtrl;
    MotorCtrl_s TrigMotorCtrl;
};

} // namespace GIMBAL
