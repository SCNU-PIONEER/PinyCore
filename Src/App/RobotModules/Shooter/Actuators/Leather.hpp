/*
 * @Author: IRON.NY 2537933690@qq.com
 * @Date: 2025-12-12 16:07:55
 * @LastEditors: IRON.NY 2537933690@qq.com
 * @LastEditTime: 2026-03-27 15:04:48
 * @FilePath: \PinyCore\Src\App\RobotModules\Gimbal\Actuators\Standard\Standard.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "Shooter.hpp"
#include "IIR.hpp"
#include "LPF.hpp"

namespace SHOOTER {

union Motors_u {
    struct {
        PINYMOTOR::IMotor *shooterL;
        PINYMOTOR::IMotor *shooterR;
        PINYMOTOR::IMotor *shooterT;
    };
    PINYMOTOR::IMotor *_[3];
};

union States_u {
    struct {
        float pos;
        float vel;
    };
    float _[2];
};

class Leather : public Shooter {
public:
    Leather();//构造函数
    ~Leather() {};

    PINYMOTOR::IMotor *shooterL() const { return motors_.shooterL; };
    PINYMOTOR::IMotor *shooterR() const { return motors_.shooterR; };
    PINYMOTOR::IMotor *shooterT() const { return motors_.shooterT; };

    void stop() final;
    void enter() final;
    void updateSelf() final;

    float screw_zeroCirNum = 0.f;//圈数
    void getsZeroPos();
    float cmd_screwPos = 0.f;
    float cmd_turnWheelPos = Shooter::angTurnwheel_Load[0];//弧度制

    bool alreadyLaunched = false;
    bool fireFlag = false;
    bool magFlag[4] = {false, false, false, false};
    bool servoFlag[4] = {false, false, false, false};

    States_u screwState() const { return screwState_; }
    States_u turnwheelState() const { return turnwheelState_; }
    void turnWheelCtrl();
    void screwSingRpmCtrl(float expsRpm);
    void screwDualAngCtrl();
    void shooterServoCtrl();
private:
    Motors_u motors_;
    States_u screwState_;
    States_u turnwheelState_;

    // FILTER::IIR3 iirFilter[2];
    FILTER::LPF torqLpf[2]{ { 1000.f, 1.f }, { 1000.f, 5.f } };

    MotorCtrl_s ShooterLMotorCtrl;
    MotorCtrl_s ShooterRMotorCtrl;
    MotorCtrl_s ShooterSMotorCtrl;
    MotorCtrl_s ShooterTMotorCtrl;
};

} // namespace SHOOTER
