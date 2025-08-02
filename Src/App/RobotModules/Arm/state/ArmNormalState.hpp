/*
 * @File         : ArmNormalState.hpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 11:16:19
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */
/* 
 * @file   armNormalState.hpp
 * #@brief 机械臂正常工作状态，机械臂在此状态下可以通过遥控器执行正常的动作
 */
#pragma once

#include "Arm.hpp"
#include "ArmMotor.hpp"

namespace ARM {

class ArmNormalState : public FSMState {
public:
    ArmNormalState(Arm &_arm) : arm_(_arm) {};

    void enter() override { LOG::info("Normal", " enter"); }

    void run() override {
        arm_.motors.update();
        if (arm_.msg_.state == State_e::run) {
            Joint7D target = { arm_.msg_.j1, arm_.msg_.j2, arm_.msg_.j3,
                                 arm_.msg_.j4, arm_.msg_.j5, arm_.msg_.j6,
                                 arm_.msg_.j7 };
            arm_.target_joints = target;
            arm_.moveOneJoint(arm_.target_joints);
        }

    }

    void exit() override { LOG::info("Normal", " exit"); }

    uint8_t checkChange() override 
    { 
        if (arm_.msg_.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (arm_.msg_.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
        }

private:
    Arm &arm_;
};

}// namespace ARM
