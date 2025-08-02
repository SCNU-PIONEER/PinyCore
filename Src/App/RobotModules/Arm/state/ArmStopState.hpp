/*
 * @File         : ArmStopState.hpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 11:17:14
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */
/* 
 * @file   ArmStopState.hpp
 * #@brief 机械臂失能状态，机械臂停止工作,应保证臂上所有电机失能
 */

#pragma once

#include "Arm.hpp"

namespace ARM {

class StopState : public FSMState {
public:
    StopState(Arm &_arm) : arm_(_arm) {};

    void enter() override 
    { 
        arm_.motors.stop();
        LOG::info("Stop", "enter"); }

    void run() override {arm_.motors.stop();}

    void exit() override { LOG::info("Stop", "exit"); }

    uint8_t checkChange() override {
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
