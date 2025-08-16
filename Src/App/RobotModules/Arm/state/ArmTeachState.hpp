/*
 * @File         : ArmTeachState.hpp
 * @Brief        : 
 * @Version      : 
 * @Author       : 3687402504@qq.com
 * @LastEditTime : 2025-08-02 11:17:29
 * Copyright 2025 by SCNU-PIONEER (c), All Rights Reserved.
 */
/* 
 * @file   ArmTeachState.hpp
 * #@brief 机械臂示教状态，通过自定义控制器控制机械臂
 */

#pragma once

#include "Arm.hpp"

namespace ARM {

class ArmTeachState : public FSMState {
public:
    ArmTeachState(Arm &_arm) : arm_(_arm) {};
    void enter() override { LOG::info("Teach", "enter"); }

    void run() override
    {
        // arm_.motors.update();
        arm_.teach(); // 执行示教
    }

    void exit() override { LOG::info("Teach", "exit"); }

    uint8_t checkChange() override
    {
        if (arm_.msg_.state == State_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (arm_.msg_.state == State_e::NORMAL)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else if (arm_.msg_.state == State_e::TEACH)
            return static_cast<uint8_t>(FSMState_e::TEACH);
        else if (arm_.msg_.state == State_e::PLAN)
            return static_cast<uint8_t>(FSMState_e::PLAN);
        else
            return 0;
    }

private:
    Arm &arm_;
};

} // namespace ARM
