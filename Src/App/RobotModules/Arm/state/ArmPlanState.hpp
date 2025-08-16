/* 
 * @file   ArmPlanState.hpp
 * #@brief 机械臂路径规划状态，负责机械臂的一键规划功能
 */

#pragma once

#include "Arm.hpp"

namespace ARM {

class PlanState : public FSMState {
public:
    PlanState(Arm &_arm) : arm_(_arm) {};
    void enter() override { LOG::info("Plan", "enter"); }

    void run() override
    {
        // arm_.motors.update();
        arm_.moveRoute(); // 执行路径规划
    }

    void exit() override { LOG::info("Plan", "exit"); }

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
