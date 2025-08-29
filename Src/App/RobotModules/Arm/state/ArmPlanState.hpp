/* 
 * @file   ArmPlanState.hpp
 * #@brief 机械臂路径规划状态，负责机械臂的一键规划功能
 */

#pragma once

#include "Arm.hpp"
#include "ArmKinematic.hpp"

namespace ARM {

class PlanState : public FSMState {
public:
    PlanState(Arm &_arm) : arm_(_arm) {};
    void enter() override { LOG::info("Plan", "enter"); }

    void run() override
    {
        arm_.setTargetPose(silverLeftRoute); // 设置目标路径
        arm_.moveRoute();                    // 执行路径规划
    }

    void exit() override { LOG::info("Plan", "exit"); }

    uint8_t checkChange() override
    {
        if (arm_.msg_.state == State_e::STOP) {
            return static_cast<uint8_t>(FSMState_e::STOP);
        } else if (arm_.msg_.state == State_e::NORMAL &&
                   arm_.msg_.source == ControlSource_e::RC) {
            return static_cast<uint8_t>(FSMState_e::NORMAL);
        } else if (arm_.msg_.state == State_e::TEACH) {
            return static_cast<uint8_t>(FSMState_e::TEACH);
        }
        return static_cast<uint8_t>(FSMState_e::PLAN);
    }

private:
    Arm &arm_;
};

} // namespace ARM
