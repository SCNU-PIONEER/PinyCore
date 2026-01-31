/* 
 * @file   ArmPlanState.hpp
 * #@brief 机械臂路径规划状态，负责机械臂的一键规划功能
 */

#pragma once

#include "Arm.hpp"
#include "StmLog.hpp"
#include <cstdint>

namespace ARM {

class PlanState : public FSMState<FSMState_e> {
public:
    PlanState(Arm &_arm) : FSMState(FSMState_e::PLAN), arm_(_arm) {};

    void enter() override { LOG::info("Plan", "enter"); }

    void run() override
    {
        // arm_.setTargetPose();
        if (arm_.msg.mode == ARM::ArmMode_e::LEFT) {
            //左臂 j1，2，3,4,5,6
            for (uint8_t i = 6; i < 10; i++) {
                arm_.RouteDta.pose->joints[i] = arm_.motors.currentJoints[i];
            }
        } else if (arm_.msg.mode == ARM::ArmMode_e::RIGHT) {
            //右臂 j1，2，7,8,9,10
            for (uint8_t i = 3; i < 7; i++) {
                arm_.RouteDta.pose->joints[i] = arm_.motors.currentJoints[i];
            }
        } else if (arm_.msg.mode == ARM::ArmMode_e::TWO) {
            //双臂 j1,2,3,4,5,6,7,8,9,10
        }

        arm_.moveRoute();
    }

    void exit() override { LOG::info("Plan", "exit"); }

    FSMState_e checkChange() override
    {
        if (arm_.msg.state == FSMState_e::STOP) {
            return FSMState_e::STOP;
        } else if (arm_.msg.state == FSMState_e::NORMAL) {
            return FSMState_e::NORMAL;
        }
        return FSMState_e::STOP;
    }

private:
    Arm &arm_;
};

} // namespace ARM
