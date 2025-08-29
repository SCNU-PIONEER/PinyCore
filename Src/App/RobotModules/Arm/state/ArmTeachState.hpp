/* 
 * @file   ArmTeachState.hpp
 * #@brief 机械臂示教状态，通过自定义控制器控制机械臂
 */

#pragma once

#include "Arm.hpp"
#include "ArmMotor.hpp"
#include "StmLog.hpp"

namespace ARM {

class ArmTeachState : public FSMState {
public:
    ArmTeachState(Arm &_arm) : arm_(_arm) {};
    void enter() override
    {
        LOG::info("Teach", "enter");
        Joint7D goal = { arm_.msg_.j1, arm_.msg_.j2, arm_.msg_.j3, arm_.msg_.j4,
                         arm_.msg_.j5, arm_.msg_.j6, arm_.msg_.j7 };

        if (!arm_.motors.checkGoal(goal)) {
            LOG::error("Teach", "enter: init goal out of range");
            this->setMode(FSMMode_e::PAUSE);
            return;
        }
        Arm::JointState_e initState = Arm::JointState_e::MOVING_STATE;
        while (initState == Arm::JointState_e::MOVING_STATE) {
            initState = arm_.moveOneGoal(goal);
        }

        LOG::info("Teach", "enter: init positioning finished");
        this->setMode(FSMMode_e::NORMAL);
    }
    void run() override
    {
        if (arm_.msg_.source == ControlSource_e::TP) {
            Joint7D target = { arm_.msg_.j1, arm_.msg_.j2, arm_.msg_.j3,
                               arm_.msg_.j4, arm_.msg_.j5, arm_.msg_.j6,
                               arm_.msg_.j7 };

            arm_.target_joints = { arm_.msg_.j1, arm_.msg_.j2, arm_.msg_.j3,
                                   arm_.msg_.j4, arm_.msg_.j5, arm_.msg_.j6,
                                   arm_.msg_.j7 };
            arm_.safety.setSpeed(0.5);
            arm_.safety.setAllAngleLimit(arm_.target_joints);
            arm_.motors.ctrl(arm_.target_joints); //output

        } else {
            // 来源不符时，停止运动（安全处理）
            arm_.motors.stop();
        }
    }

    void exit() override { LOG::info("Teach", "exit"); }

    uint8_t checkChange() override
    {
        // 根据消息状态和来源判断是否切换状态
        if (arm_.msg_.state == State_e::STOP) {
            return static_cast<uint8_t>(FSMState_e::STOP);
        }
        // 若指令来源为遥控器且状态正常，切换到正常状态
        else if (arm_.msg_.state == State_e::NORMAL &&
                 arm_.msg_.source == ControlSource_e::RC) {
            return static_cast<uint8_t>(FSMState_e::NORMAL);
        }
        // 其他情况保持示教状态
        else if (arm_.msg_.state == State_e::TEACH) {
            return static_cast<uint8_t>(FSMState_e::TEACH);
        }
        // 切换到规划状态
        else if (arm_.msg_.state == State_e::PLAN) {
            return static_cast<uint8_t>(FSMState_e::PLAN);
        }
        return 0;
    }

private:
    Arm &arm_;
};

} // namespace ARM
