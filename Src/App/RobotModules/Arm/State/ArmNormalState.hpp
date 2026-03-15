#pragma once

#include "Arm.hpp"
#include "ArmMotor.hpp"
#include "StmLog.hpp"

namespace ARM {

class NormalState : public FSMState<FSMState_e> {
public:
    NormalState(Arm &_arm) : FSMState(FSMState_e::NORMAL), arm_(_arm) {};

    void enter() override
    {
        arm_.orebin.init();
        arm_.hand.enable();
        arm_.target_joints = arm_.motors.currentJoints;

        LOG::info("Normal", "enter");
    }

    bool change() override
    {
        if (arm_.msg.state == FSMState_e::STOP) {
            arm_.isEnabled = false; //change中stop
            return true;
        }
        if (!arm_.isEnabled) {
            arm_.motors.enable();
            arm_.isEnabled = true;
            LOG::info("Normal", "motor enabled, start homing");
            return false;
        }
        return arm_.motors.init();
    }

    void run() override
    {
        arm_.orebin.loop(arm_.msg);

        arm_.motors.handRollPitch(arm_.middle_joints, arm_.msg.target);
        for (uint8_t i = 0; i < 8; ++i) {
            if ((i < 4) || (i >= 6 && i < 8)) {
                arm_.middle_joints[i] = arm_.msg.target[i];
            }
        }

        for (uint8_t i = 0; i < 2; i++) {
            arm_.target_joints.joints[i] += arm_.middle_joints[i];
        }
        if (arm_.msg.mode == ARM::ArmMode_e::LEFT) {
            //左臂 j3,4,5,6
            for (uint8_t i = 3; i < 7; i++) {
                arm_.target_joints.joints[i] += arm_.middle_joints[i];
            }
        } else if (arm_.msg.mode == ARM::ArmMode_e::RIGHT) {
            //右臂 j7,8,9,10
            for (uint8_t i = 7; i < 10; i++) {
                arm_.target_joints.joints[i] += arm_.middle_joints[i];
            }
        } else if (arm_.msg.mode == ARM::ArmMode_e::TWO) {
            //双臂 j1,2,3,4,5,6,7,8,9,10
            for (uint8_t i = 2; i < 10; i++) {
                arm_.target_joints.joints[i] += arm_.middle_joints[i];
            }
        }
        if (arm_.msg.switch1) {
            arm_.hand.clamp();
        } else {
            arm_.hand.release();
        }
        arm_.motors.safety.setSpeed(1);
        arm_.motors.ctrl(arm_.middle_joints);
    }

    void exit() override { LOG::info("Normal", "exit"); }

    FSMState_e checkChange() override
    {
        if (arm_.msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (arm_.msg.state == FSMState_e::NORMAL)
            return FSMState_e::NORMAL;
        return FSMState_e::STOP;
    }

private:
    Arm &arm_;
};

} // namespace ARM
