#pragma once

#include "Arm.hpp"
#include "ArmMotor.hpp"
#include "StmLog.hpp"
#include <cstdint>

namespace ARM {

class NormalState : public FSMState<FSMState_e> {
public:
    NormalState(Arm &_arm) : FSMState(FSMState_e::NORMAL), arm_(_arm) {};

    void enter() override
    {
        arm_.orebin.init();
        arm_.leftHand.enable();
        arm_.rightHand.enable();
        arm_.videoCtrl.init();
        arm_.target_joints.joints = arm_.motors.currentJoints;

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
        // arm_.orebin.loop(arm_.msg);

        arm_.videoCtrl.loop(static_cast<uint32_t>(arm_.target_joints.joints[1]), static_cast<uint32_t>(0)); //  TODO:

        arm_.motors.handRollPitch(arm_.target_joints.joints, arm_.msg.target);

        for (uint8_t i = 0; i < 2; i++) {
            arm_.target_joints.joints[i] += arm_.msg.target[i];
        }

        switch (arm_.msg.mode) {
        case ArmMode_e::LEFT:
            //左臂 j3,4,5,6
            for (uint8_t i = 6; i < 8; i++) {
                arm_.target_joints.joints[i] += arm_.msg.target[i];
            }
            if (arm_.msg.switch1 == ClampState_e::CLAMP) {
                arm_.leftHand.clamp();
            } else if (arm_.msg.switch1 == ClampState_e::RELEASE) {
                arm_.leftHand.release();
            }
            break;
        case ArmMode_e::RIGHT:
            //右臂 j7,8,9,10
            for (uint8_t i = 2; i < 4; i++) {
                arm_.target_joints.joints[i] += arm_.msg.target[i];
            }
            if (arm_.msg.switch1 == ClampState_e::CLAMP) {
                arm_.rightHand.clamp();
            } else if (arm_.msg.switch1 == ClampState_e::RELEASE) {
                arm_.rightHand.release();
            }
            break;
        case ArmMode_e::TWO:
            //双臂 j1,2,3,4,5,6,7,8,9,10
            for (uint8_t i = 2; i < 4; i++) {
                arm_.target_joints.joints[i] += arm_.msg.target[i];
            }
            for (uint8_t i = 6; i < 8; i++) {
                arm_.target_joints.joints[i] += arm_.msg.target[i];
            }
            if (arm_.msg.switch1 == ClampState_e::CLAMP) {
                arm_.leftHand.clamp();
                arm_.rightHand.clamp();
            } else if (arm_.msg.switch1 == ClampState_e::RELEASE) {
                arm_.leftHand.release();
                arm_.rightHand.release();
            }
        }

        arm_.motors.safety.setSpeed(0.7);
        arm_.motors.ctrl(arm_.target_joints.joints);
    }

    void exit() override { LOG::info("Normal", "exit"); }

    FSMState_e checkChange() override
    {
        if (arm_.msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (arm_.msg.state == FSMState_e::NORMAL)
            return FSMState_e::NORMAL;
        // else if (arm_.msg.state == FSMState_e::PLAN)
        //     return FSMState_e::PLAN;
        // else if (arm_.msg.state == FSMState_e::TEACH)
        //     return FSMState_e::TEACH;
        return FSMState_e::STOP;
    }

private:
    Arm &arm_;
};

} // namespace ARM
