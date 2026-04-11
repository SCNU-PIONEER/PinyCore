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
        arm_.hand.enable();
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
        arm_.orebin.loop(arm_.msg);

        // arm_.videoCtrl.loop(arm_.motors.currentJoints[1], arm_.msg.videoYaw); //  TODO:
        arm_.videoCtrl.loop(arm_.msg.videoPitch, arm_.msg.videoYaw);


        for (uint8_t i = 0; i < 4; i++) {
            arm_.target_joints.joints[i] += arm_.msg.target[i];
        }

        if (arm_.msg.switch1 == ClampState_e::CLAMP) {
            arm_.hand.clamp();
        } else if (arm_.msg.switch1 == ClampState_e::RELEASE) {
            arm_.hand.release();
        }

        arm_.motors.safety.setSpeed(0.8f);
        arm_.motors.safety.setYawPitchAngLimit(arm_.target_joints.joints, arm_.msg.target);
        arm_.motors.ctrl(arm_.target_joints.joints);
    }

    void exit() override { LOG::info("Normal", "exit"); }

    FSMState_e checkChange() override
    {
        if (arm_.msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (arm_.msg.state == FSMState_e::NORMAL)
            return FSMState_e::NORMAL;
        else if (arm_.msg.state == FSMState_e::TEACH)
            return FSMState_e::TEACH;
        else if (arm_.msg.state == FSMState_e::PLAN)
            return FSMState_e::PLAN;
        return FSMState_e::STOP;
    }

private:
    Arm &arm_;
};

} // namespace ARM
