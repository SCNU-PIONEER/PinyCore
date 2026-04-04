#pragma once

#include "Arm.hpp"
#include "ArmMotor.hpp"
#include "StmLog.hpp"


namespace ARM {


class TeachState : public FSMState<FSMState_e> {
public:
    TeachState(Arm &_arm) : FSMState(FSMState_e::TEACH), arm_(_arm) {};

    void enter() override
    {
        arm_.orebin.init();
        arm_.hand.enable();
        arm_.videoCtrl.init();
        arm_.target_joints.joints = arm_.msg.target;

        LOG::info("Teach", "enter");
    }

    bool change() override
    {
        if (arm_.msg.state == FSMState_e::STOP) {
            arm_.isEnabled = false; //change中stop
            return true;
        } else if (arm_.msg.state == FSMState_e::NORMAL) {
            return true;
        }

        arm_.target_joints.joints = arm_.msg.target;
        return arm_.motors.homing(arm_.target_joints.joints);
    }

    void run() override
    {
        arm_.orebin.loop(arm_.msg);
        arm_.videoCtrl.loop(arm_.motors.currentJoints[1], arm_.msg.videoYaw);

        for (uint8_t i = 0; i < 6; i++) {
            arm_.target_joints.joints[i] = arm_.msg.target[i];
        }

        arm_.motors.safety.setYawPitchAngLimitAbs(arm_.target_joints.joints, arm_.msg.target);

        if (arm_.msg.switch1 == ClampState_e::CLAMP) {
            arm_.hand.clamp();
        } else if (arm_.msg.switch1 == ClampState_e::RELEASE) {
            arm_.hand.release();
        }

        arm_.motors.safety.setSpeed(0.8);
        arm_.motors.ctrl(arm_.target_joints.joints);
    }

    void exit() override { LOG::info("Teach", "exit"); }

    FSMState_e checkChange() override
    {
        if (arm_.msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (arm_.msg.state == FSMState_e::NORMAL)
            return FSMState_e::NORMAL;
        else if (arm_.msg.state == FSMState_e::PLAN)
            return FSMState_e::PLAN;
        return FSMState_e::TEACH;
    }

private:
    Arm &arm_;
};

} // namespace ARM
