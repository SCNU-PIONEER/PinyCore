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
        for (uint8_t i = 0; i < 7; i++) {
            arm_.target_joints[i] += arm_.msg.target[i];
        }
        arm_.motors.safety.setSpeed(1);
        arm_.motors.safety.setAllAngleLimit(arm_.target_joints);
        //output
        arm_.motors.ctrl(arm_.target_joints);
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
