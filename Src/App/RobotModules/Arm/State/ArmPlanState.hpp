#pragma once

#include "Arm.hpp"
#include "StmLog.hpp"
#include <cstdint>

namespace ARM {

class PlanState : public FSMState<FSMState_e> {
public:
    PlanState(Arm &_arm) : FSMState(FSMState_e::PLAN), arm_(_arm) {};

    void enter() override
    {
        if (arm_.msg.oneClick == OneClick_e::STORE) {
            LOG::info("Plan", "SORE");
            arm_.setTargetPose(storageRoute);
        } else if (arm_.msg.oneClick == OneClick_e::TAKEOUT) {
            LOG::info("Plan", "TAKEOUT");
            arm_.setTargetPose(takeOutRoute);
        }
        LOG::info("Plan", "enter");
    }

    void run() override
    {
        arm_.orebin.loop(arm_.msg);
        if (arm_.msg.switch1 == ClampState_e::CLAMP) {
            arm_.hand.clamp();
        } else if (arm_.msg.switch1 == ClampState_e::RELEASE) {
            arm_.hand.release();
        }
        arm_.motors.safety.setSpeed(0.8f);
        arm_.moveRoute();
    }

    void exit() override { LOG::info("Plan", "exit"); }

    FSMState_e checkChange() override
    {
        if (arm_.msg.state == FSMState_e::STOP) {
            return FSMState_e::STOP;
        } else if (arm_.msg.state == FSMState_e::NORMAL) {
            return FSMState_e::NORMAL;
        } else if (arm_.msg.state == FSMState_e::TEACH) {
            return FSMState_e::TEACH;
        } else if (arm_.msg.state == FSMState_e::PLAN) {
            return FSMState_e::PLAN;
        }
        return FSMState_e::STOP;
    }

private:
    Arm &arm_;
};

} // namespace ARM
