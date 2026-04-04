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
        if (arm_.msg.oneClick == OneClick_e::ORE_BIN_EXTRACT) {
            LOG::info("Plan", "ORE_BIN_EXTRACT");
            arm_.setTargetPose(test1);
        } else if (arm_.msg.oneClick == OneClick_e::HIGHWAY_CROSS) {
            LOG::info("Plan", "HIGHWAY_CROSS");
            // arm_.setTargetPose(test1);
        } else if (arm_.msg.oneClick == OneClick_e::ORE_PICK1) {
            LOG::info("Plan", "ORE_PICK1");
            // arm_.setTargetPose(test1);
        } else if (arm_.msg.oneClick == OneClick_e::ORE_PICK2) {
            LOG::info("Plan", "ORE_PICK2");
            // arm_.setTargetPose(test1);
        } else if (arm_.msg.oneClick == OneClick_e::ORE_PICK3) {
            LOG::info("Plan", "ORE_PICK3");
            // arm_.setTargetPose(test1);
        }
        LOG::info("Plan", "enter");
    }

    void run() override { arm_.moveRoute(); }

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
