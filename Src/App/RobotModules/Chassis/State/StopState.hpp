#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "stm_log.hpp"

class StopState : public FSMState {
public:
    StopState(Chassis *_chassis) : chassis_(_chassis) { setStateName("stop"); };

    void enter() override { LOG::info("stop", " enter"); }

    void run() override
    {
        // TODO: shutdown motors
        LOG::info("stop", " run");
    }

    void exit() override { LOG::info("stop", " exit"); }

    std::string checkChange() override
    {
        if (chassis_->msg.state == State_e::stop)
            return "stop";
        else
            return "";
    }

private:
    Chassis *chassis_;
};
