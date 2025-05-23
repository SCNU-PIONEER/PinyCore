#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "stm_log.hpp"

class StopState : public FSMState {
public:
    StopState(Chassis *_chassis) : chassis_(_chassis) { setStateName("stop"); };

    void enter() override
    {
        chassis_->wheel_->stop();
        LOG::info("stop", " enter");
    }

    void run() override
    {
        chassis_->wheel_->stop();
        LOG::info("stop", " run");
    }

    void exit() override { LOG::info("stop", " exit"); }


    std::string checkChange() override
    {
        if (chassis_->msg_.state == State_e::stop)
            return "stop";
        else if (chassis_->msg_.state == State_e::run)
            return "run";
        else
            return "";
    }

private:
    Chassis *chassis_;
};
