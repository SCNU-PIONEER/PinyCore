#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "stm_log.hpp"

class RunState : public FSMState {
public:
    RunState(const Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName("run");
    };

    void enter() override
    {
        //TODO: check the motor is offline

        LOG::info("run", " enter");
    }

    void run() override
    {
        if (chassis_->msg_.state == State_e::run) {
            LOG::info("run", " run");
        }
    }

    void exit() override { LOG::info("run", " exit"); }

    std::string checkChange() override { return "stop"; }

private:
    const Chassis *chassis_;
};
