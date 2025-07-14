#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class StopState : public FSMState {
public:
    StopState(Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(fsmState_e::STOP));
    };

    void enter() override
    {
        chassis_->wheel_->stop();
        LOG::info("stop", " enter");
    }

    void run() override { chassis_->wheel_->stop(); }

    void exit() override { LOG::info("stop", " exit"); }


    uint8_t checkChange() override
    {
        if (chassis_->msg_.state == State_e::stop)
            return static_cast<uint8_t>(fsmState_e::STOP);
        else if (chassis_->msg_.state == State_e::run)
            return static_cast<uint8_t>(fsmState_e::RUN);
        else
            return -1;
    }

private:
    Chassis *chassis_;
};

}
