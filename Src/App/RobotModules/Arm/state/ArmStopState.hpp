#pragma once

#include "Arm.hpp"

namespace ARM {

class StopState : public FSMState {
public:
    StopState(Arm &_arm) : arm_(_arm) {};

    void enter() override
    {
        arm_.motors.stop();
        LOG::info("Stop", "enter");
    }

    void run() override { arm_.motors.stop(); }

    void exit() override { LOG::info("Stop", "exit"); }

    uint8_t checkChange() override
    {
        if (arm_.msg_.state == State_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (arm_.msg_.state == State_e::NORMAL)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else if (arm_.msg_.state == State_e::TEACH)
            return static_cast<uint8_t>(FSMState_e::TEACH);
        else if (arm_.msg_.state == State_e::PLAN)
            return static_cast<uint8_t>(FSMState_e::PLAN);
        else
            return 0;
    }


private:
    Arm &arm_;
};

} // namespace ARM
