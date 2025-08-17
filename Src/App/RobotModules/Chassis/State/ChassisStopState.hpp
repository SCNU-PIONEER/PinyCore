#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class StopState : public FSMState {
public:
    StopState(Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::STOP));
    }

    void enter() final
    {
        chassis_->wheel->stop();
        LOG::info("stop", " enter");
    }

    void run() final { chassis_->wheel->stop(); }

    void exit() final { LOG::info("ChassisStop", " exit"); }


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == FSMState_e::RUN)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Chassis *chassis_;
};

} //namespace CHASSIS
