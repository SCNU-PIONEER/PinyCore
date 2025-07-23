#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class RunState : public FSMState {
public:
    RunState(const Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::RUN));
    };

    void enter() override
    {
        //TODO: check the motor is offline
        chassis_->wheel->enter();
        LOG::info("run", " enter");
    }

    void run() override
    {
        chassis_->wheel->update();
        if (chassis_->msg.state == State_e::run) {
            Speed_u refState{ .v_x = chassis_->msg.vx,
                              .v_y = chassis_->msg.vy,
                              .w_z = chassis_->msg.wz };
            chassis_->wheel->ctrl(refState);
        }
    }

    void exit() override { LOG::info("run", " exit"); }

    uint8_t checkChange() override
    {
        if (chassis_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    const Chassis *chassis_;
};

} //namespace CHASSIS
