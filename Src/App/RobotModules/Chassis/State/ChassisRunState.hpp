#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class RunState : public FSMState {
public:
    RunState(const Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(fsmState_e::RUN));
    };

    void enter() override
    {
        //TODO: check the motor is offline
        chassis_->wheel_->enter();
        LOG::info("run", " enter");
    }

    void run() override
    {
        chassis_->wheel_->update();
        if (chassis_->msg_.state == State_e::run) {
            speed_u refState{ .v_x = chassis_->msg_.vx,
                              .v_y = chassis_->msg_.vy,
                              .w_z = chassis_->msg_.wz };
            chassis_->wheel_->ctrl(refState);
        }
    }

    void exit() override { LOG::info("run", " exit"); }

    uint8_t checkChange() override
    {
        if (chassis_->msg_.state == State_e::stop)
            return static_cast<uint8_t>(fsmState_e::STOP);
        else if (chassis_->msg_.state == State_e::run)
            return static_cast<uint8_t>(fsmState_e::RUN);
        else
            return 0;
    }

private:
    const Chassis *chassis_;
};

}
