#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

class ChassisRunState : public FSMState {
public:
    ChassisRunState(const Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName("ChassisRun");
    };

    void enter() override
    {
        //TODO: check the motor is offline

        LOG::info("run", " enter");
    }

    void run() override
    {
        chassis_->wheel_->update();
        if (chassis_->msg_.state == State_e::run) {
            ChassisState_s refState{ .v_x = chassis_->msg_.vx,
                                     .v_y = chassis_->msg_.vy,
                                     .w_z = chassis_->msg_.wz };
            chassis_->wheel_->ctrl(refState);
        }
    }

    void exit() override { LOG::info("run", " exit"); }

    std::string checkChange() override
    {
        if (chassis_->msg_.state == State_e::stop)
            return "ChassisStop";
        else if (chassis_->msg_.state == State_e::run)
            return "ChassisRun";
        else
            return "";
    }

private:
    const Chassis *chassis_;
};
