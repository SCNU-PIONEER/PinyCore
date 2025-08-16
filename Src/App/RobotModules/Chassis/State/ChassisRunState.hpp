#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "RcMsg.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class RunState : public FSMState {
public:
    RunState(Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::RUN));
    };

    void enter() final
    {
        //TODO: check the motor is offline
        chassis_->wheel->enter();
        LOG::info("run", " enter");
    }

    void run() final
    {
        chassis_->wheel->update();

        chassis_->deltaYaw = 0;
        chassis_->convertCoordinate();
        if (chassis_->msg.state == State_e::run) {
            Speed_u refState{ .v_x = chassis_->msg.vx,
                              .v_y = chassis_->msg.vy,
                              .w_z = chassis_->msg.wz };
            chassis_->wheel->ctrl(refState);
        }
    }

    void exit() final { LOG::info("run", " exit"); }

    uint8_t checkChange() final
    {
        if (chassis_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == State_e::run)
            //return static_cast<uint8_t>(FSMState_e::RUN);
            if (chassis_->rcData.rc.switchRight == RC_SW_MID) {
                return static_cast<uint8_t>(FSMState_e::FOLLOW);
            } else if (chassis_->rcData.rc.switchRight == RC_SW_UP) {
                return static_cast<uint8_t>(FSMState_e::SPIN);
            } else {
                return 0;
            }
        else
            return 0;
    }

private:
    Chassis *chassis_;
};

} //namespace CHASSIS
