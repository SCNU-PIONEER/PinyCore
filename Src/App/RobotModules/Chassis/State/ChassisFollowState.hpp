#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "PidBasic.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class FollowState : public FSMState {
public:
    FollowState(Chassis *_chassis) : chassis_(_chassis)
    {
        followPosPID = std::unique_ptr<PID>(
                new PositonalPid(0.2f, 0.005f, 0.f, 4.f, 0.f, 4.f, 0.f));
        followIncPID = std::unique_ptr<PID>(
                new IncrementalPid(0.2f, 0.005f, 0.f, 4.f, 0.f));
        setStateName(static_cast<uint8_t>(FSMState_e::FOLLOW));
    }

    void enter() final
    {
        //TODO: check the motor is offline
        chassis_->wheel->enter();
        LOG::info("ChassisFollow", " enter");
    }

    void run() final
    {
        chassis_->wheel->update();

        chassis_->convertCoordinate();

        if (chassis_->msg.state == State_e::run) {
            Speed_u refState{ .v_x = chassis_->msg.vx,
                              .v_y = chassis_->msg.vy,
                              .w_z = followIncPID->calc(0,
                                                        chassis_->deltaYaw) };
            chassis_->wheel->ctrl(refState);
        }
    }

    void exit() final { LOG::info("ChassisFollow", " exit"); };


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::FOLLOW);
        else
            return 0;
    }

private:
    Chassis *chassis_;

    std::unique_ptr<PID> followPosPID;
    std::unique_ptr<PID> followIncPID;
};

} //namespace CHASSIS