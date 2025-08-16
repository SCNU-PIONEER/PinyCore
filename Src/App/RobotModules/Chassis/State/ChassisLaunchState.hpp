#pragma once
#include "Chassis.hpp"
#include "FSMState.hpp"
#include "RcMsg.hpp"
#include "StmLog.hpp"

namespace CHASSIS {
//感觉好像不太需要，云台发来run就已经缓启动成功了
class LaunchState : public FSMState {
public:
    LaunchState(Chassis *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::LAUNCH));
    }

    void enter() final
    {
        //TODO: check the motor is offline
        chassis_->wheel->enter();
        LOG::info("ChassisLaunch", " enter");
    }

    void run() final
    {
        chassis_->wheel->update();

        chassis_->wheel->stop();
    }

    void exit() final { LOG::info("ChassisLaunch", " exit"); };


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);

        else if (chassis_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Chassis *chassis_;
};

} //namespace CHASSIS