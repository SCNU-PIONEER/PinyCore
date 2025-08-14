#pragma once

#include "Gimbal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace GIMBAL {

class StopState : public FSMState {
public:
    StopState(Gimbal *_gimbal) : gimbal_(_gimbal)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::STOP));
    }

    void enter() final
    {
        gimbal_->head->stop();
        LOG::info("GimbalStop", "enter");
    }

    void run() final { gimbal_->head->stop(); }

    void exit() final { LOG::info("GimbalStop", "exit"); }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == State_e::stop)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (gimbal_->msg.state == State_e::run)
            return static_cast<uint8_t>(FSMState_e::RUN);
        else
            return 0;
    }

private:
    Gimbal *gimbal_;
};

} // namespace GIMBAL
