#pragma once

#include "Standard.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace GIMBAL::STANDARD {

class StopState : public FSMState {
public:
    StopState(Standard *_gimbal) : gimbal_(_gimbal)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::STOP));
    }

    void enter() final
    {
        gimbal_->stop();
        LOG::info("GimbalStop", "enter");
    }

    void run() final { gimbal_->stop(); }

    void exit() final { LOG::info("GimbalStop", "exit"); }

    uint8_t checkChange() final
    {
        if (gimbal_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (gimbal_->msg.state == FSMState_e::RUN)
            return static_cast<uint8_t>(FSMState_e::LAUNCH);
        else
            return 0;
    }

private:
    Standard *gimbal_;
};

} // namespace GIMBAL::STANDARD
