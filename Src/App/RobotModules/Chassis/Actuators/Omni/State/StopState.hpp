#pragma once
#include "Omni.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS::OMNI {
class StopState : public FSMState<FSMState_e> {
public:
    StopState(Omni *_chassis)
            : FSMState<FSMState_e>(FSMState_e::STOP), chassis_(_chassis)
    {
    }

    void enter() final
    {
        chassis_->stop();
        LOG::info("ChassisStop", " enter");
    }

    void run() final { chassis_->stop(); }

    void exit() final { LOG::info("ChassisStop", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::RUN &&
                 chassis_->gimbalYawMsg.isReset)
            return FSMState_e::RUN;
        else
            return FSMState_e::STOP;
    }

private:
    Omni *chassis_;
};
} // namespace CHASSIS::OMNI
