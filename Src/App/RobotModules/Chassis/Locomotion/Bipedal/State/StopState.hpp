#pragma once
#include "Bipedal.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS::BIPEDAL {
class StopState : public FSMState {
public:
    StopState(Bipedal *_chassis) : chassis_(_chassis)
    {
        setStateName(static_cast<uint8_t>(FSMState_e::STOP));
    }

    void enter() final
    {
        chassis_->stop();
        LOG::info("ChassisStop", " enter");
    }

    void run() final { chassis_->stop(); }

    void exit() final { LOG::info("ChassisStop", " exit"); }


    uint8_t checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return static_cast<uint8_t>(FSMState_e::STOP);
        else if (chassis_->msg.state != FSMState_e::STOP &&
                 chassis_->isFreeze())
            return static_cast<uint8_t>(FSMState_e::LAUNCH);
        else
            return 0;
    }

private:
    Bipedal *chassis_;
};

} // namespace CHASSIS::BIPEDAL
