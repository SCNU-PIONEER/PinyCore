#pragma once
#include "Omni.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace CHASSIS {

class TankState : public FSMState<FSMState_e> {
    static constexpr float ACC = 0.5f;

public:
    TankState(Omni *_chassis)
            : FSMState<FSMState_e>(FSMState_e::TANK), chassis_(_chassis)
    {
    }

    void enter() final
    {
        chassis_->enter();
        LOG::info("ChassisRun", " enter");
    }

    void run() final
    {
        Speed_u ref;
        ref.vx = std::fmax(chassis_->getSpeed().vx - ACC,
                           std::fmin(chassis_->msg.vx,
                                     chassis_->getSpeed().vx + ACC));

        ref.vy = std::fmax(chassis_->getSpeed().vy - ACC,
                           std::fmin(chassis_->msg.vy,
                                     chassis_->getSpeed().vy + ACC));
        ref.wz = 0;
        chassis_->ctrl(ref);
    }

    void exit() final { LOG::info("ChassisRun", " exit"); }


    FSMState_e checkChange() final
    {
        if (chassis_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (chassis_->msg.state == FSMState_e::IDLE)
            return FSMState_e::IDLE;
        else if (chassis_->msg.state == FSMState_e::TANK)
            return FSMState_e::TANK;
        else if (chassis_->msg.state == FSMState_e::SPIN)
            return FSMState_e::SPIN;
        else
            return FSMState_e::TANK;
    }

private:
    Omni *chassis_;
};
} // namespace CHASSIS
