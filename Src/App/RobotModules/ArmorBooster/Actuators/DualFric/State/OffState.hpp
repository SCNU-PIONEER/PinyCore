#pragma once

#include "DualFric.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace ARMORBOOSTER {
class OffState : public FSMState<FSMState_e> {
public:
    OffState(DualFric *_armorBooster)
            : FSMState<FSMState_e>(FSMState_e::OFF), booster_(_armorBooster)
    {
    }

    void enter() final { LOG::info("ArmorBoosterOff", " enter"); }

    void run() final { booster_->off(); }

    void exit() final { LOG::info("ArmorBoosterOff", " exit"); }

    FSMState_e checkChange() final
    {
        if (booster_->msg.state == FSMState_e ::ON)
            return FSMState_e::ON;
        else
            return FSMState_e::OFF;
    }

private:
    DualFric *booster_;
};
} // namespace ARMORBOOSTER
