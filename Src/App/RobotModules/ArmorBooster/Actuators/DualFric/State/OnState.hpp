#pragma once

#include "DualFric.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace ARMORBOOSTER {
class OnState : public FSMState<FSMState_e> {
public:
    OnState(DualFric *_armorBooster)
            : FSMState<FSMState_e>(FSMState_e::ON), booster_(_armorBooster)
    {
    }

    void enter() final
    {
        booster_->enter();
        LOG::info("ArmorBoosterOn", " enter");
    }

    void run() final { booster_->on(); }

    void exit() final { LOG::info("ArmorBoosterOn", " exit"); }

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
