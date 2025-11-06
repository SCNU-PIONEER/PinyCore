#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace ARMORLOADER {

class StopState : public FSMState<FSMState_e> {
public:
    StopState(Dial *_armorLoader)
            : FSMState<FSMState_e>(FSMState_e::STOP), loader_(_armorLoader)
    {
    }

    void enter() final { LOG::info("ArmorLoaderStop", " enter"); }

    void run() final { loader_->stop(); }

    void exit() final
    {
        loader_->lastState = FSMState_e::STOP;
        LOG::info("ArmorLoaderStop", " exit");
    }


    FSMState_e checkChange() final
    {
        if (loader_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else if (loader_->msg.state == FSMState_e::AUTO)
            return FSMState_e::AUTO;
        else if (loader_->msg.state == FSMState_e::SINGLE)
            return FSMState_e::SINGLE;
        else
            return FSMState_e::STOP;
    }

private:
    Dial *loader_;
};
} // namespace ARMORLOADER
