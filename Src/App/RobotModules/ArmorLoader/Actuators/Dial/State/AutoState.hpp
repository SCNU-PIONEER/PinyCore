#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "PidBasic.hpp"
#include "MotorCommonMacros.hpp"

PositionalPid autoDialVelPID(0.5f, 0.01f, 0.001f, 0.001f, 0, 4.5f, 0);

namespace ARMORLOADER {

class AutoState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PID *dialVelPid;
        };
        PID *_[1];
    };

public:
    AutoState(Dial *_armorLoader) : FSMState<FSMState_e>(FSMState_e::AUTO), loader_(_armorLoader)
    {
        pids_.dialVelPid = &autoDialVelPID;
    }

    static constexpr float getDialVel(float _rate)
    {
        return _rate / static_cast<float>(Dial::BULLET_NUM_ROUND) * 2.f * PI;
    }

    void enter() final
    {
        loader_->enter();
        LOG::info("ArmorLoaderAuto", " enter");
    }

    void run() final
    {
        if (loader_->msg.fire)
            loader_->ctrl(pids_.dialVelPid, getDialVel(15.f));
        else
            loader_->ctrl(pids_.dialVelPid, 0.f);
    }

    void exit() final
    {
        for (auto &i : pids_._) {
            i->reset();
        }
        loader_->lastState = FSMState_e::AUTO;
        LOG::info("ArmorLoaderAuto", " exit");
    }


    FSMState_e checkChange() final
    {
        if (loader_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;

        if (loader_->isBlock())
            return FSMState_e::BLOCK;
        else if (loader_->msg.state == FSMState_e::AUTO)
            return FSMState_e::AUTO;
        else if (loader_->msg.state == FSMState_e::SINGLE)
            return FSMState_e::SINGLE;
        else
            return FSMState_e::STOP;
    }

private:
    Dial *loader_;

    Pids_u pids_;
};
} // namespace ARMORLOADER
