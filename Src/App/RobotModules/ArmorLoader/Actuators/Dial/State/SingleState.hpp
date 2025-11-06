#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "PidBasic.hpp"

static PositionalPid singleDialAngPID(0, 0, 0, 0.001f, 0, 0, 0);
static PositionalPid singleDialVelPID(0, 0, 0, 0.001f, 0, 0, 0);

namespace ARMORLOADER {

class SingleState : public FSMState<FSMState_e> {
    union Pids_u {
        struct {
            PID *dialAngPid;
            PID *dialVelPid;
        };
        PID *_[2];
    };

    enum class SingleFSMState_e : uint8_t { RUN, STOP };

    static constexpr float SINGLE_ANG = 1.f;

    static constexpr float DEADBAND = 0.1f;

public:
    SingleState(Dial *_armorLoader)
            : FSMState<FSMState_e>(FSMState_e::SINGLE), loader_(_armorLoader)
    {
        runState_ = std::make_unique<RunState>(*this);
        stopState_ = std::make_unique<StopState>(*this);
        singleFSMFactory_.addState(SingleFSMState_e::RUN, std::move(runState_));
        singleFSMFactory_.addState(SingleFSMState_e::STOP,
                                   std::move(stopState_));
    }

    void enter() final
    {
        loader_->enter();
        LOG::info("ArmorLoaderSingle", " enter");
    }

    void run() final { singleFSMFactory_.update(); }

    void exit() final
    {
        runState_->exit();
        loader_->lastState = FSMState_e::SINGLE;
        LOG::info("ArmorLoaderSingle", " exit");
    }


    FSMState_e checkChange() final
    {
        if (loader_->msg.state == FSMState_e::STOP)
            return FSMState_e::STOP;
        else {
            if (loader_->isBlock())
                return FSMState_e::BLOCK;
            else
                return FSMState_e::SINGLE;
        }
    }

private:
    Dial *loader_;

    StateFactory<SingleFSMState_e> singleFSMFactory_;

    // nested state - shoot
    class RunState : public FSMState<SingleFSMState_e> {
    public:
        explicit RunState(SingleState &_parent)
                : FSMState<SingleFSMState_e>(SingleFSMState_e::RUN)
                , parent_(_parent)
        {
            pids_.dialAngPid = &singleDialAngPID;
            pids_.dialVelPid = &singleDialVelPID;
        }

        void enter() final
        {
            fixAng_ = parent_.loader_->dial()->pos() + SINGLE_ANG;
        }

        void run() final
        {
            parent_.loader_->ctrlMulti(pids_.dialAngPid, pids_.dialVelPid,
                                       fixAng_ + SINGLE_ANG);
        }

        void exit() final
        {
            for (auto i : pids_._)
                i->reset();
        }

        SingleFSMState_e checkChange() final
        {
            if (isShootFinish())
                return SingleFSMState_e::STOP;
            else
                return SingleFSMState_e::RUN;
        }

    private:
        SingleState &parent_;
        Pids_u pids_;
        float fixAng_;

        bool isShootFinish() const
        {
            const float &ang = parent_.loader_->dial()->pos();
            return ang > fixAng_ - DEADBAND && ang < fixAng_ + DEADBAND;
        }
    };

    // nested state - finish shoot
    class StopState : public FSMState<SingleFSMState_e> {
    public:
        explicit StopState(SingleState &_parent)
                : FSMState<SingleFSMState_e>(SingleFSMState_e::RUN)
                , parent_(_parent)
        {
        }
        void enter() final {}
        void run() final {}
        void exit() final {}
        SingleFSMState_e checkChange() final
        {
            if (parent_.loader_->msg.fire)
                return SingleFSMState_e::RUN;
            else
                return SingleFSMState_e::STOP;
        }

    private:
        SingleState &parent_;
    };

    std::unique_ptr<RunState> runState_;
    std::unique_ptr<StopState> stopState_;
};
} // namespace ARMORLOADER
