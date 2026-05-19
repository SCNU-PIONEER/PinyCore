#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

#include "PidBasic.hpp"

static PositionalPid singleDialAngPID(25.f, 10.f, 0.f, 0.001f, 8.f, 41.f, 0);
static PositionalPid singleDialVelPID(0.2f, 0.1f, 0.0009f, 0.001f, 4.5f, 1.f, 0);

namespace ARMORLOADER {

class SingleState : public FSMState<FSMState_e> {
    struct Pids_s {
        PID *dialAngPid;
        PID *dialVelPid;
    };

    enum class SingleFSMState_e : uint8_t { BOOST, FINISH };

    static constexpr float SINGLE_ANG = 2.f * std::numbers::pi_v<float> / Dial::BULLET_NUM_ROUND;

    static constexpr float DEADBAND = 0.01f;

public:
    SingleState(Dial *_armorLoader) : FSMState<FSMState_e>(FSMState_e::SINGLE), loader_(_armorLoader)
    {
        singleFSMFactory_.addState(SingleFSMState_e::BOOST, std::make_unique<BoostState>(*this));
        singleFSMFactory_.addState(SingleFSMState_e::FINISH, std::make_unique<FinishState>(*this));
    }

    void enter() final
    {
        loader_->enter();
        singleFSMFactory_.init(singleFSMFactory_.getNextState(SingleFSMState_e::FINISH));
        LOG::info("ArmorLoaderSingle", " enter");
    }

    void run() final { singleFSMFactory_.update(); }

    void exit() final
    {
        singleDialAngPID.reset();
        singleDialVelPID.reset();
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
            else if (loader_->msg.state == FSMState_e::SINGLE)
                return FSMState_e::SINGLE;
            else
                return FSMState_e::AUTO;
        }
    }

private:
    Dial *loader_;

    StateFactory<SingleFSMState_e> singleFSMFactory_;

    // nested state - shoot
    class BoostState : public FSMState<SingleFSMState_e> {
    public:
        explicit BoostState(SingleState &_parent)
                : FSMState<SingleFSMState_e>(SingleFSMState_e::BOOST), parent_(_parent)
        {
            pids_.dialAngPid = &singleDialAngPID;
            pids_.dialVelPid = &singleDialVelPID;
        }

        void enter() final
        {
            parent_.loader_->enter();
            fixAng_ = parent_.loader_->dial()->pos() + SINGLE_ANG;
            LOG::info("", "fire");
        }

        void run() final { parent_.loader_->ctrlSingle(pids_.dialAngPid, pids_.dialVelPid, fixAng_); }

        void exit() final
        {
            pids_.dialAngPid->reset();
            pids_.dialVelPid->reset();
        }

        SingleFSMState_e checkChange() final
        {
            if (isShootFinish())
                return SingleFSMState_e::FINISH;
            else
                return SingleFSMState_e::BOOST;
        }

    private:
        SingleState &parent_;
        Pids_s pids_;
        float fixAng_;

        bool isShootFinish() const
        {
            const float &ang = parent_.loader_->dial()->pos();
            return ang > fixAng_ - DEADBAND && ang < fixAng_ + DEADBAND;
        }
    };

    // nested state - finish shoot
    class FinishState : public FSMState<SingleFSMState_e> {
    public:
        explicit FinishState(SingleState &_parent)
                : FSMState<SingleFSMState_e>(SingleFSMState_e::FINISH), parent_(_parent)
        {
        }
        void enter() final
        {
            parent_.loader_->ctrlMotor(0.f);
            LOG::info("", "done");
        }
        void run() final {}
        void exit() final {}
        SingleFSMState_e checkChange() final
        {
            if (parent_.loader_->msg.fire)
                return SingleFSMState_e::BOOST;
            else
                return SingleFSMState_e::FINISH;
        }

    private:
        SingleState &parent_;
    };
};
} // namespace ARMORLOADER