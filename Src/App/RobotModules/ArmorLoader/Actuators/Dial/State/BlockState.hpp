#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace ARMORLOADER {

class BlockState : public FSMState<FSMState_e> {
    static constexpr float BLOCK_TORQ = 0.8f;

    static constexpr uint32_t REVERSE_CNT = 500.f;

public:
    BlockState(Dial *_armorLoader)
            : FSMState<FSMState_e>(FSMState_e::BLOCK), loader_(_armorLoader)
    {
    }

    void enter() final
    {
        loader_->enter();
        blockCnt_ = 1;
        LOG::info("ArmorLoaderBlock", " enter");
    }

    void run() final
    {
        if (loader_->isBlock())
            blockCnt_ += 3;
        else
            blockCnt_++;

        if (blockCnt_ % REVERSE_CNT == 0) {
            dir_ = -dir_;
        }
        loader_->ctrl(dir_ * BLOCK_TORQ);
    }

    void exit() final
    {
        loader_->lastState = FSMState_e::BLOCK;
        LOG::info("ArmorLoaderBlock", " exit");
    }


    FSMState_e checkChange() final
    {
        if (loader_->msg.state == FSMState_e::STOP)
            return FSMState_e ::STOP;
        if (loader_->isBlock())
            return FSMState_e::BLOCK;
        else
            return loader_->lastState;
    }

private:
    Dial *loader_;

    uint32_t blockCnt_ = 1;
    float dir_ = 1.f;
};
} // namespace ARMORLOADER
