#pragma once
#include "Dial.hpp"
#include "FSMState.hpp"
#include "StmLog.hpp"

namespace ARMORLOADER {

class BlockState : public FSMState<FSMState_e> {
    static constexpr float BLOCK_TORQ = 2.f;

    static constexpr uint16_t REVERSE_CNT = 100;

    static constexpr uint16_t RESTORE_CNT_MAX = 300;

public:
    BlockState(Dial *_armorLoader) : FSMState<FSMState_e>(FSMState_e::BLOCK), loader_(_armorLoader) {}

    void enter() final
    {
        loader_->enter();
        dirCnt_ = 0;
        restoreCnt_ = 0;
        LOG::info("ArmorLoaderBlock", " enter");
    }

    void run() final
    {
        if (std::fabs(loader_->dial()->data().curr) > Dial::BLOCK_CURRENT &&
            std::fabs(loader_->dial()->vel()) < Dial::BLOCK_SPEED_MIN) {
            restoreCnt_ = restoreCnt_ - 1 > 0 ? restoreCnt_ - 1 : 0;
            if (dirCnt_++ % REVERSE_CNT == 0) {
                dir_ = -dir_;
            }
        } else {
            restoreCnt_ = restoreCnt_ < RESTORE_CNT_MAX ? restoreCnt_ + 1 : RESTORE_CNT_MAX;
        }

        loader_->ctrlMotor(dir_ * BLOCK_TORQ);
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
        if (restoreCnt_ < RESTORE_CNT_MAX)
            return FSMState_e::BLOCK;
        else
            return loader_->lastState;
    }

private:
    Dial *loader_;

    uint16_t restoreCnt_ = 1;
    float dir_ = (Dial::DIAL_REVERSE ? 1.f : -1.f);
    uint32_t dirCnt_ = 0;
};
} // namespace ARMORLOADER
