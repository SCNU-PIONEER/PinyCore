#pragma once

#include "StmLog.hpp"
#include <cstdint>
#include <functional>
#include "BSP.hpp"

namespace MINIPC {

class PcManager {
    static constexpr int MAX_TX_NUM = 2;
    static constexpr int MAX_RX_NUM = 2;

public:
    PcManager() = default;

    static PcManager &instance()
    {
        static PcManager instance;
        return instance;
    }

    PcManager(const PcManager &) = delete;
    PcManager &operator=(const PcManager &) = delete;

    void registerTransmitter( // NOLINTNEXTLINE
            std::function<void()> _tx)
    {
        if (txIndex_ < MAX_TX_NUM) {
            transmitter_[txIndex_++] = _tx;
        }
    }

    void txTask()
    {
        static uint8_t i = 0;
        i < txIndex_ ? transmitter_[i](), i++ : i = 0;
    }

private:
    // NOLINTNEXTLINE
    std::function<void()> transmitter_[MAX_TX_NUM];
    uint8_t txIndex_ = 0;
};
} // namespace MINIPC
