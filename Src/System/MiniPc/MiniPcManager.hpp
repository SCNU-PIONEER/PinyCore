#pragma once

#include "StmLog.hpp"
#include <cstdint>
#include <functional>
#include "Bsp.hpp"
#include <vector>

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

    void registerTransmitter(std::function<void()> _tx)
    {
        if (txIndex_ < MAX_TX_NUM) {
            transmitter_.push_back(_tx);
            txIndex_++;
        }
    }

    void txTask()
    {
        static uint8_t i = 0;
        i < txIndex_ ? transmitter_[i](), i++ : i = 0;
    }

private:
    std::vector<std::function<void()> > transmitter_;
    uint8_t txIndex_ = 0;
};
} // namespace MINIPC
