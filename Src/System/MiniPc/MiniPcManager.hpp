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

    struct TransmitterInfo_s {
        std::function<void()> txFunc;
        uint32_t intervalMs;
        TickType_t lastTxTime;
    };

public:
    PcManager() = default;

    static PcManager &instance()
    {
        static PcManager instance;
        return instance;
    }

    PcManager(const PcManager &) = delete;
    PcManager &operator=(const PcManager &) = delete;

    void registerTransmitter(std::function<void()> _tx, uint32_t _intervalMs = 100)
    {
        if (txIndex_ < MAX_TX_NUM) {
            transmitter_.push_back({ _tx, _intervalMs, xTaskGetTickCount() });
            txIndex_++;
        }
    }

    void setTxInterval(uint8_t _index, uint32_t _intervalMs)
    {
        if (_index < txIndex_) {
            transmitter_[_index].intervalMs = _intervalMs;
        }
    }

    void txTask()
    {
        TickType_t now = xTaskGetTickCount();
        bool txExecuted = false; // 标志位，跟踪当前帧是否已执行发送

        for (uint8_t i = 0; i < txIndex_; i++) {
            TickType_t elapsed = now - transmitter_[i].lastTxTime;

            // 只有当未执行发送且时间间隔满足时才执行发送
            if (!txExecuted && elapsed >= pdMS_TO_TICKS(transmitter_[i].intervalMs)) {
                transmitter_[i].txFunc();
                transmitter_[i].lastTxTime = now;
                txExecuted = true; // 标记已执行发送
            }
        }
    }

private:
    std::vector<TransmitterInfo_s> transmitter_;
    uint8_t txIndex_ = 0;
};
} // namespace MINIPC
