#pragma once

#include <cstdint>
#include <functional>

class CommManager {
    static constexpr int MAX_TX_NUM = 3;
    static constexpr int MAX_RX_NUM = 3;

public:
    static CommManager &instance()
    {
        static CommManager instance;
        return instance;
    }

    CommManager(const CommManager &) = delete;
    CommManager &operator=(const CommManager &) = delete;


    void registerTransmitter( // NOLINTNEXTLINE
            std::function<void()> _tx)
    {
        if (txIndex_ < MAX_TX_NUM) {
            transmitter_[txIndex_++] = _tx;
        }
    }

    void registerReceiver(std::function<void()> _rx)
    {
        if (rxIndex_ < MAX_RX_NUM) {
            receiver_[rxIndex_++] = _rx;
        }
    }

    void txTask()
    {
        for (uint8_t i = 0; i < txIndex_; i++) {
            transmitter_[i]();
        }
    }

    void rxTask()
    {
        for (uint8_t i = 0; i < rxIndex_; i++) {
            receiver_[i]();
        }
    }

private:
    CommManager() = default;

    // NOLINTNEXTLINE
    std::function<void()> transmitter_[MAX_TX_NUM];
    uint8_t txIndex_ = 0;

    std::function<void()> receiver_[MAX_RX_NUM];
    uint8_t rxIndex_ = 0;
};
