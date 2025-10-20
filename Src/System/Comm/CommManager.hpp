#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>

#include "StmLog.hpp"

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

    void registerTransmitter(std::function<void()> _tx, uint16_t _id)
    {
        if (transmitter_.contains(_id)) {
            LOG::warn("TxPacket", "NO.%d already registered", _id);
            return;
        }
        transmitter_[_id] = _tx;
    }

    void cancelTransmitter(uint16_t _id) { transmitter_.erase(_id); }

    void registerReceiver(std::function<void()> _rx, uint16_t _id)
    {
        if (receiver_.contains(_id)) {
            LOG::warn("RxPacket", "NO.%d already registered", _id);
            return;
        }
        receiver_[_id] = _rx;
    }

    void cancelReceiver(uint16_t _id) { receiver_.erase(_id); }

    void txTask()
    {
        for (auto &txPair : transmitter_) {
            txPair.second();
        }
    }

    void rxTask()
    {
        for (auto &rxPair : receiver_) {
            rxPair.second();
        }
    }

private:
    CommManager() = default;

    std::unordered_map<uint16_t, std::function<void()> > transmitter_;
    std::unordered_map<uint16_t, std::function<void()> > receiver_;
};
