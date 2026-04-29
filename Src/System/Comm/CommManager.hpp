#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>

#include "sdkconfig.h"
#include "StmLog.hpp"
#include "Singleton.hpp"

class CommManager : public Singleton<CommManager> {
public:
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
    CommManager() { static_assert(APP_USE_COMM == 1, "CommManager requires APP_USE_COMM == 1"); }
    friend class Singleton<CommManager>;

    std::unordered_map<uint16_t, std::function<void()> > transmitter_;
    std::unordered_map<uint16_t, std::function<void()> > receiver_;
};
