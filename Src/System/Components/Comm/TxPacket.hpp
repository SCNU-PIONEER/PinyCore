#pragma once

#include "CommManager.hpp"

#include "cmsis_os.h"

namespace COMM {

template <typename PacketType> class TxPacket {
public:
    TxPacket()
    {
        CommManager::instance().registerTransmitter([this]() {
            if (checkSend())
                this->send();
        });
    }

    virtual ~TxPacket() = default;
    virtual void send() = 0; // TODO: better protocol abstraction

    void load(PacketType *_packet)
    {
        memcpy(&this->txBuf_, _packet, sizeof(PacketType));
    }

protected:
    float txFreq_ = 100.f; // default 100Hz

    PacketType txBuf_{};

private:
    uint32_t lastSendTick_ = 0;
    bool checkSend()
    {
        if ((xTaskGetTickCount() - lastSendTick_) >=
            pdMS_TO_TICKS(1000.f / this->txFreq_)) {
            this->lastSendTick_ = xTaskGetTickCount();
            return true;
        } else {
            return false;
        }
    }
};

} // namespace COMM
