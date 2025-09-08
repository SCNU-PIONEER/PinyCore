#pragma once

#include "CommManager.hpp"

#include "cmsis_os.h"
#include "queue.h"

namespace COMM {

template <typename PacketType, int BufferSize> class RxPacket {
public:
    RxPacket() : queue_(xQueueCreate(BufferSize, sizeof(PacketType)))
    {
        CommManager::instance().registerReceiver([this] { receive(); });
    }

    virtual ~RxPacket() = default;
    virtual void registerCallback() = 0; // TODO: better protocol abstraction

    void receive()
    {
        if (xQueueReceive(this->queue_, &this->rxBuf_, 0) == pdTRUE) {
            recvCnt_++;
            memcpy(&this->packet_, &this->rxBuf_, sizeof(PacketType));
        }
    }

    void updateRxFreq()
    {
        uint32_t dt = xTaskGetTickCount() - lastRecvTick_; // ms
        if (dt < 1000) {
            return;
        } else {
            this->rxFreq_ = static_cast<float>(this->recvCnt_) /
                            (static_cast<float>(dt) / 1000.f);
            this->recvCnt_ = 0;
            lastRecvTick_ = xTaskGetTickCount();
        }
    }

    const PacketType &packet() const { return packet_; }

private:
    uint32_t lastRecvTick_ = 0;
    uint16_t recvCnt_ = 0;
    float rxFreq_ = 0.f;

    QueueHandle_t queue_;
    PacketType rxBuf_{}; // clone buffer
    PacketType packet_{};
};

} // namespace COMM
