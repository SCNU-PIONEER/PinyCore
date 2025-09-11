#pragma once

#include "CommManager.hpp"

#include "FreeRTOS.h"
#include "queue.h"
#include "DWT.hpp"

namespace COMM {

template <typename PacketType, int BufferSize> class RxPacket {
protected:
    using Data = typename PacketType::Data_u;
    using ProtoData = typename PacketType::ProtoData_s;

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
            data_ = PacketType::decompress(this->rxBuf_);
            rxFreq_ = Dwt::instance().getFreq(&recvCnt_);
        }
    }

    const ProtoData &data() const { return data_; }
    uint16_t uid() const { return PacketType::ID; }

protected:
    QueueHandle_t queue_;

    Data rxBuf_{};

    ProtoData data_{};

private:
    uint32_t recvCnt_ = 0;
    float rxFreq_ = 0.f;
};

} // namespace COMM
