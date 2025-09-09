#pragma once

#include "CommManager.hpp"

#include "cmsis_os.h"
#include "queue.h"

namespace COMM {

template <typename PacketType, int BufferSize> class RxPacket {
protected:
    using Data = typename PacketType::Data_u;
    using ProtoData = typename PacketType::ProtoData_s;

public:
    RxPacket() : queue_(xQueueCreate(BufferSize, sizeof(PacketType)))
    {
        initalize();
    }

    virtual ~RxPacket() = default;
    virtual void registerCallback() = 0; // TODO: better protocol abstraction

    void receive()
    {
        if (xQueueReceive(this->queue_, &this->rxBuf_, 0) == pdTRUE) {
            recvCnt_++;
            data_ = PacketType::decompress(this->rxBuf_);
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

    const ProtoData &data() const { return data_; }
    uint16_t uid() const { return PacketType::ID; }

protected:
    QueueHandle_t queue_;

    Data rxBuf_{};

    ProtoData data_{};

private:
    void initalize()
    {
        registerCallback();
        CommManager::instance().registerReceiver([this] { receive(); });
    }

    uint32_t lastRecvTick_ = 0;
    uint16_t recvCnt_ = 0;
    float rxFreq_ = 0.f;
};

} // namespace COMM
