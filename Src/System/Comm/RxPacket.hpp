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
        start();
    }

    virtual ~RxPacket()
    {
        stop();
        vQueueDelete(queue_);
    };

    /// @brief Register a callback to be called when a packet is received
    /// @note This function should be implemented in the derived class
    virtual void registerCallback() = 0; // TODO: better protocol abstraction

    /// @brief Receive a packet from the queue
    /// @return true if a packet was received, false otherwise
    bool receive()
    {
        if (xQueueReceive(this->queue_, &this->rxBuf_, 0) == pdTRUE) {
            data_ = PacketType::decompress(this->rxBuf_);
            rxFreq_ = Dwt::instance().getFreq(&recvCnt_);
            return true;
        } else
            return false;
    }

    /// @brief Start the packet receiver
    /// @note This function should be called in the constructor
    void start()
    {
        CommManager::instance().registerReceiver([this] { receive(); }, uid());
    }

    /// @brief Stop the packet receiver
    /// @note This function should be called in the destructor
    void stop() { CommManager::instance().cancelReceiver(uid()); }

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
