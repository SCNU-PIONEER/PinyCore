#pragma once

#include "CommManager.hpp"

#include "cmsis_os.h"

namespace COMM {

template <typename PacketType> class TxPacket {
protected:
    using Data = typename PacketType::Data_u;
    using ProtoData = typename PacketType::ProtoData_s;

public:
    TxPacket(float _txFreq = 100.f) : txFreq_(_txFreq)
    {
        CommManager::instance().registerTransmitter([this]() {
            if (checkSend()) {
                Data txBuf = PacketType::compress(this->data_);
                this->send(txBuf.bytes, PacketType::LEN);
            }
        });
    }

    virtual ~TxPacket() = default;
    virtual void send(uint8_t *_buf,
                      uint16_t _len) = 0; // TODO: better protocol abstraction

    void loadFull(ProtoData *_data)
    {
        memcpy(&data_, &_data, sizeof(ProtoData));
    }

    ProtoData &setData() { return data_; }

    uint16_t uid() const { return PacketType::ID; }

protected:
    ProtoData data_{};

private:
    uint32_t lastSendTick_ = 0;
    float txFreq_;
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
