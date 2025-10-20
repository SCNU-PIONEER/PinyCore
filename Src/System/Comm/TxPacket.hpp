#pragma once

#include "CommManager.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace COMM {

template <typename PacketType> class TxPacket {
protected:
    using Data = typename PacketType::Data_u;
    using ProtoData = typename PacketType::ProtoData_s;

public:
    TxPacket(float _txFreq = 100.f) : txFreq_(_txFreq) { start(); }

    virtual ~TxPacket() { stop(); };

    /// @brief The driver of transmitting
    /// @details This function is called by the CommManager
    /// @param _buf The buffer to be transmitted
    /// @param _len The length of the buffer
    /// @note This function is pure virtual and must be implemented by the derived class
    virtual void
    transmit(uint8_t *_buf,
             uint16_t _len) = 0; // TODO: better protocol abstraction

    /// @brief Send the packet
    /// @details This function is called by the user
    void send()
    {
        Data txBuf = PacketType::compress(this->data_);
        this->transmit(txBuf.bytes, PacketType::LEN);
    }

    /// @brief Start the transmitter
    /// @details This function is called by the constructor or the user
    void start()
    {
        CommManager::instance().registerTransmitter(transmitFunc_, uid());
    }

    /// @brief Stop the transmitter
    /// @details This function is called by the destructor or the user
    void stop() { CommManager::instance().cancelTransmitter(uid()); }

    /// @brief Load the packet with the given data
    /// @details This function is called by the user
    /// @param _data The data to be loaded
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
        if (txFreq_ > 0.f && ((xTaskGetTickCount() - lastSendTick_) >=
                              pdMS_TO_TICKS(1000.f / this->txFreq_))) {
            this->lastSendTick_ = xTaskGetTickCount();
            return true;
        } else {
            return false;
        }
    }

    std::function<void()> transmitFunc_{ [this]() {
        if (checkSend()) {
            Data txBuf = PacketType::compress(this->data_);
            this->transmit(txBuf.bytes, PacketType::LEN);
        }
    } };
};

} // namespace COMM
