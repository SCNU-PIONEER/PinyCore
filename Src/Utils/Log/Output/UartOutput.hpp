#pragma once

#include "Bsp_uart.hpp"
#include "sdkconfig.h"
#include "main.h"
#include "Output.hpp"
#include <atomic>

namespace LOG {

class UartOutput final : public Output {
public:
    static constexpr size_t DMA_BUFFER_SIZE = 256;
    static constexpr size_t DMA_BUFFER_COUNT = 4;

    bool init() final;
    bool send(const uint8_t *_data, size_t _size) final;

    void onTransmitComplete();

private:
    static constexpr uint8_t BUFFER_FREE = 0;
    static constexpr uint8_t BUFFER_READY = 1;
    static constexpr uint8_t BUFFER_SENDING = 2;
    static constexpr uint8_t NO_ACTIVE_BUFFER = DMA_BUFFER_COUNT;

    void startNextTransmission();

    Uart uart_{ &LOG_UART };
    uint8_t *dmaBuffers_[DMA_BUFFER_COUNT]{};
    uint16_t bufferLengths_[DMA_BUFFER_COUNT]{};
    std::atomic<uint8_t> bufferStates_[DMA_BUFFER_COUNT];
    std::atomic<uint8_t> writeIndex_{ 0 };
    std::atomic<uint8_t> readIndex_{ 0 };
    std::atomic<uint8_t> activeIndex_{ NO_ACTIVE_BUFFER };
    std::atomic_flag sendLock_ = ATOMIC_FLAG_INIT;
};

} // namespace LOG
