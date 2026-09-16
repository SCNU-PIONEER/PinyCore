#include "UartOutput.hpp"
#include "Bsp_dma.hpp"
#include <cstring>

namespace LOG {

bool UartOutput::init()
{
    auto *dmaBuffer = static_cast<uint8_t *>(Dma::instance().ram_alloc(DMA_BUFFER_SIZE * DMA_BUFFER_COUNT));
    if (dmaBuffer == nullptr) {
        while (true) {
            /* 内存分配失败 */
        };
    }

    for (size_t index = 0; index < DMA_BUFFER_COUNT; ++index) {
        dmaBuffers_[index] = dmaBuffer + (index * DMA_BUFFER_SIZE);
        bufferStates_[index].store(BUFFER_FREE, std::memory_order_relaxed);
    }

    writeIndex_.store(0, std::memory_order_relaxed);
    readIndex_.store(0, std::memory_order_relaxed);
    activeIndex_.store(NO_ACTIVE_BUFFER, std::memory_order_relaxed);
    uart_.registerTxCallback([this] { onTransmitComplete(); });
    return true;
}

bool UartOutput::send(const uint8_t *_data, size_t _size)
{
    if (sendLock_.test_and_set(std::memory_order_acquire)) {
        return false;
    }

    const uint8_t index = writeIndex_.load(std::memory_order_relaxed);
    if (bufferStates_[index].load(std::memory_order_acquire) != BUFFER_FREE) {
        sendLock_.clear(std::memory_order_release);
        return false;
    }

    std::memcpy(dmaBuffers_[index], _data, _size);
    bufferLengths_[index] = static_cast<uint16_t>(_size);
    bufferStates_[index].store(BUFFER_READY, std::memory_order_release);
    writeIndex_.store(static_cast<uint8_t>((index + 1) % DMA_BUFFER_COUNT), std::memory_order_relaxed);

    startNextTransmission();

    sendLock_.clear(std::memory_order_release);
    return true;
}

void UartOutput::onTransmitComplete()
{
    const uint8_t index = activeIndex_.exchange(NO_ACTIVE_BUFFER, std::memory_order_acq_rel);
    if (index != NO_ACTIVE_BUFFER) {
        bufferStates_[index].store(BUFFER_FREE, std::memory_order_release);
        readIndex_.store(static_cast<uint8_t>((index + 1) % DMA_BUFFER_COUNT), std::memory_order_release);
    }

    startNextTransmission();
}

void UartOutput::startNextTransmission()
{
    const uint8_t index = readIndex_.load(std::memory_order_acquire);
    if (bufferStates_[index].load(std::memory_order_acquire) != BUFFER_READY) {
        return;
    }

    uint8_t expected = NO_ACTIVE_BUFFER;
    if (!activeIndex_.compare_exchange_strong(expected, index, std::memory_order_acq_rel)) {
        return;
    }

    bufferStates_[index].store(BUFFER_SENDING, std::memory_order_release);
    if (uart_.transmitDma(dmaBuffers_[index], bufferLengths_[index]) != HAL_OK) {
        bufferStates_[index].store(BUFFER_READY, std::memory_order_release);
        activeIndex_.store(NO_ACTIVE_BUFFER, std::memory_order_release);
    }
}

} // namespace LOG
