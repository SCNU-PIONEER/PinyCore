
#include "Crc.hpp"
#include "Referee.hpp"
#include <cstring>

namespace REFEREE {

Referee::Referee(UART_HandleTypeDef *_huart, const EventGroupHandle_t &_event, uint32_t _eventBit)
        : receiver(_huart, _event, _eventBit), transmitter(_huart)
{
}

Receiver::Receiver(UART_HandleTypeDef *_huart, const EventGroupHandle_t &_event, uint32_t _eventBit)
        : uart_(_huart), rxBuffer_((uint8_t *)Dma::instance().ram_alloc(REFEREE_RX_BUFFER_LEN))
{
    // Should be executed after MX_USARTx_UART_Init()
    uart_.recvDmaInit(rxBuffer_, REFEREE_RX_BUFFER_LEN);
    uart_.registerCallback([this, _event, _eventBit](UART_HandleTypeDef *_huart, uint16_t _size) {
        this->uartIdleCallback(_huart, _size, _event, _eventBit);
    });
}

Receiver::~Receiver()
{
    Dma::instance().ram_free(rxBuffer_);
    uart_.unregisterCallback();
}

void Receiver::uartIdleCallback(UART_HandleTypeDef *_huart, uint16_t _size, const EventGroupHandle_t &_event,
                                uint32_t _eventBit)
{
    if (_huart->Instance != uart_.huart_->Instance)
        return;
    pendingSize = _size;

    xEventGroupSetBitsFromISR(_event, _eventBit, nullptr);
}

void Receiver::readRefereeData()
{
    uint16_t frameHeaderPos, nextPos, frameLen;
    for (frameHeaderPos = 0; frameHeaderPos < pendingSize; frameHeaderPos = nextPos) {
        while (frameHeaderPos < pendingSize && rxBuffer_[frameHeaderPos] != SOF)
            frameHeaderPos++;

        const FrameHeader_s *header = (FrameHeader_s *)&rxBuffer_[frameHeaderPos];
        frameLen = sizeof(FrameHeader_s) + LEN_CMDID + header->dataLen + LEN_TAIL;

        if (header->dataLen > 128 || (!Verify_CRC8_Check_Sum(&rxBuffer_[frameHeaderPos], sizeof(FrameHeader_s))) ||
            (!Verify_CRC16_Check_Sum(&rxBuffer_[frameHeaderPos], frameLen))) {
            nextPos = frameHeaderPos + 1;
            continue;
        } else
            nextPos = frameHeaderPos + frameLen;

        uint16_t cmdId = *((uint16_t *)&rxBuffer_[frameHeaderPos + sizeof(FrameHeader_s)]);
        for (auto i : INFO) {
            if (static_cast<CmdId_e>(cmdId) == i.cmdId) {
                rxCnt_++;
                void *dataPtr = reinterpret_cast<uint8_t *>(&refereeData_) + i.offsetByte;
                memcpy(dataPtr, &rxBuffer_[frameHeaderPos + sizeof(FrameHeader_s) + LEN_CMDID], i.size);
            }
        }
    }
    memset(rxBuffer_, 0, REFEREE_RX_BUFFER_LEN);
    uart_.receiveDma(rxBuffer_, REFEREE_RX_BUFFER_LEN);
}

void Receiver::rxFreqCalc()
{
    static uint32_t lastTick = 0;
    if ((xTaskGetTickCount() - lastTick) >= pdMS_TO_TICKS(1000)) {
        rxFreq_ = static_cast<float>(rxCnt_) / (static_cast<float>(xTaskGetTickCount() - lastTick) / 1000.f);
        rxCnt_ = 0;
        lastTick = xTaskGetTickCount();
    }
}

Transmitter::Transmitter(UART_HandleTypeDef *_huart) : uart_(_huart) {}

uint16_t Transmitter::sendData(uint16_t _cmdId, uint8_t *_data, uint16_t _dataLen)
{
    uint16_t totalSize;
    FrameHeader_s txHeader;

    if (_dataLen + sizeof(FrameHeader_s) + 4 > REFEREE_TX_BUFFER_LEN)
        return 0;
    memset(txBuffer_, 0, REFEREE_TX_BUFFER_LEN);

    txHeader.sof = SOF;
    txHeader.dataLen = _dataLen;
    txHeader.seq = 0;
    txHeader.crc8 = Get_CRC8_Check_Sum((uint8_t *)&txHeader, sizeof(FrameHeader_s) - 1, 0xff);

    memcpy(&txBuffer_, &txHeader, sizeof(FrameHeader_s));
    *(uint16_t *)&txBuffer_[sizeof(FrameHeader_s)] = _cmdId;
    if (_data != nullptr && _dataLen > 0)
        memcpy(&txBuffer_[sizeof(FrameHeader_s) + LEN_CMDID], _data, _dataLen);

    totalSize = sizeof(FrameHeader_s) + LEN_CMDID + _dataLen + LEN_TAIL;
    Append_CRC16_Check_Sum(txBuffer_, totalSize);

    uart_.transmitDma(txBuffer_, totalSize);
    return totalSize;
}

} // namespace REFEREE
