
#include "Crc.hpp"
#include "Referee.hpp"
#include "Bsp_uart.hpp"
#include "Bsp_dma.hpp"
#include "stm32f4xx_hal_uart.h"
#include <cstring>

namespace REFEREE {

RefReceiver::RefReceiver()
        : uart_(nullptr), rxBuffer_(nullptr), rxLostCnt_(REFEREE_SYS_MAX_LOST)
{
}

void RefReceiver::init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event)
{
    uart_ = _huart;
    event_ = _event;

    rxBuffer_ = (uint8_t *)Dma::instance().ram_alloc(REFEREE_RX_BUFFER_LEN);

    Uart::instance().RecvDmaMultiBufInit(_huart, (uint32_t *)&rxBuffer_[0],
                                         REFEREE_RX_BUFFER_LEN);
    Uart::instance().registerCallback(_huart, [this](UART_HandleTypeDef *_huart,
                                                     uint16_t _size) {
        this->uartIdleCallback(_huart);
    });
}

void RefReceiver::uartIdleCallback(UART_HandleTypeDef *_huart)
{
    if (_huart->Instance != uart_->Instance) {
        return;
    }
    uint16_t dmaRxPos =
            REFEREE_RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(_huart->hdmarx);

    int32_t lenDif = dmaRxPos - lastPos;

    dataLen = (lenDif >= 0) ? lenDif : (REFEREE_RX_BUFFER_LEN + lenDif);

    if (lenDif < 0)
        memcpy(&rxBuffer_[REFEREE_RX_BUFFER_LEN], &rxBuffer_[0], dmaRxPos);

    xEventGroupSetBitsFromISR(event_, REFEREE_READY_EVENT, nullptr);
}

void RefReceiver::readRefereeData()
{
    uint16_t frameStartPos, nextPos, frameLen;
    uint16_t maxLen = dataLen + lastPos;

    for (frameStartPos = lastPos; frameStartPos < maxLen;
         frameStartPos = nextPos) {
        while (frameStartPos < maxLen && rxBuffer_[frameStartPos] != SOF) {
            frameStartPos++;
        }

        if (frameStartPos + sizeof(FrameHeader_s) > maxLen) {
            break;
        }

        FrameHeader_s *header = (FrameHeader_s *)&rxBuffer_[frameStartPos];

        frameLen =
                sizeof(FrameHeader_s) + LEN_CMDID + header->dataLen + LEN_TAIL;

        if (header->dataLen > 128 ||
            (!Verify_CRC8_Check_Sum((uint8_t *)header,
                                    sizeof(FrameHeader_s))) ||
            (!Verify_CRC16_Check_Sum(&rxBuffer_[frameStartPos], frameLen))) {
            nextPos = frameStartPos + 1;
            continue;
        } else {
            nextPos = frameStartPos + frameLen;
        }

        uint16_t cmdId = *(
                (uint16_t *)&rxBuffer_[frameStartPos + sizeof(FrameHeader_s)]);

        for (auto i : INFO) {
            if (static_cast<CmdId_e>(cmdId) == i.cmdId) {
                void *dataPtr = reinterpret_cast<uint8_t *>(&refereeData_) +
                                i.offsetByte;
                memcpy(dataPtr,
                       &rxBuffer_[frameStartPos + sizeof(FrameHeader_s) +
                                  LEN_CMDID],
                       i.size);
            }
        }
    }
    lastPos = frameStartPos % REFEREE_RX_BUFFER_LEN;
}


RefereeTransmitter::RefereeTransmitter(UART_HandleTypeDef *_huart)
        : uart_(_huart)
{
}

uint16_t RefereeTransmitter::sendData(uint16_t _cmdId, uint8_t *_data,
                                      uint16_t _dataLen)
{
    uint16_t totalSize;
    FrameHeader_s txHeader;

    _data[0] = 1;
    _data[1] = 2;
    _data[2] = 3;
    _data[3] = 4;

    if (_dataLen + sizeof(FrameHeader_s) + 4 > REFEREE_TX_BUFFER_LEN) {
        return 0;
    }
    memset(txBuffer_, 0, REFEREE_TX_BUFFER_LEN);

    txHeader.sof = SOF;
    txHeader.dataLen = _dataLen;
    txHeader.seq = 0;
    txHeader.crc8 = Get_CRC8_Check_Sum((uint8_t *)&txHeader,
                                       sizeof(FrameHeader_s) - 1, 0xff);

    memcpy(&txBuffer_, &txHeader, sizeof(FrameHeader_s));
    *(uint16_t *)&txBuffer_[sizeof(FrameHeader_s)] = _cmdId;
    if (_data != nullptr && _dataLen > 0)
        memcpy(&txBuffer_[sizeof(FrameHeader_s) + LEN_CMDID], _data, _dataLen);

    totalSize = sizeof(FrameHeader_s) + LEN_CMDID + _dataLen + LEN_TAIL;

    Append_CRC16_Check_Sum(txBuffer_, totalSize);

    //HAL_UART_Transmit_DMA(uart_, txBuffer_, totalSize);
    //TODO: use bsp_uart transimit function
    return totalSize;
}

void RefReceiver::incRxLost()
{
    if (rxLostCnt_ < REFEREE_SYS_MAX_LOST) {
        rxLostCnt_++;
    }
}

bool RefReceiver::isOffline() { return rxLostCnt_ >= REFEREE_SYS_MAX_LOST; }

} // namespace REFEREE