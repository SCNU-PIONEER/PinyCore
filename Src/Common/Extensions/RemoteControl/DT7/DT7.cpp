#include "Soc.hpp"
#include <cstring>
#include "DT7.hpp"
#include "task.h"


Rc::Rc(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event)
        : RemoteControl(_huart, 2 * FRAME_LENGTH, _event)
{
    uart_.recvDmaMultiBufInit((uint32_t *)buf_, 2 * FRAME_LENGTH);
    uart_.registerCallback(
            [this](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
                callBackFromISR(_huart, _dataLength);
            });
    rxLostCnt_ = RC_RX_LOST_MAX;
}

Rc::~Rc()
{
    Dma::instance().ram_free(buf_);
    uart_.unregisterCallback();
}

void Rc::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos)
{
    uint16_t size = _huart->RxXferCount;
    // NOLINTBEGIN(readability-redundant-casting)
    if (((((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR) &
         DMA_SxCR_CT) == RESET) {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * RC_FRAME_LENGTH);
        if (size == RC_FRAME_LENGTH) {
            xEventGroupSetBitsFromISR(event_, RC_READY_EVENT, nullptr);
        }
    } else {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * RC_FRAME_LENGTH);
        if (size == RC_FRAME_LENGTH) {
            xEventGroupSetBitsFromISR(event_, RC_READY_EVENT, nullptr);
        }
    }
    __HAL_DMA_ENABLE(_huart->hdmarx);
    // NOLINTEND(readability-redundant-casting)
}

void Rc::parse()
{
    data_.rc.ch0 = ((int16_t)buf_[0] | ((int16_t)buf_[1] << 8)) & 0x07FF;
    if (data_.rc.ch0 > RC_CH_VALUE_MAX || data_.rc.ch0 < RC_CH_VALUE_MIN)
        return;
    data_.rc.ch0 -= RC_CH_VALUE_OFFSET;

    data_.rc.ch1 =
            (int16_t)(((int16_t)buf_[1] >> 3) | ((int16_t)buf_[2] << 5)) &
            0x07FF;
    if (data_.rc.ch1 > RC_CH_VALUE_MAX || data_.rc.ch1 < RC_CH_VALUE_MIN)
        return;
    data_.rc.ch1 -= RC_CH_VALUE_OFFSET;

    data_.rc.ch2 = (int16_t)(((int16_t)buf_[2] >> 6) | ((int16_t)buf_[3] << 2) |
                             ((int16_t)buf_[4] << 10)) &
                   0x07FF;
    if (data_.rc.ch2 > RC_CH_VALUE_MAX || data_.rc.ch2 < RC_CH_VALUE_MIN)
        return;
    data_.rc.ch2 -= RC_CH_VALUE_OFFSET;

    data_.rc.ch3 =
            (int16_t)(((int16_t)buf_[4] >> 1) | ((int16_t)buf_[5] << 7)) &
            0x07FF;
    if (data_.rc.ch3 > RC_CH_VALUE_MAX || data_.rc.ch3 < RC_CH_VALUE_MIN)
        return;
    data_.rc.ch3 -= RC_CH_VALUE_OFFSET;

    data_.rc.switchLeft = (uint8_t)((uint8_t)(buf_[5] >> 4) & 0x0C) >> 2;
    data_.rc.switchRight = (uint8_t)((uint8_t)(buf_[5] >> 4) & 0x03);

    data_.mouse.x = ((int16_t)buf_[6]) | ((int16_t)buf_[7] << 8);
    data_.mouse.y = ((int16_t)buf_[8]) | ((int16_t)buf_[9] << 8);
    data_.mouse.z = ((int16_t)buf_[10]) | ((int16_t)buf_[11] << 8);

    data_.mouse.pressLeft = buf_[12];
    data_.mouse.pressRight = buf_[13];

    data_.keyboard.keyCode = (((uint16_t)buf_[14]) | ((uint16_t)buf_[15] << 8));

    data_.wheel = ((int16_t)buf_[16]) | ((int16_t)buf_[17] << 8);

    rxLostCnt_ = 0;
}

bool Rc::isOnline()
{
    if (rxLostCnt_ < RC_RX_LOST_MAX) {
        rxLostCnt_ = rxLostCnt_ + 1;
        return true;
    } else
        return false;
}

void *Rc::getData() { return &data_; }
