#include "Soc.hpp"
#include <cstring>
#include "Rc.hpp"
#include "Bsp_uart.hpp"
#include "Bsp_dma.hpp"
#include "task.h"

namespace RC {

Rc::Rc() { memset(&data_, 0, sizeof(data_)); }

void Rc::init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event)
{
    uart_ = _huart;
    event_ = _event;
    rcBuffer_ = (uint8_t *)Dma::instance().ram_alloc(2 * RC_FRAME_LENGTH);
    Uart::instance().RecvDmaMultiBufInit(_huart, (uint32_t *)&rcBuffer_[0],
                                         2 * RC_FRAME_LENGTH);
    Uart::instance().registerCallback(_huart, &Rc::rawCallBackFromISR);
}

void Rc::rawCallBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos)
{
    Rc::instance().callBackFromISR(_huart, _pos);
}

void Rc::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos)
{
    // if (_huart == uart_) {
    uint16_t size = _huart->RxXferCount;
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
}

uint8_t Rc::parseData()
{
    data_.rc.ch0 = ((int16_t)rcBuffer_[0] | ((int16_t)rcBuffer_[1] << 8)) &
                   0x07FF;
    if (data_.rc.ch0 > RC_CH_VALUE_MAX || data_.rc.ch0 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data_.rc.ch0 -= RC_CH_VALUE_OFFSET;

    data_.rc.ch1 = (int16_t)(((int16_t)rcBuffer_[1] >> 3) |
                             ((int16_t)rcBuffer_[2] << 5)) &
                   0x07FF;
    if (data_.rc.ch1 > RC_CH_VALUE_MAX || data_.rc.ch1 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data_.rc.ch1 -= RC_CH_VALUE_OFFSET;

    data_.rc.ch2 = (int16_t)(((int16_t)rcBuffer_[2] >> 6) |
                             ((int16_t)rcBuffer_[3] << 2) |
                             ((int16_t)rcBuffer_[4] << 10)) &
                   0x07FF;
    if (data_.rc.ch2 > RC_CH_VALUE_MAX || data_.rc.ch2 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data_.rc.ch2 -= RC_CH_VALUE_OFFSET;

    data_.rc.ch3 = (int16_t)(((int16_t)rcBuffer_[4] >> 1) |
                             ((int16_t)rcBuffer_[5] << 7)) &
                   0x07FF;
    if (data_.rc.ch3 > RC_CH_VALUE_MAX || data_.rc.ch3 < RC_CH_VALUE_MIN)
        return RC_CH_ERROR;
    data_.rc.ch3 -= RC_CH_VALUE_OFFSET;

    data_.rc.switchLeft = (uint8_t)((uint8_t)(rcBuffer_[5] >> 4) & 0x0C) >> 2;
    data_.rc.switchRight = (uint8_t)((uint8_t)(rcBuffer_[5] >> 4) & 0x03);

    data_.mouse.x = ((int16_t)rcBuffer_[6]) | ((int16_t)rcBuffer_[7] << 8);
    data_.mouse.y = ((int16_t)rcBuffer_[8]) | ((int16_t)rcBuffer_[9] << 8);
    data_.mouse.z = ((int16_t)rcBuffer_[10]) | ((int16_t)rcBuffer_[11] << 8);

    data_.mouse.pressLeft = rcBuffer_[12];
    data_.mouse.pressRight = rcBuffer_[13];

    data_.keyboard.keyCode =
            (((uint16_t)rcBuffer_[14]) | ((uint16_t)rcBuffer_[15] << 8));

    data_.wheel = ((int16_t)rcBuffer_[16]) | ((int16_t)rcBuffer_[17] << 8);

    dt7RxLostCnt_ = 0;

    return RC_NO_ERROR;
}

bool Rc::isOnline()
{
    if (dt7RxLostCnt_ < RC_RX_LOST_MAX) {
        dt7RxLostCnt_ = dt7RxLostCnt_ + 1;
        return true;
    } else
        return false;
}

} // namespace RC
