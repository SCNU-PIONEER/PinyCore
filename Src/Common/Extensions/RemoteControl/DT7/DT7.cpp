#include "Soc.hpp"
#include <cstring>
#include "DT7.hpp"
#include "task.h"

DT7::DT7(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event) : RemoteControl(_huart, 2 * FRAME_LENGTH, _event)
{
    uart_.recvDmaMultiBufInit((uint32_t *)buf_, 2 * FRAME_LENGTH);
    uart_.registerCallback(
            [this](UART_HandleTypeDef *_huart, uint16_t _dataLength) { callBackFromISR(_huart, _dataLength); });
    rxLostCnt_ = RX_LOST_MAX;
}

DT7::~DT7()
{
    Dma::instance().ram_free(buf_);
    uart_.unregisterCallback();
}

void DT7::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos)
{
    uint16_t size = _huart->RxXferCount;
    // NOLINTBEGIN(readability-redundant-casting)
    if (((((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            xEventGroupSetBitsFromISR(event_, RC_READY_EVENT, nullptr);
        }
    } else {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            xEventGroupSetBitsFromISR(event_, RC_READY_EVENT, nullptr);
        }
    }
    __HAL_DMA_ENABLE(_huart->hdmarx);
    // NOLINTEND(readability-redundant-casting)
}

void DT7::parse()
{
    data_.rc.ch0 = static_cast<int16_t>((buf_[0] | (buf_[1] << 8)) & 0x07FF);

    if (data_.rc.ch0 > CH_VALUE_MAX || data_.rc.ch0 < CH_VALUE_MIN)
        return;
    data_.rc.ch0 -= CH_VALUE_OFFSET;

    data_.rc.ch1 = static_cast<int16_t>(((buf_[1] >> 3) | (buf_[2] << 5)) & 0x07FF);

    if (data_.rc.ch1 > CH_VALUE_MAX || data_.rc.ch1 < CH_VALUE_MIN)
        return;
    data_.rc.ch1 -= CH_VALUE_OFFSET;

    data_.rc.ch2 = static_cast<int16_t>(((buf_[2] >> 6) | (buf_[3] << 2) | (buf_[4] << 10)) & 0x07FF);
    if (data_.rc.ch2 > CH_VALUE_MAX || data_.rc.ch2 < CH_VALUE_MIN)
        return;
    data_.rc.ch2 -= CH_VALUE_OFFSET;

    data_.rc.ch3 = static_cast<int16_t>(((buf_[4] >> 1) | (buf_[5] << 7)) & 0x07FF);
    if (data_.rc.ch3 > CH_VALUE_MAX || data_.rc.ch3 < CH_VALUE_MIN)
        return;
    data_.rc.ch3 -= CH_VALUE_OFFSET;

    data_.rc.switchLeft = static_cast<Sw_e>((static_cast<uint8_t>(buf_[5] >> 4) & 0x0C) >> 2);
    data_.rc.switchRight = static_cast<Sw_e>(static_cast<uint8_t>(buf_[5] >> 4) & 0x03);


    data_.mouse.x = static_cast<int16_t>((buf_[6]) | (buf_[7] << 8));
    data_.mouse.y = static_cast<int16_t>((buf_[8]) | (buf_[9] << 8));
    data_.mouse.z = static_cast<int16_t>((buf_[10]) | (buf_[11] << 8));

    data_.mouse.pressLeft = buf_[12];
    data_.mouse.pressRight = buf_[13];

    data_.keyboard.keyCode = (((uint16_t)buf_[14]) | ((uint16_t)buf_[15] << 8));

    data_.wheel = static_cast<int16_t>((buf_[16]) | (buf_[17] << 8));

    rxLostCnt_ = 0;
}

void DT7::convert(RcMsg_t &_msg)
{
    _msg.rx = static_cast<float>(data_.rc.ch0) / CH_VALUE_RANGE * RC_ROCKER_MAX;
    _msg.ry = static_cast<float>(data_.rc.ch1) / CH_VALUE_RANGE * RC_ROCKER_MAX;
    _msg.lx = static_cast<float>(data_.rc.ch2) / CH_VALUE_RANGE * RC_ROCKER_MAX;
    _msg.ly = static_cast<float>(data_.rc.ch3) / CH_VALUE_RANGE * RC_ROCKER_MAX;

    switch (data_.rc.switchRight) {
    case Sw_e::UP:
        _msg.rSwitch = RcSw_e::UP;
        break;
    case Sw_e::MID:
        _msg.rSwitch = RcSw_e::MID;
        break;
    default:
        _msg.rSwitch = RcSw_e::DOWN;
        break;
    }

    switch (data_.rc.switchLeft) {
    case Sw_e::UP:
        _msg.lSwitch = RcSw_e::UP;
        break;
    case Sw_e::MID:
        _msg.lSwitch = RcSw_e::MID;
        break;
    default:
        _msg.lSwitch = RcSw_e::DOWN;
        break;
    }

    _msg.slider = static_cast<float>((data_.wheel) - 1024) / WHEEL_VALUE_RANGE * RC_SLIDER_MAX;

    _msg.lPress = data_.mouse.pressLeft;
    _msg.rPress = data_.mouse.pressRight;
    _msg.xMove = data_.mouse.x;
    _msg.yMove = data_.mouse.y;
    _msg.zRoller = data_.mouse.z;
}

bool DT7::isOnline()
{
    if (rxLostCnt_ < RX_LOST_MAX) {
        rxLostCnt_ = rxLostCnt_ + 1;
        return true;
    } else
        return false;
}

void *DT7::getData() { return &data_; }
