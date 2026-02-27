#include "VT13.hpp"
#include <cstring>
#include "Crc.hpp"

VT13::VT13(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event, uint32_t _eventBit)
        : RemoteControl(_huart, 2 * FRAME_LENGTH, _event)
{
    // Should be executed after MX_USARTx_UART_Init()
    uart_.recvDmaMultiBufInit((uint32_t *)buf_, 2 * FRAME_LENGTH);
    uart_.registerCallback([this, _eventBit](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
        callBackFromISR(_huart, _dataLength, _eventBit);
    });
    rxLostCnt_ = RX_LOST_MAX;
}

VT13::~VT13()
{
    Dma::instance().ram_free(buf_);
    uart_.unregisterCallback();
}

void VT13::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit)
{
    (void)_pos;
    uint16_t size = _huart->RxXferCount;
    // NOLINTBEGIN(readability-redundant-casting)
    if (((((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            xEventGroupSetBitsFromISR(event_, _eventBit, nullptr);
        }
    } else {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            xEventGroupSetBitsFromISR(event_, _eventBit, nullptr);
        }
    }
    __HAL_DMA_ENABLE(_huart->hdmarx);
    // NOLINTEND(readability-redundant-casting)
}

void VT13::parse()
{
    raw_ = reinterpret_cast<RcRawData_s *>(buf_);
    if (buf_[0] == 0xA9 && buf_[1] == 0x53 && Verify_CRC16_Check_Sum(buf_, FRAME_LENGTH)) {
        data_.gamepad.ch[0] = static_cast<int16_t>(raw_->ch0 - CH_VALUE_OFFSET);
        data_.gamepad.ch[1] = static_cast<int16_t>(raw_->ch1 - CH_VALUE_OFFSET);
        data_.gamepad.ch[2] = static_cast<int16_t>(raw_->ch2 - CH_VALUE_OFFSET);
        data_.gamepad.ch[3] = static_cast<int16_t>(raw_->ch3 - CH_VALUE_OFFSET);
        data_.gamepad.sw = static_cast<Sw_e>(raw_->sw);
        data_.gamepad.pause = raw_->pause;
        data_.gamepad.fn1 = raw_->fn1;
        data_.gamepad.fn2 = raw_->fn2;
        data_.gamepad.wheel = static_cast<int16_t>(raw_->wheel - CH_VALUE_OFFSET);
        data_.gamepad.trigger = raw_->trigger;
        data_.KBM.mouseX = raw_->mouseX;
        data_.KBM.mouseY = raw_->mouseY;
        data_.KBM.mouseZ = raw_->mouseZ;
        data_.KBM.mouseLeft = raw_->mouseLeft;
        data_.KBM.mouseRight = raw_->mouseRight;
        data_.KBM.mouseMiddle = raw_->mouseMiddle;
        data_.KBM.key = raw_->key;
        rxLostCnt_ = 0;
    }
}

bool VT13::isOnline()
{
    if (rxLostCnt_ < RX_LOST_MAX) {
        rxLostCnt_ = rxLostCnt_ + 1;
        return true;
    } else
        return false;
}

void *VT13::getData() { return &data_; }
