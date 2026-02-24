#include "ET08A.hpp"
#include <cstring>

ET08A::ET08A(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event, uint32_t _eventBit)
        : RemoteControl(_huart, 2 * FRAME_LENGTH, _event)
{
    // Should be executed after MX_USARTx_UART_Init()
    uart_.recvDmaMultiBufInit((uint32_t *)buf_, 2 * FRAME_LENGTH);
    uart_.registerCallback([this, _eventBit](UART_HandleTypeDef *_huart, uint16_t _dataLength) {
        callBackFromISR(_huart, _dataLength, _eventBit);
    });

    rxLostCnt_ = RX_LOST_MAX;
}

ET08A::~ET08A()
{
    Dma::instance().ram_free(buf_);
    uart_.unregisterCallback();
}

void ET08A::callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit)
{
    (void)_pos;
    uint16_t size = _huart->RxXferCount;
    // NOLINTBEGIN(readability-redundant-casting)
    if (((((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            if (buf_[0] == 0x0f) {
                xEventGroupSetBitsFromISR(event_, _eventBit, nullptr);
            }
        }
    } else {
        __HAL_DMA_DISABLE(_huart->hdmarx);
        ((DMA_Stream_TypeDef *)_huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(_huart->hdmarx, 2 * FRAME_LENGTH);
        if (size == FRAME_LENGTH) {
            if (buf_[0] == 0x0f) {
                xEventGroupSetBitsFromISR(event_, _eventBit, nullptr);
            }
        }
    }
    __HAL_DMA_ENABLE(_huart->hdmarx);
    // NOLINTEND(readability-redundant-casting)
}

void ET08A::parse()
{
    sbus_.ch[0] = ((buf_[2] << 8) + (buf_[1])) & 0x07ff;
    sbus_.ch[1] = ((buf_[3] << 5) + (buf_[2] >> 3)) & 0x07ff;
    sbus_.ch[2] = ((buf_[5] << 10) + (buf_[4] << 2) + (buf_[3] >> 6)) & 0x07ff;
    sbus_.ch[3] = ((buf_[6] << 7) + (buf_[5] >> 1)) & 0x07ff;
    sbus_.ch[4] = ((buf_[7] << 4) + (buf_[6] >> 4)) & 0x07ff;
    sbus_.ch[5] = ((buf_[9] << 9) + (buf_[8] << 1) + (buf_[7] >> 7)) & 0x07ff;
    sbus_.ch[6] = ((buf_[10] << 6) + (buf_[9] >> 2)) & 0x07ff;
    sbus_.ch[7] = ((buf_[11] << 3) + (buf_[10] >> 5)) & 0x07ff;

    data_.ch0 = (int16_t)(sbus_.ch[0] - CH_VALUE_OFFSET);
    data_.ch1 = (int16_t)(-sbus_.ch[1] + CH_VALUE_OFFSET);
    data_.ch2 = (int16_t)(sbus_.ch[3] - CH_VALUE_OFFSET);
    data_.ch3 = (int16_t)(sbus_.ch[2] - CH_VALUE_OFFSET);

    data_.switchLeft = static_cast<Sw_e>(sbus_.ch[4]);
    data_.switchRight = static_cast<Sw_e>(sbus_.ch[5]);

    data_.wheel = sbus_.ch[6];

    rxLostCnt_ = 0;
}

bool ET08A::isOnline()
{
    if (rxLostCnt_ < RX_LOST_MAX) {
        rxLostCnt_ = rxLostCnt_ + 1;
        return true;
    } else
        return false;
}

void *ET08A::getData() { return &data_; }
