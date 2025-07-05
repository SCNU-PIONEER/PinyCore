#include "Vofa.hpp"
#include <string.h>
#include "Bsp_dma.hpp"


void Vofa::init(UART_HandleTypeDef *_uartHandle,
                DMA_HandleTypeDef *_dmaTxHandle)
{
    uartHandle_ = _uartHandle;
    dmaTxHandle_ = _dmaTxHandle;
    FWBuffer_ = (uint8_t *)Dma::instance().ram_alloc(128);
    // JFBuffer_ = (uint8_t *)Dma::instance().ram_alloc(128);
}

void Vofa::FireWater(const char *format, ...)
{
    uint32_t n;
    va_list args;
    va_start(args, format);
    n = vsnprintf((char *)FWBuffer_, 128, format, args);
    HAL_UART_Transmit_DMA(uartHandle_, (uint8_t *)FWBuffer_, n);
    va_end(args);
}

// void Vofa::JustFloat(float *_data, uint8_t _num)
// {
//     memset(JFBuffer_, 0, 256);
//     const uint8_t endMarker[] = { 0x00, 0x00, 0x80, 0x7F };
//     for (uint8_t i = 0; i < _num; i++) {
//         memcpy(&JFBuffer_[i * 4], &_data[i], sizeof(float));
//     }
//     memcpy(&JFBuffer_[_num * 4], endMarker, 4);
//     HAL_UART_Transmit_DMA(uartHandle_, (uint8_t *)JFBuffer_, ((_num + 1) * 4));
// }
// void Vofa::JustFloat(float *_data, uint8_t _num)
// {
//     uint8_t JFBuffer_[64] = { 0 };
//     constexpr std::array<uint32_t, 4> temp_end = { 0, 0, 0x80, 0x7f };
//     std::copy(_data, _data + _num, reinterpret_cast<float *>(JFBuffer_));
//     std::copy(temp_end.begin(), temp_end.end(),
//               reinterpret_cast<uint32_t *>(JFBuffer_ + _num * sizeof(float)));
//     HAL_UART_Transmit_DMA(uartHandle_, JFBuffer_, (_num + 1) * sizeof(float));
// }
