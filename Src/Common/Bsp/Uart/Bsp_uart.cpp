#include "Bsp_uart.hpp"
#include <vector>
#include <cstdint>

static std::vector<std::pair<UART_HandleTypeDef *, Uart::callback> > cbTable;


Uart::Uart(UART_HandleTypeDef *_huart) : huart_(_huart)
{
    /* reserve 4 firstly */
    cbTable.reserve(4);
}

Uart::Uart(UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_dma)
        : huart_(_huart), hdma_(_dma)
{
    cbTable.reserve(4);
}

void Uart::registerCallback(callback _pCallback)
{
    for (const auto &pair : cbTable) {
        if (pair.first == huart_) {
            return;
        }
    }
    cbTable.emplace_back(huart_, _pCallback);
}

HAL_StatusTypeDef Uart::recvDmaMultiBufInit(uint32_t *_dstAddress,
                                            uint32_t _dataLength)
{
    HAL_StatusTypeDef result = HAL_OK;
    huart_->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart_->RxEventType = HAL_UART_RXEVENT_IDLE;
    huart_->RxXferSize = _dataLength;
    SET_BIT(huart_->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    uint32_t *secondMemAddress = _dstAddress + (_dataLength / 2);
    result = HAL_DMAEx_MultiBufferStart(
            huart_->hdmarx, (uint32_t)&SOC_UART_DMA_DR, (uint32_t)_dstAddress,
            (uint32_t)secondMemAddress, _dataLength);

    return result;
}

HAL_StatusTypeDef Uart::recvDmaInit(uint32_t *_dstAddress, uint32_t _dataLength)
{
    HAL_StatusTypeDef result = HAL_OK;
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    result = HAL_UARTEx_ReceiveToIdle_DMA(huart_, (uint8_t *)&_dstAddress,
                                          _dataLength);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
    return result;
}

HAL_StatusTypeDef Uart::transmit(const uint8_t *_pData, uint16_t _size)
{
    return HAL_UART_Transmit(huart_, _pData, _size, 0);
}

HAL_StatusTypeDef Uart::transmitDma(const uint8_t *_pData, uint16_t _size)
{
    return HAL_UART_Transmit_DMA(huart_, _pData, _size);
}

HAL_StatusTypeDef Uart::receive(uint8_t *_pData, uint16_t _size)
{
    return HAL_UART_Receive(huart_, _pData, _size, 0);
}

HAL_StatusTypeDef Uart::receiveDma(uint8_t *_pData, uint16_t _size)
{
    return HAL_UARTEx_ReceiveToIdle_DMA(huart_, _pData, _size);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *_huart, uint16_t _size)
{
    for (const auto &pair : cbTable) {
        if (pair.first == _huart) {
            pair.second(_huart, _size);
            break;
        }
    }
}
