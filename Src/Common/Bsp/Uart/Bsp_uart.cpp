#include "Bsp_uart.hpp"
#include <stdlib.h>

void Uart::registerCallback(UART_HandleTypeDef *_huart, callback _pCallback)
{
    cbTable[_huart] = _pCallback;
}

HAL_StatusTypeDef Uart::RecvDmaMultiBufInit(UART_HandleTypeDef *_huart,
                                            uint32_t *_dstAddress,
                                            uint32_t _dataLength)
{
    HAL_StatusTypeDef result = HAL_OK;
    _huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    _huart->RxEventType = HAL_UART_RXEVENT_IDLE;
    _huart->RxXferSize = _dataLength;
    SET_BIT(_huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);
    uint32_t *secondMemAddress = _dstAddress + (_dataLength / 2);
    result = HAL_DMAEx_MultiBufferStart(
            _huart->hdmarx, (uint32_t)&SOC_UART_DMA_DR, (uint32_t)_dstAddress,
            (uint32_t)secondMemAddress, _dataLength);

    return result;
}

HAL_StatusTypeDef Uart::RecvDmaInit(UART_HandleTypeDef *_huart,
                                    uint32_t *_dstAddress, uint32_t _dataLength)
{
    HAL_StatusTypeDef result = HAL_OK;
    __HAL_UART_ENABLE_IT(_huart, UART_IT_IDLE);
    result = HAL_UARTEx_ReceiveToIdle_DMA(_huart, (uint8_t *)&_dstAddress,
                                          _dataLength);
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
    return result;
}

void Uart::callbackFromISR(UART_HandleTypeDef *_huart, uint16_t _size)
{
    auto it = cbTable.find(_huart);
    if (it != cbTable.end()) {
        it->second(_huart, _size);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Uart::instance().callbackFromISR(huart, Size);
}
