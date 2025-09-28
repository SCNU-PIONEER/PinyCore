#pragma once

#include "Soc.hpp"
#include HAL_INCLUDE

class Uart {
public:
    Uart(UART_HandleTypeDef *_huart);

    Uart(UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_dmaHandle);

    /**
    * @brief uart registerCallback
    */
    using callback = std::function<void(UART_HandleTypeDef *, uint16_t)>;
    void registerCallback(callback _pCallback);

    /**
    * @brief uart multi_DMA_rx_buf init
    *
    * @param DataLength 请开辟两倍的缓冲区
    */
    HAL_StatusTypeDef recvDmaMultiBufInit(uint32_t *_dstAddress,
                                          uint32_t _dataLength);

    /**
    * @brief uart multi_DMA_rx_buf init
    */
    HAL_StatusTypeDef recvDmaInit(uint32_t *_dstAddress, uint32_t _dataLength);

    /**
    * @brief uart rx callbackFromISR
    */
    void callbackFromISR(uint16_t _size);

    /**
    * @brief uart tx
    */
    HAL_StatusTypeDef transmit(const uint8_t *_pData, uint16_t _size);

    /**
    * @brief uart tx dma
    */
    HAL_StatusTypeDef transmitDma(const uint8_t *_pData, uint16_t _size);

    /**
    * @brief uart rx
    */
    HAL_StatusTypeDef receive(uint8_t *_pData, uint16_t _size);

    /**
    * @brief uart rx
    */
    HAL_StatusTypeDef receiveDma(uint8_t *_pData, uint16_t _size);

    UART_HandleTypeDef *huart_;
    DMA_HandleTypeDef *hdma_ = nullptr;
};
