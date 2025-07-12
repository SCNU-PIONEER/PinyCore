#pragma once

#include "BspBase.hpp"
#include "Soc.hpp"
#include HAL_INCLUDE

class Uart : public BspBase<Uart> {
public:
    using callback = std::function<void(UART_HandleTypeDef *, uint16_t)>;
    /**
    * @brief uart registerCallback
    */
    void registerCallback(UART_HandleTypeDef *_huart, callback _pCallback);

    /**
    * @brief uart multi_DMA_rx_buf init
    *
    * @param DataLength 请开辟两倍的缓冲区
    */
    HAL_StatusTypeDef RecvDmaMultiBufInit(UART_HandleTypeDef *_huart,
                                          uint32_t *_dstAddress,
                                          uint32_t _dataLength);

    /**
    * @brief uart multi_DMA_rx_buf init
    */
    HAL_StatusTypeDef RecvDmaInit(UART_HandleTypeDef *_huart,
                                  uint32_t *_dstAddress, uint32_t _dataLength);

    /**
    * @brief uart rx callbackFromISR
    */
    void callbackFromISR(UART_HandleTypeDef *_huart, uint16_t _size);

private:
    std::unordered_map<UART_HandleTypeDef *, callback> cbTable;
};
