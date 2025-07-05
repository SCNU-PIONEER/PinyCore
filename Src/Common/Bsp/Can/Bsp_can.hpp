#pragma once

#include "BspBase.hpp"
#include "Soc.hpp"
#include <functional>
#include <unordered_map>
#include HAL_INCLUDE

class Can : public BspBase<Can> {
public:
    using callback = std::function<void(const uint8_t *)>;
    /**
     * @brief can registerCallback
     */
    void registerCallback(canHandle *_hcan, uint32_t _stdid,
                          callback _pCallback);

    /**
     * @brief can unregisterCallback
     */
    void unregisterCallback(canHandle *_hcan, uint32_t _stdid);

    /**
    * @brief can初始化并配置滤波器，不过滤任何ID
    */
    HAL_StatusTypeDef init(canHandle *_hcan, uint32_t _fifo);

    /**
    * @brief can发送普通数据帧
    */
    HAL_StatusTypeDef transmitData(canHandle *_hcan, uint16_t _stdid,
                                   uint8_t *_txData, uint32_t _len);

    /**
    * @brief can发送可变波特率数据帧
    */
    HAL_StatusTypeDef transmitBrsData(canHandle *_hcan, uint16_t _stdid,
                                      uint8_t *_txData, uint32_t _len);

    /**
    * @brief can rx callbackFromISR
    */
    void callbackFromISR(canHandle *_hcan, uint32_t _rxFifo);

private:
    std::unordered_map<canHandle *, std::unordered_map<uint32_t, callback> >
            cbTable;
};
