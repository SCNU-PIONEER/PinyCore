#pragma once

#include "BspBase.hpp"
#include "Soc.hpp"
#include <functional>
#include <unordered_map>
#include <array>
#include HAL_INCLUDE

class Can : public BspBase<Can> {
public:
    using callback = std::function<void(const uint8_t *)>;
    /**
     * @brief can registerCallback
     */
    HAL_StatusTypeDef registerCallback(canHandle *_hcan, uint32_t _stdid,
                                       callback _pCallback);

    /**
     * @brief can unregisterCallback
     */
    void unregisterCallback(canHandle *_hcan, uint32_t _stdid);

    /**
    * @brief can初始化并配置滤波器，不过滤任何ID
    */
    HAL_StatusTypeDef init();

    /**
    * @brief can发送普通数据帧 */
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
    struct Handler_s {
        uint32_t stdid;
        callback func;
    };

    HAL_StatusTypeDef initSelf(canHandle *_hcan, uint32_t _fifo);

    /* one can max recv device number */
    static constexpr uint8_t MAX_RECV_DEVICE = 9;

    uint8_t can1cnt = 0;
    std::array<Handler_s, MAX_RECV_DEVICE> cbTable1;
    uint8_t can2cnt = 0;
    std::array<Handler_s, MAX_RECV_DEVICE> cbTable2;
#if SOC_CAN_NUM == 3
    uint8_t can3cnt = 0;
    std::array<Handler_s, MAX_RECV_DEVICE> cbTable3;
#endif
    // std::unordered_map<canHandle *, std::unordered_map<uint32_t, callback> >
    //         cbTable;
};
