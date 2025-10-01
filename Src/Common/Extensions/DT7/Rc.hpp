/**
 * @file rc.hpp
 * @brief 遥控器
 *
 * @version Version 1.0.1
 * @author yjy
 * @date 2025/9/8
 *
 * note 1.遥控器接受频率大概为70hz
 *      2.架构使用饿汉式单例
 *
 * @copyright SCNU-PIONEER (c) 2025-2026
 *
 */


#pragma once

#include "RcMsg.hpp"
#include "FreeRTOS.h"
#include "main.h"
#include "event_groups.h"
#include "Bsp.hpp"

#define RC_READY_EVENT (1 << 1)

namespace RC {

class Rc {
private:
    RcRawMsg_t data_;
    uint8_t *rcBuffer_;
    volatile uint8_t dt7RxLostCnt_ = RC_RX_LOST_MAX;
    EventGroupHandle_t event_;
    static constexpr uint16_t UPDATE_FREQ = 70;
    Uart uart_;

public:
    Rc(UART_HandleTypeDef *_huart);
    void init(EventGroupHandle_t _event);

    RcRawMsg_t &getData() { return data_; }

    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos);
    static void rawCallBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos);
    uint8_t parseData();

    bool isOnline();
};

} // namespace RC
