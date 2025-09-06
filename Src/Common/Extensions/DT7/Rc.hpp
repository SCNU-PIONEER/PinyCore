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

#define RC_READY_EVENT (1 << 1)

namespace RC {

class Rc {
private:
    Rc();
    RcRawMsg_t data_;
    UART_HandleTypeDef *uart_;
    uint8_t *rcBuffer_;
    uint8_t dt7RxLostCnt_ = RC_RX_LOST_MAX;
    EventGroupHandle_t event_;
    static constexpr uint16_t UPDATE_FREQ = 70;

public:
    Rc(const Rc &) = delete;
    Rc &operator=(const Rc &) = delete;

    void init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event);

    RcRawMsg_t &getData() { return data_; }

    static Rc &instance()
    {
        static Rc instance;
        return instance;
    }

    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos);
    static void rawCallBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos);
    uint8_t parseData();

    bool isOnline();
};

} // namespace RC
