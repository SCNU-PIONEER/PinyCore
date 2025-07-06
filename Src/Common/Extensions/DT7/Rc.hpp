/**
 * @file rc.hpp
 * @brief 遥控器
 *
 * @version Version 1.0.0
 * @author yjy
 * @date 2025/3/19
 *
 * note 1.遥控器接受频率大概为70hz
 *      2.架构使用饿汉式单例模式，保证线程安全
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

public:
    Rc(const Rc &) = delete;
    Rc &operator=(const Rc &) = delete;

    void init(UART_HandleTypeDef *_huart, EventGroupHandle_t _event);

    inline RcRawMsg_t &getData() { return data_; }

    inline static Rc &instance()
    {
        static Rc instance_;
        return instance_;
    }

    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _Pos);
    static void RawCallBackFromISR(UART_HandleTypeDef *_huart, uint16_t _Pos);
    uint8_t parseData();
};

}
