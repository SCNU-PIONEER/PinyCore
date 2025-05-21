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

#include "rc_msg.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "main.h"

namespace RC {

class Rc {
private:
    Rc();
    static SemaphoreHandle_t dataReadySem; // 数据更新信号量
    rc_ctrl_t data;
    UART_HandleTypeDef *uart_;
    uint8_t *rc_buffer;

public:
    Rc(const Rc &) = delete;
    Rc &operator=(const Rc &) = delete;

    void init(UART_HandleTypeDef *huart);

    inline rc_ctrl_t &getData() { return data; }
    inline static Rc &instance()
    {
        static Rc instance_;
        return instance_;
    }

    static void callBackFromISR(UART_HandleTypeDef *huart, uint16_t Pos);
    uint8_t parseData();
};

}
