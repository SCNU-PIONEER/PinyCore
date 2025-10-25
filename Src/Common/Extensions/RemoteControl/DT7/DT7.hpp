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

#include "DT7Msg.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "Bsp.hpp"
#include "remoteControl.hpp"

#define RC_READY_EVENT (1 << 1)

class Rc : public RemoteControl {
    static constexpr uint8_t FRAME_LENGTH = 18;

public:
    Rc(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    ~Rc() override;
    bool isOnline() final;

    void *getData() final;

private:
    void parse() final;
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos) final;

    RcRawMsg_t data_;
    static constexpr uint16_t UPDATE_FREQ = 70;
};
