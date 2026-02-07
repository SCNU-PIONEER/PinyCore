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

#include "FreeRTOS.h"
#include "event_groups.h"
#include "Bsp.hpp"
#include "remoteControl.hpp"

class DT7 final : public RemoteControl {
    static constexpr uint8_t FRAME_LENGTH = 18;
    static constexpr uint8_t RX_LOST_MAX = 15;

public:
    static constexpr uint16_t CH_VALUE_MIN = 364;
    static constexpr uint16_t CH_VALUE_OFFSET = 1024;
    static constexpr uint16_t CH_VALUE_MAX = 1684;

    static constexpr int16_t MOUSE_MAX_ABS = 32767;

    enum class Sw_e : uint8_t { UP = 1, MID = 3, DOWN = 2 };

#pragma pack(push, 1)
    struct RcData_s {
        struct {
            int16_t ch0;
            int16_t ch1;
            int16_t ch2;
            int16_t ch3;
            Sw_e switchLeft;
            Sw_e switchRight;
        } rc;
        struct {
            int16_t mouseX;
            int16_t mouseY;
            int16_t mouseZ;
            bool mouseLeft;
            bool mouseRight;
            bool mouseMiddle;
            uint16_t key;
            uint16_t lastKey;
        };
        //1684 - 1024 - 364， 自动居中 1024
        int16_t wheel;
    };
#pragma pack(pop)

    DT7(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event, uint32_t _eventBit);
    ~DT7() final;
    bool isOnline() final;

    void *getData() final;

private:
    void parse() final;
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit) final;

    RcData_s data_;
};
