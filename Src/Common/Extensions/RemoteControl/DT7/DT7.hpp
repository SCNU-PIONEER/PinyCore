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

    static constexpr uint16_t CH_VALUE_MIN = 364;
    static constexpr uint16_t CH_VALUE_OFFSET = 1024;
    static constexpr uint16_t CH_VALUE_MAX = 1684;
    static constexpr uint16_t CH_VALUE_RANGE = 660;
    static constexpr uint16_t WHEEL_VALUE_RANGE = 660;

    static constexpr uint8_t RX_LOST_MAX = 15;

    // TODO: Not support PC keyboard yet
    static constexpr int16_t MOUSE_MAX_ABS = 32767;

    static constexpr uint16_t W = 0x01 << 0;
    static constexpr uint16_t S = 0x01 << 1;
    static constexpr uint16_t A = 0x01 << 2;
    static constexpr uint16_t D = 0x01 << 3;
    static constexpr uint16_t SHIFT = 0x01 << 4;
    static constexpr uint16_t CTRL = 0x01 << 5;
    static constexpr uint16_t Q = 0x01 << 6;
    static constexpr uint16_t E = 0x01 << 7;
    static constexpr uint16_t R = 0x01 << 8;
    static constexpr uint16_t F = 0x01 << 9;
    static constexpr uint16_t G = 0x01 << 10;
    static constexpr uint16_t Z = 0x01 << 11;
    static constexpr uint16_t X = 0x01 << 12;
    static constexpr uint16_t C = 0x01 << 13;
    static constexpr uint16_t V = 0x01 << 14;
    static constexpr uint16_t B = 0x01 << 15;

    enum class Sw_e : uint8_t { UP = 1, MID = 3, DOWN = 2 };

public:
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
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t pressLeft;
            uint8_t pressRight;
        } mouse;
        struct {
            uint16_t keyCode;
            uint16_t lastKeyCode;
        } keyboard;
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
    void convert(RcMsg_t &_msg) final;
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit) final;

    RcData_s data_;
};
