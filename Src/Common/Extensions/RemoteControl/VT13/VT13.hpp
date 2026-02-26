/**
 * @file       VT13.hpp
 * @brief      裁判系统图传相机模组 VT13 发送端 (使用VT03 接收端串口接收) 驱动
 *             波特率 921600
 *             数据位 8
 *             停止位 1
 *             校验位 无
 *             流控 无
 *             单帧包长 153 + 16 bit (21 Byte)
 */
#pragma once

#include "FreeRTOS.h"
#include "event_groups.h"
#include "Bsp.hpp"
#include "remoteControl.hpp"

class VT13 final : public RemoteControl {
    static constexpr uint8_t FRAME_LENGTH = 21;
    static constexpr uint8_t RX_LOST_MAX = 15;

public:
    static constexpr uint16_t CH_VALUE_MIN = 364;
    static constexpr uint16_t CH_VALUE_OFFSET = 1024;
    static constexpr uint16_t CH_VALUE_MAX = 1684;

    static constexpr int16_t MOUSE_VALUE_MIN = -32768;
    static constexpr int16_t MOUSE_VALUE_MAX = 32767;

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

    enum class Sw_e : uint8_t { C = 0, N = 1, S = 2 };

#pragma pack(push, 1)
    struct RcRawData_s {
        uint8_t sof1;
        uint8_t sof2;
        uint16_t ch0 : 11;
        uint16_t ch1 : 11;
        uint16_t ch2 : 11;
        uint16_t ch3 : 11;
        uint8_t sw : 2;
        uint8_t pause : 1;
        uint8_t fn1 : 1;
        uint8_t fn2 : 1;
        uint16_t wheel : 11;
        uint8_t trigger : 1;

        uint8_t reserved1 : 3;
        int16_t mouseX;
        int16_t mouseY;
        int16_t mouseZ;
        uint8_t mouseLeft : 2;
        uint8_t mouseRight : 2;
        uint8_t mouseMiddle : 2;
        uint8_t reserved2 : 2;
        uint16_t key;
        uint16_t crc16;
    };

    struct RcData_s {
        struct {
            int16_t ch[4];
            Sw_e sw;
            bool pause;
            bool fn1;
            bool fn2;
            int16_t wheel;
            bool trigger;
        } gamepad;
        struct {
            int16_t mouseX;
            int16_t mouseY;
            int16_t mouseZ;
            bool mouseLeft;
            bool mouseRight;
            bool mouseMiddle;
            uint16_t key;
        } KBM;
    };
#pragma pack(pop)

    VT13(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event, uint32_t _eventBit);
    ~VT13() final;
    bool isOnline() final;

    void *getData() final;

private:
    void parse() final;
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos, uint32_t _eventBit) final;

    RcRawData_s *raw_ = nullptr;
    RcData_s data_;
};