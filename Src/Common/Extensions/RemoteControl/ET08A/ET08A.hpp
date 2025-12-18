/**
 * @file       ET08A.hpp
 * @brief      ET08A 遥控器驱动，使用SBUS协议，DMA双缓冲环形接收，如果要改成
 *             单缓冲，word length要改成 9bits，sbus一帧25字节，但是ET08A
 *             八通道只用 12 字节。
 *             接收频率大概70Hz
 *             先按照说明书对码
 *             接收机为 RF209s, 插口要选择 wbus 而不是 wbus2
 *             接收机为 RF206s, 主菜单 —> 通信设置 —> 接收机端口设置 —> 通道6 —> W.BUS
 * @config     ET08A 系统设置 -> 摇杆模式 -> 模式2
 *             ET08A 主菜单 —> 通用功能 —> 通道设置 -> 5 -> SB
 *             ET08A 主菜单 —> 通用功能 —> 通道设置 -> 6 -> SC
 *             ET08A 主菜单 —> 通用功能 —> 通道设置 -> 7 -> LD
 *             ET08A 主菜单 —> 通用功能 —> 通道设置 -> 8 -> --
 */

#pragma once

#include <cstdint>
#include "Bsp.hpp"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "remoteControl.hpp"

#define ET08A_READY_EVENT (1 << 3)

class ET08A final : public RemoteControl {
    static constexpr uint8_t FRAME_LENGTH = 25;

    static constexpr uint16_t CH_VALUE_OFFSET = 1024;
    static constexpr uint16_t CH_VALUE_RANGE = 670;
    static constexpr uint16_t WHEEL_VALUE_RANGE = (1694 - 353) / 2;

    static constexpr uint8_t RX_LOST_MAX = 15;

    enum class Sw_e : uint16_t { UP = 353, MID = 1024, DOWN = 1694 };

public:
#pragma pack(push, 1)
    struct Sbus_s {
        uint16_t ch[8];
    };

    struct RcData_s {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        Sw_e switchLeft;
        Sw_e switchRight;
        uint16_t wheel;
    };
#pragma pack(pop)

    ET08A(UART_HandleTypeDef *_huart, EventGroupHandle_t &_event);
    ~ET08A() final;
    bool isOnline() final;
    void *getData() final;

private:
    void parse() final;
    void convert(RcMsg_t &_msg) final;
    void callBackFromISR(UART_HandleTypeDef *_huart, uint16_t _pos) final;

    Sbus_s sbus_{};
    RcData_s data_{};
};
