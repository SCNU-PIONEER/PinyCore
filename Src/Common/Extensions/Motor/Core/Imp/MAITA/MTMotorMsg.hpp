#pragma once

#include <cstdint>

namespace PINYMOTOR::MTMOTOR {

#pragma pack(push, 1)

//  ━━━━━━━━━━━━━━━━━━━ can - 绝对位置闭环控制指令 0xa4 ━━━━━━━━━━━━━━━━━━━
struct TransmiAbsPosCtrlMsg_s {
    uint8_t head = 0xA4; // 命令0xA4                         1  Byte
    uint8_t none;        // 保留位                           1  Byte
    uint16_t maxspeed;   // 期望关节最大速度 1dps/LSB         2  Byte
    int32_t pos;         // 期望关节输出位置 0.01deg/LSB      4  Byte
}; // 电机控制命令数据包 8 Byte

struct FeedbackAbsPosCtrl_s {
    uint8_t head;        // 命令0xA4              1  Byte
    uint8_t temperature; // 温度                  1  Byte
    int16_t iq;          // 转矩电流              2  Byte
    int16_t speed;       // 输出轴转速            2  Byte
    int16_t pos;         // 输出轴角度            2  Byte
}; // 电机反馈数据包 8 Byte

//  ━━━━━━━━━━━━━━━━━━━ can - 转矩闭环控制指令 0xA1 ━━━━━━━━━━━━━━━━━━━
struct TransmiTorqCtrlMsg_s {
    uint8_t head = 0xA1; // 命令0xA1                        1  Byte
    uint8_t none1;       // 保留位                          1  Byte
    uint8_t none2;       // 保留位                          1  Byte
    uint8_t none3;       // 保留位                          1  Byte
    int16_t iqControl;   // 期望关节输出转矩 0.01A/LSB       2  Byte
    uint8_t none4;       // 保留位                          1  Byte
    uint8_t none5;       // 保留位                          1  Byte
}; // 电机控制命令数据包 8 Byte

struct FeedbackTorqCtrl_s {
    uint8_t head;        // 命令0xA1              1  Byte
    uint8_t temperature; // 温度                  1  Byte
    int16_t iq;          // 转矩电流              2  Byte
    int16_t speed;       // 输出轴转速            2  Byte
    int16_t pos;         // 输出轴角度            2  Byte
}; // 电机反馈数据包 8 Byte

//  ━━━━━━━━━━━━━━━━━━━ can - 读取电机状态2指令 0x9c ━━━━━━━━━━━━━━━━━━━
struct FeedbackState2_s {
    uint8_t head;        // 命令 0x9c               1  Byte
    uint8_t temperature; // 温度 1C/LSB             1  Byte
    int16_t iq;          // 转矩电流 0.01A/LSB      2  Byte
    int16_t speed;       // 输出轴转速 1dps/LSB     2  Byte
    int16_t pos;         // 输出轴角度 1deg/LSB     2  Byte
}; // 电机读取状态2

#pragma pack(pop)

} // namespace PINYMOTOR::MTMOTOR
