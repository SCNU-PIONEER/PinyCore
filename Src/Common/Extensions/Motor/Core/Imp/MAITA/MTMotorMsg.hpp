#pragma once

#include <cstdint>

namespace PINYMOTOR::MTMOTOR {

//按照MT电机手册V4.4划分
#pragma pack(push, 1)

//  ━━━━━━━━━━━━━━━━━━━ can - 读取电机状态1和错误标志指令 0x9A (目前只使用错误状态数据) ━━━━━━━━━━━━━━━━━━━
enum class ErrorCode_e : uint16_t {
    NO_ERR = 0x00u,
    STALL_MOTOR = 0x02u,
    LOW_VOLT = 0x04u,
    OVER_VOLT = 0x08u,
    OVER_CURRENT = 0x10u,
    POWER_OVER = 0x40u,
    PARA_ERR = 0x80u,
    OVER_SPEED = 0x0100u,
    PCB_HIGH_TEMP = 0x0800u,
    MOTOR_HIG_TEMP = 0x1000u,
    ENCODER_CAIL_ERR = 0x2000u,
    ENCDOER_DATA_ERR = 0x4000u,
};
/*  ━━━━━━━━━━━━━━━━━━━ can - 读取电机状态2指令 0x9c ━━━━━━━━━━━━━━━━━━━
  (uint8_t temp,int16_t iq,int16_t speed,int16_t pos)
   0x9c 读取电机状态2指令 反馈这个结构体,反馈帧头为0x9c
   0xA1 转矩闭环 反馈这个结构体,反馈帧头为0xA1 
   0xA2 速度闭环 反馈这个结构体,反馈帧头为0xA2
   0xA4 绝对位置闭环 反馈这个结构体,反馈帧头为0xA4 
   0xA6 单圈位置闭环 反馈这个结构体(int8_t temp,···uint16_t encoder),反馈帧头为0xA6
   0xA8 增量位置闭环 反馈这个结构体(int8_t temp),反馈帧头为0xA8
   0xA9 力控位置闭环 反馈这个结构体,反馈帧头为0xA9
   温度只取正值
*/
template <typename T1> struct FbData_s {
    uint8_t head;        //                        1  Byte
    uint8_t temperature; // 温度 1C/LSB             1  Byte
    int16_t iq;          // 转矩电流 0.01A/LSB      2  Byte
    int16_t speed;       // 输出轴转速 1dps/LSB     2  Byte
    T1 pos;              // 输出轴角度 1deg/LSB     2  Byte
}; // 电机读取状态2

//  ━━━━━━━━━━━━━━━━━━━ can - 转矩闭环控制指令 0xA1 ━━━━━━━━━━━━━━━━━━━
struct TransmitTorqCtrlMsg_s {
    uint8_t head = 0xA1; // 命令0xA1                        1  Byte
    uint8_t none1;       // 保留位                          1  Byte
    uint8_t none2;       // 保留位                          1  Byte
    uint8_t none3;       // 保留位                          1  Byte
    int16_t iqControl;   // 期望关节输出转矩 0.01A/LSB       2  Byte
    uint8_t none4;       // 保留位                          1  Byte
    uint8_t none5;       // 保留位                          1  Byte
}; // 电机控制命令数据包 8 Byte

//  ━━━━━━━━━━━━━━━━━━━ can - 速度闭环控制指令 0xA2 ━━━━━━━━━━━━━━━━━━━
struct TransmitSpeedCtrlMsg_s {
    uint8_t head = 0xA2;  // 命令0xA2                        1  Byte
    uint8_t torqueMax;    // 输出轴最大力矩 0.01额定电流       1  Byte
    uint8_t none2;        // 保留位                          1  Byte
    uint8_t none3;        // 保留位                          1  Byte
    int32_t speedControl; // 期望关节输出速度 0.01dps/LSB      4  Byte
}; // 电机控制命令数据包 8 Byte
//  ━━━━━━━━━━━━━━━━━━━ can - 绝对位置闭环控制指令 0xA4 ━━━━━━━━━━━━━━━━━━━
struct TransmitAbsPosCtrlMsg_s {
    uint8_t head = 0xA4; // 命令0xA4                         1  Byte
    uint8_t none;        // 保留位                           1  Byte
    uint16_t speedMax;   // 期望关节最大速度 1dps/LSB         2  Byte
    int32_t pos;         // 期望关节输出位置 0.01deg/LSB      4  Byte
}; // 电机控制命令数据包 8 Byte
//  ━━━━━━━━━━━━━━━━━━━ can - 单圈位置闭环控制指令 0xA6 ━━━━━━━━━━━━━━━━━━━
struct TransmitSinglePosCtrlMsg_s {
    uint8_t head = 0xA6; // 命令0xA6                        1  Byte
    uint8_t spinDir;     // 转动方向,0x00顺 0x01逆           1  Byte
    uint16_t speedMax;   // 输出轴实际转速 1dps/LSB          2  Byte
    uint16_t angleCtrl;  // 0.01dgeree/LSB                  2  Byte
    uint8_t none1;       // 保留位                          1  Byte
    uint8_t none2;       // 保留位                          1  Byte
}; // 电机控制命令数据包 8 Byte
//  ━━━━━━━━━━━━━━━━━━━ can - 增量位置闭环控制指令 0xA8 ━━━━━━━━━━━━━━━━━━━
struct TransmitIncrementalPosCtrlMsg_s {
    uint8_t head = 0xA8; // 命令0xA8                        1  Byte
    uint8_t none1;       // 保留位                          1  Byte
    uint16_t speedMax;   // 输出轴实际转速 1dps/LSB          2  Byte
    int32_t addAngle;    // 0.01dgeree/LSB                  4  Byte
}; // 电机控制命令数据包 8 Byte
//  ━━━━━━━━━━━━━━━━━━━ can - 力控闭环控制指令 0xA9 ━━━━━━━━━━━━━━━━━━━
struct TransmitForcePosCtrlMsg_s {
    uint8_t head = 0xA9; // 命令0xA9                        1  Byte
    uint8_t torqueMax;   // 输出轴最大力矩 0.01额定电流       1  Byte
    uint16_t speedMax;   // 输出轴最大速度 1 dps/LSB         2  Byte
    int32_t angCtrl;     // 输出关节角度 0.01degree/LSB      4  Byte
}; // 电机控制命令数据包 8 Byte

#pragma pack(pop)

} // namespace PINYMOTOR::MTMOTOR
