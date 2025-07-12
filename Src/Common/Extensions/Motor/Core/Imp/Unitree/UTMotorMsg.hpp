#pragma once

#include <cstdint>
#include <cstring>

namespace PINYMOTOR {
namespace UTMOTOR {

enum class ErrorStatus_e {
    Normal = 0u,
    OverHot,
    OverCurrent,
    OverVoltage,
    EncoderError,
};

#pragma pack(push, 1)
/**
 * @brief 电机模式控制信息
 */
struct RISMode_s {
    uint8_t id : 4; // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3; // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none : 1;   // 保留位
}; // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 */
struct RISComd_s {
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)
}; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 */
struct RISFbk_s {
    int16_t torque;     // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;      // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;        // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;        // 电机温度: -128~127°C
    uint8_t MError : 3; // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障
                        // 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
}; // 状态数据 11Byte

struct TransmitMsg_s {
    // 定义 电机控制命令数据包
    uint8_t head[2]; // 包头               2  Byte
    RISMode_s mode;  // 电机控制模式       1  Byte
    RISComd_s comd;  // 电机期望数据       12 Byte
    uint16_t CRC16;  // CRC                2  Byte
}; // 电机控制命令数据包 17 Byte

struct Feedback_s {
    uint8_t head[2]; // 包头          2  Byte
    RISMode_s mode;  // 电机控制模式  1  Byte
    RISFbk_s fbk;    // 电机反馈数据  11 Byte
    uint16_t CRC16;  // CRC           2  Byte
}; // 返回数据      16 Byte

#pragma pack(pop)

}
}
