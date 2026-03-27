#pragma once
#include <cstdint>
#include <array>

namespace PINYMOTOR::LKMOTOR {

enum class [[nodiscard]] ErrorState_e : uint8_t {
    NONE = 0,
    UNDER_VOLTAGE,           // 欠压
    OVER_VOLTAGE,            // 过压
    OVER_DRIVER_TEMPERATURE, // 驱动过温
    OVER_MOTOR_TEMPERATURE,  // 电机过温
    OVER_CURRENT,            // 过流
    SHORT_OUT,               // 短路
    OVERLOAD_ROTATION,       // 堵转
    INPUT_SIGNAL_LOST        // 输入信号丢失
};

enum class MotorState_e : uint8_t { DISABLE = 0x10, ENABLE = 0x00 };

#pragma pack(push, 1)

struct State1_s {
    int8_t temperature;
    int16_t voltage;
    int16_t current;
    MotorState_e motorState;
    ErrorState_e errorState;
};

struct State2_s {
    int8_t temperature;
    int16_t current; // MS power, MF/MG current
    int16_t speed;
    uint16_t encoder;
};

// not support for MF motor
struct State3_s {
    uint8_t temperature;
    int16_t iA;
    int16_t iB;
    int16_t iC;
};

template <uint8_t len> struct TransmitMsg_s;

template <uint8_t len> requires(len == 0) struct TransmitMsg_s<len> {
    const uint8_t head = 0X3E; // 帧头       1  Byte
    uint8_t CMD;               // 命令       1  Byte
    uint8_t ID;                // id         1  Byte
    uint8_t length = len;      // 数据长度   1  Byte
    uint8_t CMD_SUM;           // CRC校验    1  Byte
};

template <uint8_t len> requires(len > 0) struct TransmitMsg_s<len> {
    const uint8_t head = 0X3E;     // 帧头       1  Byte
    uint8_t CMD;                   // 命令       1  Byte
    uint8_t ID;                    // id         1  Byte
    uint8_t length = len;          // 数据长度   1  Byte
    uint8_t CMD_SUM;               // CRC校验    1  Byte
    std::array<uint8_t, len> DATA; // 帧数据     0~100 Byte
    uint8_t DATA_SUM;              // CRC校验    1  Byte
};

#pragma pack(pop)

enum class ParamID_e : uint8_t {
    ANGLE_PID = 0x0A,          // 角度环PID
    SPEED_PID = 0x0B,          // 速度环PID
    CURRENT_PID = 0x0C,        // 电流环PID
    INPUT_TORQUE_LIMIT = 0x1E, // 最大力矩电流
    INPUT_SPEED_LIMIT = 0X20,  // 最大速度
    INPUT_ANGLE_LIMIT = 0X22,  // 角度限制
    INPUT_CURRENT_RAMP = 0X24, // 电流斜率
    INPUT_SPEED_RAMP = 0X26    // 速度斜率
};

} // namespace PINYMOTOR::LKMOTOR
