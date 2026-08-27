#pragma once
#include <cstdint>
#include <array>
#include "IMotor.hpp"

namespace PINYMOTOR::LKMOTOR {

enum class WorkMode_e : MotorTypeDef_e {
    UNKNOWN,
    QUAD_CURR,
    VOLT,
    VDES,
    MIT_TT,
    SINGLE_PDES,
    SINGLE_PDESVDES,
    MULTI_PDESVDES,
    MULTI_PDES,
    PDESVDES,
    INC_PDES,
    INC_PDESVDES,
};

enum class [[nodiscard]] ErrorState_e : uint8_t {
    NONE = 0,
    UNDER_VOLTAGE,
    OVER_VOLTAGE,
    OVER_DRIVER_TEMPERATURE,
    OVER_MOTOR_TEMPERATURE,
    OVER_CURRENT,
    SHORT_OUT,
    OVERLOAD_ROTATION,
    INPUT_SIGNAL_LOST
};

enum class MotorState_e : uint8_t { DISABLE = 0x10, ENABLE = 0x00 };

struct Status_s {
    float powerMax;            // 峰值功率
    float torqueMax;           // 峰值扭矩
    int32_t speedMax;          // 峰值转矩
    float speedConstant;       // 转速常数
    float torqueConstant;      // 扭矩常数
    int16_t CurrMax;           // 转矩电流最大值
    float innerReductionRatio; // 内部减速比

    Status_s() = default;

    Status_s(float _powerMax, float _torqueMax, int32_t _speedMax, float _speedConstant, float _torqueConstant,
             int16_t _currMax, float _innerReductionRatio)
            : powerMax(_powerMax)
            , torqueMax(_torqueMax)
            , speedMax(_speedMax)
            , speedConstant(_speedConstant)
            , torqueConstant(_torqueConstant)
            , CurrMax(_currMax)
            , innerReductionRatio(_innerReductionRatio) {};
};

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
    int16_t current;
    int16_t speed;
    uint16_t encoder;
};

struct State3_s {
    uint8_t temperature;
    int16_t iA;
    int16_t iB;
    int16_t iC;
};

template <uint8_t len> struct TransmitMsg_s;

template <uint8_t len> requires(len == 0) struct TransmitMsg_s<len> {
    const uint8_t head = 0X3E;
    uint8_t CMD;
    uint8_t ID;
    uint8_t length = len;
    uint8_t CMD_SUM;
};

template <uint8_t len> requires(len > 0) struct TransmitMsg_s<len> {
    const uint8_t head = 0X3E;
    uint8_t CMD;
    uint8_t ID;
    uint8_t length = len;
    uint8_t CMD_SUM;
    std::array<uint8_t, len> DATA;
    uint8_t DATA_SUM;
};

// CAN TX Message
struct VOLTMsg_s {
    uint8_t head = 0xA0;
    uint8_t none1;
    uint8_t none2;
    uint8_t none3;
    int16_t powerControl;
    uint8_t none4;
    uint8_t none5;
};

struct MITMsg_s {
    uint8_t head = 0xA1;
    uint8_t none1;
    uint8_t none2;
    uint8_t none3;
    int16_t iqControl;
    uint8_t none4;
    uint8_t none5;
};

struct VDESMsg_s {
    uint8_t head = 0xA2;
    uint8_t none1;
    int16_t iqControl;
    int32_t speedControl;
};

struct MULTIPDESMsg_s {
    uint8_t head = 0xA3;
    uint8_t none1;
    uint8_t none2;
    uint8_t none3;
    int32_t angleControl;
};

struct MULTIPDESVDESMsg_s {
    uint8_t head = 0xA4;
    uint8_t none1;
    uint16_t maxSpeed;
    int32_t angleControl;
};

struct SINGLEPDESMsg_s {
    uint8_t head = 0xA5;
    uint8_t spinDirection;
    uint8_t none1;
    uint8_t none2;
    uint32_t angleControl;
};

struct SINGLEPDESVDESMsg_s {
    uint8_t head = 0xA6;
    uint8_t spinDirection;
    uint16_t maxSpeed;
    uint32_t angleControl;
};

struct INCPDESMsg_s {
    uint8_t head = 0xA7;
    uint8_t none1;
    uint8_t none2;
    uint8_t none3;
    int32_t angleIncrement;
};

struct INCPDESVDESMsg_s {
    uint8_t head = 0xA8;
    uint8_t none1;
    uint16_t maxSpeed;
    int32_t angleIncrement;
};

#pragma pack(pop)

enum class ParamID_e : uint8_t {
    ANGLE_PID = 0x0A,
    SPEED_PID = 0x0B,
    CURRENT_PID = 0x0C,
    INPUT_TORQUE_LIMIT = 0x1E,
    INPUT_SPEED_LIMIT = 0X20,
    INPUT_ANGLE_LIMIT = 0X22,
    INPUT_CURRENT_RAMP = 0X24,
    INPUT_SPEED_RAMP = 0X26
};

} // namespace PINYMOTOR::LKMOTOR