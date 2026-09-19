#pragma once
#include "sdkconfig.h"
#if MF9025_RS485
#include "LKMotorRS485.hpp"
using LKMotor9025 = PINYMOTOR::LKMOTOR::LKMotorRS485;
#elif MF9025_CAN
#include "LKMotorCan.hpp"
using LKMotor9025 = PINYMOTOR::LKMOTOR::LKMotorCAN;
#elif MF9025_RS485_BROADCAST
#elif MF9025_CAN_BROADCAST
#include "LKMotor.hpp"
using LKMotor9025 = PINYMOTOR::LKMOTOR::LKMotor;
#endif

namespace PINYMOTOR::LKMOTOR {

class MF9025 final : public LKMotor9025 {
public:
    static constexpr float POWER_MAX = 170.f;           // 峰值功率 W
    static constexpr float TORQ_MAX = 4.5f;             //  峰值扭矩 N/m
    static constexpr float SPEED_MAX = 710.f;           // 峰值转矩 710rpm@24V
    static constexpr float SPEED_CONSTANT = 20.f;       // 转速常数 rpm/V
    static constexpr float TORQ_CONSTANT = 0.32f;       // 扭矩常数 N*m/A
    static constexpr float INNER_REDUCTION_RATIO = 1.f; // 内部减速比
    static constexpr float CURR_MAX = 33.f;             // 电流最大值
    static constexpr uint16_t ENCODER_SPAN = 65535;     // 16 bit编码器范围

    MF9025(const char _name[16], InitConfig_s _config, WorkMode_e _workmode);
};

} // namespace PINYMOTOR::LKMOTOR
