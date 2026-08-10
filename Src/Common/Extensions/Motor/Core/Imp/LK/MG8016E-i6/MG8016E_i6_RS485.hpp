#pragma once
#include "../LKMotorRS485.hpp"

namespace PINYMOTOR::LKMOTOR {

class MG8016Ei6RS485 final : public LKMotorRS485 {
public:
    static constexpr float POWER_MAX = 670.f;           // 峰值功率 W
    static constexpr float TORQ_MAX = 37.f;             // 峰值扭矩 N/m
    static constexpr float SPEED_MAX = 130.f;           // 峰值转矩 130rpm@24V 258rpm@48V
    static constexpr float SPEED_CONSTANT = 41.7f;      // 转速常数 rpm/V
    static constexpr float TORQ_CONSTANT = 0.24f;       // 扭矩常数 N*m/A
    static constexpr float INNER_REDUCTION_RATIO = 6.f; // 内部减速比
    static constexpr float CURR_MAX = 66.f;             // 电流最大值
    static constexpr uint16_t ENCODER_SPAN = 65535;     // 16 bit编码器范围

    MG8016Ei6RS485(const char _name[16], InitConfig_s _config, WorkMode_e _workmode);
};

} // namespace PINYMOTOR::LKMOTOR
