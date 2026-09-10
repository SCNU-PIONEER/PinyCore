#pragma once
#include "sdkconfig.h"

#if MG4005_RS485
#include "LKMotorRS485.hpp"
using LKMotor4005 = PINYMOTOR::LKMOTOR::LKMotorRS485;
#elif MG4005_CAN
#include "LKMotorCan.hpp"
using LKMotor4005 = PINYMOTOR::LKMOTOR::LKMotorCAN;
#elif MG4005_RS485_BROADCAST
#elif MG4005_CAN_BROADCAST
#include "LKMotor.hpp"
using LKMotor4005 = PINYMOTOR::LKMOTOR::LKMotor;
#endif

namespace PINYMOTOR::LKMOTOR {

class MG4005 final : public LKMotor4005 {
public:
    static constexpr float POWER_MAX = 65.f;             // 峰值功率 W
    static constexpr float TORQ_MAX = 2.5f;              // 峰值扭矩 N/m
    static constexpr float SPEED_MAX = 320.f;            // 峰值转矩 320rpm@24V
    static constexpr float SPEED_CONSTANT = 106.3f;      // 转速常数 rpm/V
    static constexpr float TORQ_CONSTANT = 0.06f;        // 扭矩常数 N*m/A
    static constexpr float INNER_REDUCTION_RATIO = 10.f; // 内部减速比
    static constexpr float CURR_MAX = 66.f;              // 转矩电流最大值
    static constexpr uint16_t ENCODER_SPAN = 65535.f;    // 16 bit编码器范围

    MG4005(const char _name[16], InitConfig_s _config, WorkMode_e _workmode);
};

} // namespace PINYMOTOR::LKMOTOR
