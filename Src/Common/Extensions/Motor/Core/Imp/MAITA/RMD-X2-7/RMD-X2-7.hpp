#pragma once

#include "../MTMotor.hpp"

namespace PINYMOTOR::MTMOTOR {

static constexpr float CURR_MAX = 8.1f;   //峰值相电流
static constexpr float TORQ_MAX = 7.f;    // 峰值扭矩
static constexpr uint8_t NP = 13;         // 极对数
static constexpr float INTER_RR = 28.17f; // 内部减速比
static constexpr float KN = 0.8f;         // 模组扭矩常数

class RMDX27 : public MTMotor {
public:
    RMDX27(const char _name[16], InitConfig_s _config);

private:
    MotorTypeDef_e checkBaseConfig();
};

} // namespace PINYMOTOR::MTMOTOR
