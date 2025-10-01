#pragma once

#include "../MTMotor.hpp"

namespace PINYMOTOR::MTMOTOR {

class RMDX436 : public MTMotor {
    static constexpr float CURR_MAX = 21.5f; //峰值相电流
    static constexpr float TORQ_MAX = 34.f;  // 峰值扭矩
    static constexpr uint8_t NP = 13;        // 极对数
    static constexpr float INTER_RR = 36.f;  // 内部减速比
    static constexpr float KN = 1.9f;        // 模组扭矩常数

public:
    RMDX436(const char _name[16], InitConfig_s _config);

private:
    MotorTypeDef_e checkBaseConfig();
};

} // namespace PINYMOTOR::MTMOTOR
