#pragma once

#include "../MTMotor.hpp"

namespace PINYMOTOR::MTMOTOR {

class RMDX410 : public MTMotor {
    static constexpr float SPEED_MAX = 24.923f; // 输出轴最大额定转速
    static constexpr float CURR_MAX = 19.5f;    // 峰值相电流
    static constexpr float TORQ_MAX = 10.f;     // 峰值扭矩
    static constexpr uint8_t NP = 11;           // 极对数
    static constexpr float INTER_RR = 12.5f;    // 内部减速比
    static constexpr float KN = 0.8f;           // 模组扭矩常数

public:
    RMDX410(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);

private:
    MotorTypeDef_e checkBaseConfig();
};

} // namespace PINYMOTOR::MTMOTOR
