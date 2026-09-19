#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR::DJIMOTOR {

class M2006 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float VOLT_TX_CODE_SPAN = 25000.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;
    static constexpr float CURR_RX_CODE_SPAN = 10000.f;
    // 经过实验测算，数据手册表意不明，原始数据的unit: mA
    // 故此电机的电流数据需要除以1000，对应到parse中即 (÷10000 × 10) 来转换为A

    static constexpr float ORIGINAL_GEARBOX_RR = 36.f / 1.f;

    static constexpr float CURR_RATED = 3.f;
    static constexpr float TORQ_RATED = 1.f / ORIGINAL_GEARBOX_RR;
    static constexpr float VOLT_MAX = 25.2f;
    static constexpr float CURR_MAX = 10.f; // C610 MAX CURR
    static constexpr float TORQ_MAX = 1.8f / ORIGINAL_GEARBOX_RR;
    static constexpr float KN = TORQ_RATED / CURR_RATED; // 详见手册中 "搭配C610电调时的电机性能曲线"

    M2006(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
};

} // namespace PINYMOTOR::DJIMOTOR
