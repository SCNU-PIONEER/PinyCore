#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR::DJIMOTOR {

class M3508 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float VOLT_TX_CODE_SPAN = 25000.f;
    static constexpr float CURR_TX_CODE_SPAN = 16384.f;
    static constexpr float CURR_RX_CODE_SPAN = 16384.f;

    static constexpr float ORIGINAL_GEARBOX_RR = 3591.f / 187.f;
    static constexpr float XROLL_GEARBOX_RR = 286.f / 17.f;

    static constexpr float CURR_RATED = 10.f;
    static constexpr float TORQ_RATED = 3.f / ORIGINAL_GEARBOX_RR;
    static constexpr float VOLT_MAX = 25.2f;
    static constexpr float CURR_MAX = 20.f; // C620 MAX CURR
    static constexpr float CURR_BLOCK = 2.5f;
    static constexpr float TORQ_BLOCK = 4.5f / ORIGINAL_GEARBOX_RR;
    static constexpr float KN =
            TORQ_RATED /
            CURR_RATED; // 详见手册中 "搭配C620电调时的电机性能曲线"

    M3508(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DJIMOTOR
