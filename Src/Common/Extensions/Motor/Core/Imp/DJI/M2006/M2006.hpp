#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR::DJIMOTOR {

class M2006 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float VOLT_TX_CODE_SPAN = 25000.f;
    static constexpr float CURR_TX_CODE_SPAN = 16384.f;
    static constexpr float CURR_RX_CODE_SPAN = 8192.f;

    static constexpr float ORIGINAL_GEARBOX_RR = 36.f / 1.f;

    static constexpr float CURR_RATED = 3.f;
    static constexpr float TORQ_RATED = 1.f;
    static constexpr float VOLT_MAX = 25.2f;
    static constexpr float CURR_MAX = 3.f;
    static constexpr float TORQ_MAX = 1.f;
    static constexpr float KN = 0.18f;

    M2006(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DJIMOTOR
