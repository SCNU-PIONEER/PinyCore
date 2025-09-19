#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR::DJIMOTOR {

class M3508 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float VOLT_TX_CODE_SPAN = 25000.f;
    static constexpr float CURR_TX_CODE_SPAN = 16384.f;
    static constexpr float CURR_RX_CODE_SPAN = 8192.f;

    static constexpr float CURR_RATED = 10.f;
    static constexpr float TORQ_RATED = 3.f / (3591.f / 187.f);
    static constexpr float VOLT_MAX = 25.2f;
    static constexpr float CURR_MAX = 2.5f;
    static constexpr float TORQ_MAX = 4.5f / (3591.f / 187.f);
    static constexpr float KN = 0.3f / (3591.f / 187.f);

    M3508(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DJIMOTOR
