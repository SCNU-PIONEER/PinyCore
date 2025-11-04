#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR::DMMOTOR {
class DM3519 final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float P_MAX = 12.5f;
    static constexpr float V_MAX = 30.f;
    static constexpr float T_MAX = 10.f;
    static constexpr float MIT_KP_MAX = 500.f;
    static constexpr float MIT_KD_MAX = 5.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;

    static constexpr float ORIGINAL_GEARBOX_RR = 3591.f / 187.f;

    static constexpr float CURR_RATED = 9.2f;
    static constexpr float TORQ_RATED = 3.5f / ORIGINAL_GEARBOX_RR;
    static constexpr float CURR_MAX = 20.5f;
    static constexpr float TORQ_MAX = 7.8f / ORIGINAL_GEARBOX_RR;
    static constexpr float KN = 0.3805f;

    DM3519(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DMMOTOR
