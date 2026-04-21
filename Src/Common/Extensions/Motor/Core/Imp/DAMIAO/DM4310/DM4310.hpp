#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR::DMMOTOR {

class DM4310 final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MAX = 12.57f; // 120rpm
    static constexpr float T_MAX = 7.0f;
    static constexpr float MIT_KP_MAX = 500.f;
    static constexpr float MIT_KD_MAX = 5.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;

    static constexpr float CURR_RATED = 3.7f;
    static constexpr float TORQ_RATED = 3.f;
    static constexpr float CURR_MAX = 7.2f;
    static constexpr float TORQ_MAX = T_MAX;
    static constexpr float KN = TORQ_RATED / CURR_RATED;

    DM4310(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DMMOTOR
