#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR::DMMOTOR {

class DM10010L final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MAX = 4.19f; // 40rpm
    static constexpr float T_MAX = 120.0f;
    static constexpr float MIT_KP_MAX = 500.f;
    static constexpr float MIT_KD_MAX = 5.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;

    static constexpr float CURR_RATED = 23.5f;
    static constexpr float TORQ_RATED = 40.f;
    static constexpr float CURR_MAX = 95.f;
    static constexpr float TORQ_MAX = T_MAX;
    static constexpr float KN = TORQ_RATED / CURR_RATED;

    DM10010L(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DMMOTOR