#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR::DMMOTOR {

class DM10010L final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float P_MAX = 12.5f;
    static constexpr float V_MAX = 25.f;
    static constexpr float T_MAX = 200.0f;
    static constexpr float MIT_KP_MAX = 500.f;
    static constexpr float MIT_KD_MAX = 5.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;

    static constexpr float CURR_RATED = 31.7f;
    static constexpr float TORQ_RATED = 40.f;
    static constexpr float CURR_MAX = 95.0f;
    static constexpr float TORQ_MAX = 120.f;
    static constexpr float KN = 0.7917f;

    DM10010L(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DMMOTOR