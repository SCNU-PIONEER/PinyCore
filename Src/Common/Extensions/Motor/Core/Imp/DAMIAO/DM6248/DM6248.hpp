#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR::DMMOTOR {

class DM6248 final : public DMMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float P_MAX = 12.57f;
    static constexpr float V_MAX = 20.f; // 191rpm
    static constexpr float T_MAX = 97.f;
    static constexpr float MIT_KP_MAX = 500.f;
    static constexpr float MIT_KD_MAX = 5.f;
    static constexpr float CURR_TX_CODE_SPAN = 10000.f;

    static constexpr float CURR_RATED = 11.3f;
    static constexpr float TORQ_RATED = 30.f;
    static constexpr float CURR_MAX = 38.7f;
    static constexpr float TORQ_MAX = T_MAX;
    static constexpr float KN = TORQ_RATED / CURR_RATED;

    DM6248(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
};

} // namespace PINYMOTOR::DMMOTOR
