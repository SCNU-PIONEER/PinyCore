#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR::DJIMOTOR {

class GM6020 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float VOLT_TX_CODE_SPAN = 25000.f;
    static constexpr float CURR_TX_CODE_SPAN = 16384.f;
    static constexpr float CURR_RX_CODE_SPAN = 16384.f;

    static constexpr float CURR_RATED = 1.62f;
    static constexpr float TORQ_RATED = 1.2f;
    static constexpr float VOLT_MAX = 25.2f;
    static constexpr float CURR_MAX = 3.f;
    static constexpr float CURR_BLOCK = 0.9f;
    static constexpr float TORQ_MAX = 2.23f;
    static constexpr float TORQ_BLOCK = 0.86f;

    static constexpr float KN = TORQ_RATED / CURR_RATED;

    GM6020(const char _name[16], InitConfig_s _config, WorkMode_e _workMode);
};

} // namespace PINYMOTOR::DJIMOTOR
