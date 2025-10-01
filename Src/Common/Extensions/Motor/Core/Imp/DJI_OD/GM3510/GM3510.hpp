#pragma once
#include "../DJIOldMotor.hpp"

namespace PINYMOTOR::DJI_ODMOTOR {

class GM3510 final : public DJIOldMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    static constexpr float VOLT_TX_CODE_SPAN = 29000.f;
    static constexpr float TORQ_RX_CODE_SPAN = 8192.f;

    static constexpr float CURR_RATED = 1.3f;
    static constexpr float TORQ_RATED = 0.14f;
    static constexpr float VOLT_MAX = 24.f;
    static constexpr float CURR_MAX = 0.6f;
    static constexpr float TORQ_MAX = 0.2f;
    static constexpr float KN = 0.16f;

    GM3510(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR::DJI_ODMOTOR
