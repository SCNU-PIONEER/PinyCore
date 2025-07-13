#pragma once
#include "../DJIMotor.hpp"

#define GM6020_VOLT_TX_CODE_SPAN 25000.f
#define GM6020_CURR_TX_CODE_SPAN 16384.f
#define GM6020_CURR_RX_CODE_SPAN 8192.f
#define GM6020_CURR_RATED        1.62f
#define GM6020_TORQ_RATED        1.2f
#define GM6020_VOLT_MAX          25.2f
#define GM6020_CURR_MAX          0.9f
#define GM6020_TORQ_MAX          0.86f
#define GM6020_TORQ_CONSTANT     0.741f

namespace PINYMOTOR {
namespace DJIMOTOR {


class GM6020 final : public DJIMotor {
private:
public:
    GM6020(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
}
} // namespace PINYMOTOR
