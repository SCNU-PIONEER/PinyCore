#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {

#define M2006_VOLT_TX_CODE_SPAN 25000.f
#define M2006_CURR_TX_CODE_SPAN 16384.f
#define M2006_CURR_RX_CODE_SPAN 8192.f
#define M2006_CURR_RATED        3.f
#define M2006_TORQ_RATED        1.f
#define M2006_VOLT_MAX          25.2f
#define M2006_CURR_MAX          3.f
#define M2006_TORQ_MAX          1.f
#define M2006_TORQ_CONSTANT     0.18f

class M2006 final : public DJIMotor {
private:
public:
    M2006(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR
