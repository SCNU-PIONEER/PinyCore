#pragma once
#include "../DJI_ODMotor.hpp"

#define GM3510_VOLT_TX_CODE_SPAN 29000.f
#define GM3510_TORQ_RX_CODE_SPAN 8192.f
#define GM3510_CURR_RATED        1.3f
#define GM3510_TORQ_RATED        0.14f
#define GM3510_VOLT_MAX          24.f
#define GM3510_CURR_MAX          0.6f
#define GM3510_TORQ_MAX          0.2f
#define GM3510_TORQ_CONSTANT     0.16f

namespace PINYMOTOR {
namespace DJI_ODMOTOR {

class GM3510 final : public DJI_ODMotor {
private:
public:
    GM3510(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
}
};
