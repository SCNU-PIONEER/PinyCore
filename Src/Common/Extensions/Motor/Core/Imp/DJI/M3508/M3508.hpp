#pragma once
#include "../DJIMotor.hpp"

#define M3508_VOLT_TX_CODE_SPAN 25000.f
#define M3508_CURR_TX_CODE_SPAN 16384.f
#define M3508_CURR_RX_CODE_SPAN 8192.f
#define M3508_CURR_RATED        10.f
#define M3508_TORQ_RATED        3.f
#define M3508_VOLT_MAX          25.2f
#define M3508_CURR_MAX          2.5f
#define M3508_TORQ_MAX          4.5f
#define M3508_TORQ_CONSTANT     0.3f

namespace PINYMOTOR {
namespace DJIMOTOR {


class M3508 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    M3508(const char _name[16], InitConfig_s _config);
};
}
} // namespace PINYMOTOR
