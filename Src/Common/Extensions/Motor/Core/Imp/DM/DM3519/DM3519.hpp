#pragma once
#include "../DMMotor.hpp"

#define DM3519_P_MAX             12.5f
#define DM3519_V_MAX             30.f
#define DM3519_T_MAX             10.f
#define DM3519_MITKp_MAX         500.f
#define DM3519_MITKd_MAX         5.f
#define DM3519_CURR_TX_CODE_SPAN 10000.f
#define DM3519_CURR_RATED        9.2f
#define DM3519_TORQ_RATED        3.5f
#define DM3519_CURR_MAX          20.5f
#define DM3519_TORQ_MAX          7.8f
#define DM3519_TORQ_CONSTANT     0.3805f

namespace PINYMOTOR {
namespace DMMOTOR {
class DM3519 final : public DMMotor {
private:
public:
    DM3519(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
}
} // namespace PINYMOTOR
