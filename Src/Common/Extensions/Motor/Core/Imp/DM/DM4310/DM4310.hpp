#pragma once
#include "../DMMotor.hpp"

#define DM4310_P_MAX             12.5f
#define DM4310_V_MAX             30.f
#define DM4310_T_MAX             10.0f
#define DM4310_MITKp_MAX         500.f
#define DM4310_MITKd_MAX         5.f
#define DM4310_CURR_TX_CODE_SPAN 10000.f
#define DM4310_CURR_RATED        2.5f
#define DM4310_TORQ_RATED        3.f
#define DM4310_CURR_MAX          7.5f
#define DM4310_TORQ_MAX          7.f
#define DM4310_TORQ_CONSTANT     0.9333f

namespace PINYMOTOR {
namespace DMMOTOR {

class DM4310 final : public DMMotor {
private:
public:
    DM4310(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
}
} // namespace PINYMOTOR
