#pragma once
#include "../DMMotor.hpp"

#define DM4340_P_MAX             12.5f
#define DM4340_V_MAX             30.f
#define DM4340_T_MAX             10.f
#define DM4340_MITKp_MAX         500.f
#define DM4340_MITKd_MAX         5.f
#define DM4340_CURR_TX_CODE_SPAN 10000.f
#define DM4340_CURR_RATED        2.5f
#define DM4340_TORQ_RATED        9.f
#define DM4340_CURR_MAX          8.f
#define DM4340_TORQ_MAX          27.f
#define DM4340_TORQ_CONSTANT     0.9333f

namespace PINYMOTOR {
namespace DMMOTOR {

class DM4340 final : public DMMotor {
private:
public:
    DM4340(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
}
} // namespace PINYMOTOR
