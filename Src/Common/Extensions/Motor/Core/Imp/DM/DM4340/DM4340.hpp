#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR {
class DM4340 : public DMMotor {
private:
public:
    DM4340(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR
