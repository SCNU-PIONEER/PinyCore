#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR {
class DM4310 final : public DMMotor {
private:
public:
    DM4310(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR
