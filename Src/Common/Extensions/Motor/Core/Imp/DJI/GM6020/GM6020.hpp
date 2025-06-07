#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {
class GM6020 : public DJIMotor {
private:
public:
    GM6020(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR
