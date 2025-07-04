#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {
class M3508 final : public DJIMotor {
private:
    MotorTypeDef_e checkBaseConfig();

public:
    M3508(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR
