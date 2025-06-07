#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {
class M3508 final : public DJIMotor{
private:
public:
    M3508(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR
