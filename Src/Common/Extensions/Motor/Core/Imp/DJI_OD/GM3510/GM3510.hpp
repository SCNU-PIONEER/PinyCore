#pragma once
#include "../DJI_ODMotor.hpp"

namespace PINYMOTOR {
class GM3510 final : public DJI_ODMotor {
private:
public:
    GM3510(const char _name[16], InitConfig_s _config);
    MotorTypeDef_e checkBaseConfig();
};
};
