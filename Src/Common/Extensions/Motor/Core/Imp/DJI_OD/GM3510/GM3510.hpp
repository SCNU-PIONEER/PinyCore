#pragma once
#include "../DJI_ODMotor.hpp"

namespace PINYMOTOR {
class GM3510 : public DJI_ODMotor<GM3510> {
private:
public:
    GM3510(const char _name[16], InitConfig_s _config);

    MotorTypeDef_e checkBaseConfig();
};
};
