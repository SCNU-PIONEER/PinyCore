#pragma once
#include "../DMMotor.hpp"

namespace PINYMOTOR {
class DM3519 : public DMMotor<DM3519> {
private:
public:
    DM3519(const char _name[16], InitConfig_s _config);

    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR
