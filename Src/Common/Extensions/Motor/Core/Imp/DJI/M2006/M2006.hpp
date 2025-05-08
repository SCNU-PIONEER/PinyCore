#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {
class M2006 : public DJIMotor<M2006> {
private:
public:
    M2006(const char _name[16], InitConfig_s _config);

    MotorTypeDef_e checkBaseConfig();
};
} // namespace PINYMOTOR