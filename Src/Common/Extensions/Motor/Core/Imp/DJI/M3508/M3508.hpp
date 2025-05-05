#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {
class M3508 : public DJIMotor<M3508> {
private:
public:
    M3508(const char _name[16], InitConfig_s _config);
};
} // namespace PINYMOTOR