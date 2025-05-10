#pragma once
#include "../DJIMotor.hpp"

namespace PINYMOTOR {
class M3508 : public DJIMotor<M3508> {
private:
public:
    M3508(const char _name[16], InitConfig_s _config);

    MotorTypeDef_e checkBaseConfig();
    void overrideReductionRatio(float _newReductionRatio){
        this->model_.reductionRatio = _newReductionRatio;
    }
};
} // namespace PINYMOTOR