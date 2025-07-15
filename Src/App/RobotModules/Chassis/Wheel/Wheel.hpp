#pragma once
#include "QuadricycleController.hpp"

struct ChassisState_s {
    float v_x; // m/s
    float v_y; // m/s
    float w_z; // rad/s
};

class Wheel {
public:
    virtual void stop() = 0;
    virtual void update() = 0;
    virtual void ctrl(ChassisState_s _refState) = 0;

    ChassisState_s chassisState;

protected:
    std::shared_ptr<PowerController> ctrl_ = nullptr;
};
