#pragma once

#include "PowerController.hpp"

namespace CHASSIS {

union Speed_u {
    struct {
        float v_x; // m/s
        float v_y; // m/s
        float w_z; // rad/s
    };
    float _[3];
};

class Wheel {
public:
    virtual void stop() = 0;
    virtual void enter() = 0;
    virtual void update() = 0;
    virtual void ctrl(const Speed_u &_refSpeed) = 0;
    virtual ~Wheel() = default;

protected:
    std::unique_ptr<PowerController> powerCtrl_;
    Speed_u refSpeed_;
    Speed_u curSpeed_;
};

} // namespace CHASSIS
