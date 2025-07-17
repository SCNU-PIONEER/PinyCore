#pragma once

#include "PowerController.hpp"

namespace CHASSIS {

union speed_u {
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
    virtual void ctrl(const speed_u &_speed) = 0;

protected:
    std::unique_ptr<PowerController> powerCtrl_;
    speed_u speed_;
};

}
