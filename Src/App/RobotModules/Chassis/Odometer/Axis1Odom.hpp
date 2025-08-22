#pragma once

#include "Odometry.hpp"
#include "SimpleTracker.hpp"
#include "dsp/fast_math_functions.h"

namespace CHASSIS {

class Axis1Odom : public Odometry {
public:
    Axis1Odom() = default;
    void update(float _dt, float _vel, float _acc, float _theta)
    {
        tracker_.updateCA(_dt, _vel, _acc);
        this->state_.theta = _theta;
        // sin(θ+π/2) = cos(θ)
        this->state_.x += tracker_.obsVel() * _dt * arm_cos_f32(_theta);
        this->state_.y += tracker_.obsVel() * _dt * -arm_sin_f32(_theta);
    }

    const float &normVel() { return tracker_.obsVel(); }
    const float &normAcc() { return tracker_.obsAcc(); }

private:
    SimpleTracker tracker_;
};

} // namespace CHASSIS
