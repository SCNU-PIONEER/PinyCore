#pragma once

#include "Odometry.hpp"
#include "SimpleTracker.hpp"
#include "dsp/fast_math_functions.h"

namespace CHASSIS {

class Axis2Odom : public Odometry {
public:
    Axis2Odom() = default;
    void update(float _dt, const float _vel[2], const float _acc[2],
                float _theta)
    {
        this->state_.theta = _theta;
        tracker_[0].updateCA(_dt, _vel[0], _acc[0]);
        tracker_[1].updateCA(_dt, _vel[1], _acc[1]);
        this->state_.x += tracker_[0].obsVel() * _dt;
        this->state_.y += tracker_[1].obsVel() * _dt;
    }

    float normVel()
    {
        float vel;
        arm_sqrt_f32((tracker_[0].obsVel() * tracker_[0].obsVel()) +
                             (tracker_[1].obsVel() * tracker_[1].obsVel()),
                     &vel);
        return vel;
    }
    float normAcc()
    {
        float acc;
        arm_sqrt_f32((tracker_[0].obsAcc() * tracker_[0].obsAcc()) +
                             (tracker_[1].obsAcc() * tracker_[1].obsAcc()),
                     &acc);
        return acc;
    }

private:
    SimpleTracker tracker_[2];
};

} // namespace CHASSIS
