#pragma once

#include "ArmPlan.hpp"

namespace ARM {

class Motors;

class Safety {
    static constexpr float DEFAULT_JOINT_SPEED = 0.4f;             // rad/s
    static constexpr float DEFAULT_JOINT_SPEED_MIN = 0.f;          // rad/s
    static constexpr float DEFAULT_JOINT_SPEED_MAX = 0.8f;         // rad/s
    static constexpr float DEFAULT_JOINT_ACCELERATION_LOW = 0;     // 0~100
    static constexpr float DEFAULT_JOINT_ACCELERATION_HIGH = 10.f; // 0~100

public:
    explicit Safety(Motors &_motors, Joint _angleMin, Joint _angleMax);

    void setSpeed(const float _speed);

    void setJointSpeedLimit(const Joint _currtentAng, const Joint _targetAng);

    void setAllAngleLimit(Joint &_targetJoints);

    const Joint &getAngleMin() const { return angleMin; }
    const Joint &getAngleMax() const { return angleMax; }
    const Joint &getSpeedLimit() const { return speedLimit; }

private:
    Motors &motors;

    Joint angleMin;   // motor min angle
    Joint angleMax;   // motor min angle
    Joint speedLimit; // motor speed limit

    float jointSpeedRatio = 1;
};

} // namespace ARM
