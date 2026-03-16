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
    explicit Safety(Motors &_motors, Joint10 _angleMin, Joint10 _angleMax);

    void setSpeed(const float _speed);

    void setJointSpeedLimit(const Joint10 _currtentAng, const Joint10 _targetAng);

    void setAllAngleLimit(Joint10 &_targetJoints);

    const Joint10 &getAngleMin() const { return angleMin; }
    const Joint10 &getAngleMax() const { return angleMax; }
    const Joint10 &getSpeedLimit() const { return speedLimit; }

private:
    Motors &motors;

    Joint10 angleMin;   // motor min angle
    Joint10 angleMax;   // motor min angle
    Joint10 speedLimit; // motor speed limit

    float jointSpeedRatio = 1;
};

} // namespace ARM
