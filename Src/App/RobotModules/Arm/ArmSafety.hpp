#pragma once

#include "ArmKinematic.hpp"

namespace ARM {

class Motors;


class Safety {
    static constexpr float DEFAULT_JOINT_SPEED_MIN = 0.f;          // rad/s
    static constexpr float DEFAULT_JOINT_SPEED_MAX = 0.4f;         // rad/s
    static constexpr float DEFAULT_JOINT_ACCELERATION_LOW = 0;     // 0~100
    static constexpr float DEFAULT_JOINT_ACCELERATION_HIGH = 10.f; // 0~100

public:
    explicit Safety(Motors &_motors) : motors_(_motors) {}

    void setSpeed(const float _speed);

    void setJointSpeedLimit(const float _time, const Joint _delta_ang);

    void setAllAngleLimit(Joint &_targetJoints);


private:
    Motors &motors_;

    float jointSpeedRatio = 1;
};

} // namespace ARM
