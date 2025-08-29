/* 
 * @file   ARMSafety.hpp
 * #@brief  Arm safety class definition for controlling a robotic arm.
 * 作为arm中间层，主要是对arm的关节位置和速度进行限制。
 * 主要是为了防止arm在工作时由于意外情况导致的关节损坏。
 */

#pragma once


#include "ArmMotor.hpp"

namespace ARM {


class Safety {
    static constexpr float DEFAULT_JOINT_SPEED_MIN = 0.f;          // rad/s
    static constexpr float DEFAULT_JOINT_SPEED_MAX = 0.4f;         // rad/s
    static constexpr float DEFAULT_JOINT_ACCELERATION_LOW = 0;     // 0~100
    static constexpr float DEFAULT_JOINT_ACCELERATION_HIGH = 10.f; // 0~100

public:
    explicit Safety(Motors &_motors) : motors_(_motors) {}

    void setSpeed(const float _speed);

    void setJointSpeedLimit(const float _time, const Joint7D _delta_ang);

    void setAllAngleLimit(Joint7D &_targetJoints);


private:
    Motors &motors_; // Reference to motor controller

    float jointSpeedRatio = 1;
};

} // namespace ARM
