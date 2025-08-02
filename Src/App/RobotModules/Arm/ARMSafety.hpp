/* 
 * @file   ARMSafety.hpp
 * #@brief  Arm safety class definition for controlling a robotic arm.
 * 作为arm中间层，主要是对arm的关节位置和速度进行限制。
 * 主要是为了防止arm在工作时由于意外情况导致的关节损坏。
 */

#pragma once

#include "ArmKinematic.hpp"
namespace ARM {
class Motors; 

class Safety {
    static constexpr float DEFAULT_JOINT_SPEED = 0.8f;             // rad/s
    static constexpr float DEFAULT_JOINT_SPEED_MIN = 0.f;          // rad/s
    static constexpr float DEFAULT_JOINT_SPEED_MAX = 0.4f;         // rad/s
    static constexpr float DEFAULT_JOINT_ACCELERATION_LOW = 0;     // 0~100
    static constexpr float DEFAULT_JOINT_ACCELERATION_HIGH = 10.f; // 0~100

public:
    Safety(Motors* _motors) : motors(_motors) {}

    bool angleLimit(const Joint7D &_joints);

    bool speedLimit(const float _speed);

private:
    Motors* motors;
    float jointSpeedRatio = 1;
};

}// namespace ARM

