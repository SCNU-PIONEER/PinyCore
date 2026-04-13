
#include "ArmSafety.hpp"
#include <algorithm>
#include "ArmMotor.hpp"

using namespace ARM;


Safety::Safety(Motors &_motors, Joint _angleMin, Joint _angleMax)
        : motors(_motors), angleMin(_angleMin), angleMax(_angleMax) {};

void Safety::setSpeed(const float _speed)
{
    for (auto &speed : motors.ref_speed._) {
        speed = std::clamp(_speed, DEFAULT_JOINT_SPEED_MIN, DEFAULT_JOINT_SPEED_MAX);
    }
    float joint1Speed = std::clamp(_speed, DEFAULT_JOINT_SPEED_MIN, DEFAULT_JOINT_SPEED_MAX);
    motors.ref_speed.joint1 = joint1Speed + 3.f;
}

void Safety::setAllAngleLimit(Joint &_targetJoints)
{
    for (uint8_t i = 0; i < 6; i++) {
        _targetJoints[i] = std::clamp(_targetJoints[i], angleMin[i], angleMax[i]);
    }
}


void Safety::setYawPitchAngLimit(Joint &_targetJoints, const Joint &_msgJoints)
{
    float pitch = motors.pitchRoll[0] + _msgJoints[5];
    float roll = motors.pitchRoll[1] + _msgJoints[4];

    pitch = std::clamp(pitch, PITCH_ANG_MIN, PITCH_ANG_MAX);
    roll = std::clamp(roll, ROLL_ANG_MIN, ROLL_ANG_MAX);

    _targetJoints[4] = pitch + roll;
    _targetJoints[5] = -(pitch - roll);
}

void Safety::setYawPitchAngLimitAbs(Joint &_targetJoints, const Joint &_msgJoints)
{
    float pitch = std::clamp(_msgJoints[4], PITCH_ANG_MIN, PITCH_ANG_MAX);
    float roll = std::clamp(_msgJoints[5], ROLL_ANG_MIN, ROLL_ANG_MAX);

    _targetJoints[4] = pitch + roll;
    _targetJoints[5] = -(pitch - roll);
}
