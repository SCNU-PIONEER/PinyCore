
#include "ArmSafety.hpp"
#include <algorithm>
#include "ArmMotor.hpp"

using namespace ARM;


Safety::Safety(Motors &_motors, Joint10 _angleMin, Joint10 _angleMax)
        : motors(_motors), angleMin(_angleMin), angleMax(_angleMax) {};

void Safety::setSpeed(const float _speed)
{
    for (auto &speed : motors.ref_speed._) {
        speed = std::clamp(_speed, DEFAULT_JOINT_SPEED_MIN, DEFAULT_JOINT_SPEED_MAX);
    }
}

void Safety::setJointSpeedLimit(const Joint10 _currtentAng, const Joint10 _targetAng)
{
    const Joint10 deltaAng = _targetAng - _currtentAng;
    const float maxDeltaAngle = AbsMaxOf10(deltaAng);
    const float maxTime = maxDeltaAngle / DEFAULT_JOINT_SPEED;

    for (uint8_t i = 1; i < 10; i++) {
        motors.ref_speed._[i] = fabsf(deltaAng[i] * maxTime);
    }

    for (uint8_t i = 1; i < 10; i++) {
        motors.ref_speed._[i] = std::clamp(motors.ref_speed._[i], angleMin[i], angleMax[i]);
    }
}

void Safety::setAllAngleLimit(Joint10 &_targetJoints)
{
    for (uint8_t i = 0; i < 10; i++) {
        _targetJoints[i] = std::clamp(_targetJoints[i], angleMin[i], angleMax[i]);
    }
}
