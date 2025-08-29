#include "ARMSafety.hpp"
#include "ArmKinematic.hpp"
#include "MotorCommonMacros.hpp"

namespace ARM {


void Safety::setSpeed(const float _speed)
{
    motors_.ref_speed.joint2 = _speed;
    motors_.ref_speed.joint3 = _speed;
    motors_.ref_speed.joint4 = _speed;
    motors_.ref_speed.joint5 = _speed * 1.5f;
    motors_.ref_speed.joint6 = _speed * 1.5f;
}

void Safety::setJointSpeedLimit(const float _time, const Joint7D _delta_ang)
{
    for (uint8_t i = 1; i < 6; i++) {
        motors_.ref_speed._[i] = fabsf(_delta_ang.j[i] * _time);
    }

    for (uint8_t i = 1; i < 6; i++) {
        LIMIT_MIN_MAX(motors_.ref_speed._[i], DEFAULT_JOINT_SPEED_MIN,
                      DEFAULT_JOINT_SPEED_MAX);
    }
    motors_.ref_speed.joint1 = 0.f;
    motors_.ref_speed.joint5 = 1.5f;
    motors_.ref_speed.joint6 = 1.5f;
}

void Safety::setAllAngleLimit(Joint7D &_targetJoints)
{
    /*limit*/
    for (uint8_t i = 0; i < 7; i++) {
        LIMIT_MIN_MAX(_targetJoints.j[i], motors_.jointInfos[i].angle_min,
                      motors_.jointInfos[i].angle_max);
    }
    /* joint1  cmd implemented*/
    /* joint2-6  had targetjoint*/
}
} // namespace ARM