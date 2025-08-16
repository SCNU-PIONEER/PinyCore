#include "ARMSafety.hpp"
#include "MotorCommonMacros.hpp"

namespace ARM {

bool Safety::angleLimit(const Joint7D &_joints)
{
    for (int i = 0; i < 7; ++i) {
        float min = motors_.jointInfos[i].angle_min;
        float max = motors_.jointInfos[i].angle_max;
        if (_joints.j[i] < min || _joints.j[i] > max) {
            return false;
        }
    }
    return true;
}

bool Safety::speedLimit(const float _speed)
{
    motors_.ref_speed.joint2 = _speed;
    motors_.ref_speed.joint3 = _speed;
    motors_.ref_speed.joint4 = _speed;
    motors_.ref_speed.joint5 = _speed * 1.5f;
    motors_.ref_speed.joint6 = _speed * 1.5f;
    // 检查速度是否在允许范围内
    return (_speed >= DEFAULT_JOINT_SPEED_MIN &&
            _speed <= DEFAULT_JOINT_SPEED_MAX);
}
void Safety::setJointSpeedLimit(const float _time, const Joint7D _delta_ang)
{
    for (uint8_t i = 1; i < 6; i++) {
        motors_.ref_speed._[i] = fabsf(_delta_ang.j[i] * _time);
    }

    for (uint8_t i = 1; i < 6; i++) {
        // LIMIT_MIN_MAX(motors_.ref_speed._[i], DEFAULT_JOINT_SPEED_MIN,
        //               DEFAULT_JOINT_SPEED_MAX);
    }
    motors_.ref_speed.joint1 = 0.f;
    motors_.ref_speed.joint6 = 1.5f;
    motors_.ref_speed.joint5 = 1.5;
}

} // namespace ARM