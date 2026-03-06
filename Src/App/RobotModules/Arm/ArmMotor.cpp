#include "ArmMotor.hpp"
#include "Soc.hpp"
#include "DM8009.hpp"
#include "DM4340.hpp"
#include "DM4310.hpp"

using namespace ARM;

// TODO: 电机初始化，电机ID，电机类型等
Motors::Motors(Joint10 _angleMin, Joint10 _angleMax) : safety(*this, _angleMin, _angleMax) {};

void Motors::update()
{
    size_t index = 0;
    for (auto &motor : motors.all_motors) {
        /* singleCirAng range: 0 ~ 2PI, we need range: -pi ~ pi */
        if (motor->data().singleCirAng >= std::numbers::pi_v<float>) {
            currentJoints[index] = motor->data().singleCirAng - (2 * std::numbers::pi_v<float>);
        } else {
            currentJoints[index] = motor->data().singleCirAng;
        }
        index++;
    }
}

// TODO: 电机复位
bool Motors::init() { return true; }

void Motors::stop()
{
    for (const auto &motor : motors.all_motors) {
        motor->cmd(MotorCmdType_e::OFF);
    }
}

void Motors::enable()
{
    for (const auto &motor : motors.all_motors) {
        motor->cmd(MotorCmdType_e::ON);
    }
}

bool Motors::isMoving(const Joint10 &_target)
{
    Joint10 deltaJoints = _target - currentJoints;
    float maxDeltaAngle = AbsMaxOf10(deltaJoints);
    return maxDeltaAngle < 0.02f;
}

void Motors::ctrl(const Joint10 &_targetJoints)
{
    Joint10 temp = _targetJoints;

    safety.setAllAngleLimit(temp);

    size_t index = 0;
    for (const auto &motor : motors.all_motors) {
        motor->cmdPos(temp[index]);
        index++;
    }
}

void Motors::handYaw(Joint10 &_msgTargetJoints)
{
    _msgTargetJoints[4] = _msgTargetJoints[4];
    _msgTargetJoints[5] = -_msgTargetJoints[4];
    _msgTargetJoints[9] = _msgTargetJoints[8];
    _msgTargetJoints[10] = -_msgTargetJoints[8];
}

void Motors::handPitch(Joint10 &_msgTargetJoints)
{
    _msgTargetJoints[4] = _msgTargetJoints[5];
    _msgTargetJoints[5] = _msgTargetJoints[5];
    _msgTargetJoints[9] = _msgTargetJoints[9];
    _msgTargetJoints[10] = _msgTargetJoints[9];
}
