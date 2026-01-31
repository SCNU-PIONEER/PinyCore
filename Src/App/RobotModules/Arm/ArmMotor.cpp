#include "ArmMotor.hpp"

using namespace ARM;

Motors::Motors() : safety(*this) {}

void Motors::update() {}

bool Motors::init() { return true; }

void Motors::stop() {}

void Motors::enable() {}

void Motors::ctrl(const Joint10 &_target_joints)
{
    for (int i = 0; i < _target_joints.size(); i++) {
        motors.all_motors[i]->cmdPosVel(_target_joints[i], ref_speed._[i]);
    }
}
