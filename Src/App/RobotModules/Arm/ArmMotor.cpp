#include "ArmMotor.hpp"

using namespace ARM;

Motors::Motors() : safety(*this) {}

void Motors::update() {}

bool Motors::init() { return true; }

void Motors::stop() {}

void Motors::enable() {}

void Motors::ctrl(const Joint &_target_joints) {}
