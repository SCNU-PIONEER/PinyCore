#include <stdint.h>
#include "filter.hpp"
#include "pidBasic.hpp"
#include "omni.hpp"

Omni::Omni()
{
    for (int i = 0; i < 4; i++) {
        wheelPid_[i] = new PidBasic();
    }
}

Omni::Omni(float diameter, float kxyFront, float kxyBack)
        : diameter(diameter), kxyFront(kxyFront), kxyBack(kxyBack)
{
    circumference = diameter * M_PI;
    for (int i = 0; i < 4; i++) {
        wheelPid_[i] = new PidBasic();
    }
}

void Omni::stop()
{
    chassisState.v_x = 0;
    chassisState.v_y = 0;
    chassisState.w_z = 0;

    currentWheels.M_RF = 0;
    currentWheels.M_LF = 0;
    currentWheels.M_LB = 0;
    currentWheels.M_RB = 0;

    for (int i = 0; i < 4; i++) {
        wheelPid_[i]->reset();
    }
}

void Omni::update()
{
    //TODO: update currentWheels from motor encoder
}

void Omni::forward(WheelsState_s _refState) {}


WheelsState_s Omni::reverse(ChassisState_s _refState) {}

void Omni::iir3speed(WheelsState_s _rawSpeed)
{
    currentWheels.M_RF = iir_filter_3(_rawSpeed.M_RF, 0);
    currentWheels.M_LF = iir_filter_3(_rawSpeed.M_LF, 1);
    currentWheels.M_LB = iir_filter_3(_rawSpeed.M_LB, 2);
    currentWheels.M_RB = iir_filter_3(_rawSpeed.M_RB, 3);
}

void Omni::ctrl(ChassisState_s _refState)
{
    WheelsState_s wheels_ref = reverse(_refState);

    float diff_speed[4] = { 0 };

    for (uint8_t i = 0; i < 4; i++) {
        diff_speed[i] = wheelPid_[i]->calc(wheels_ref.M_RF, currentWheels.M_RF);
    }

    for (uint8_t i = 0; i < 4; i++) {
        //TODO: set 4 motor torque
    }

    // TODO: set motor output
}
