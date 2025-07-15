#include <stdint.h>
#include "Filter.hpp"
#include "PidBasic.hpp"
#include "Omni.hpp"

Omni::Omni()
{
    for (int i = 0; i < 4; i++) {
        // wheelPID_[i] = new incrementalPid();
    }
}

Omni::Omni(float diameter, float kxyFront, float kxyBack)
        : diameter(diameter), kxyFront(kxyFront), kxyBack(kxyBack)
{
    circumference = diameter * M_PI;
    for (int i = 0; i < 4; i++) {
        // wheelPID_[i] = new incrementalPid();
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
        wheelPID_[i]->reset();
    }
}

void Omni::update()
{
    //TODO: update currentWheels from motor encoder
}

void Omni::forward(WheelsState_u _refState) {}


WheelsState_u Omni::reverse(ChassisState_s _refState) {}

void Omni::iir3speed(WheelsState_u _rawSpeed)
{
    currentWheels.M_RF = iir_filter_3(_rawSpeed.M_RF, 0);
    currentWheels.M_LF = iir_filter_3(_rawSpeed.M_LF, 1);
    currentWheels.M_LB = iir_filter_3(_rawSpeed.M_LB, 2);
    currentWheels.M_RB = iir_filter_3(_rawSpeed.M_RB, 3);
}

void Omni::ctrl(ChassisState_s _refState)
{
    WheelsState_u wheels_ref = reverse(_refState);

    float diff_speed[4] = { 0 };

    for (uint8_t i = 0; i < 4; i++) {
    }

    for (uint8_t i = 0; i < 4; i++) {
        //TODO: set 4 motor torque
    }

    // TODO: set motor output
}
