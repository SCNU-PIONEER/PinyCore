#include <stdint.h>
#include "Filter.hpp"
#include "PidBasic.hpp"
#include "Mecanum.hpp"

Mecanum::Mecanum()
{
    for (int i = 0; i < 4; i++) {
        // wheelPID_[i] = new incrementalPid();
    }
}

Mecanum::Mecanum(float diameter, float kxyFront, float kxyBack)
        : diameter(diameter), kxyFront(kxyFront), kxyBack(kxyBack)
{
    circumference = diameter * M_PI;
    for (int i = 0; i < 4; i++) {
        // wheelPID_[i] = new incrementalPid();
    }
}

void Mecanum::stop()
{
    chassisState.v_x = 0;
    chassisState.v_y = 0;
    chassisState.w_z = 0;

    currentWheels.M_RF = 0;
    currentWheels.M_LF = 0;
    currentWheels.M_LB = 0;
    currentWheels.M_RB = 0;

    for (int i = 0; i < 4; i++) {
        // wheelPID_[i]->reset();
    }
}

void Mecanum::update()
{
    //TODO: update currentWheels from motor encoder
}

void Mecanum::forward(WheelsState_s _refState)
{
    chassisState.v_x = circumference *
                       (currentWheels.M_LF - currentWheels.M_RF +
                        currentWheels.M_LB - currentWheels.M_RB) /
                       4 / 60;
    chassisState.v_y =
            circumference *
            (kxyBack * currentWheels.M_RF + kxyBack * currentWheels.M_LF -
             kxyFront * currentWheels.M_LB - kxyFront * currentWheels.M_RB) /
            (2 * (kxyBack + kxyFront)) / 60;
    chassisState.w_z = (diameter / 2) *
                       (currentWheels.M_RF + currentWheels.M_LF +
                        currentWheels.M_LB + currentWheels.M_RB) /
                       (2 * (kxyBack + kxyFront)) * (2.f * M_PI) / 60.f;
}


WheelsState_s Mecanum::reverse(ChassisState_s _refState)
{
    WheelsState_s refWheels;
    refWheels.M_RF =
            (60 / circumference) * (-chassisState.v_x + chassisState.v_y) +
            kxyFront / (diameter / 2) * chassisState.w_z * 60.f / (2.f * M_PI);
    refWheels.M_LF =
            (60 / circumference) * (chassisState.v_x + chassisState.v_y) +
            kxyFront / (diameter / 2) * chassisState.w_z * 60.f / (2.f * M_PI);
    refWheels.M_LB =
            (60 / circumference) * (chassisState.v_x - chassisState.v_y) +
            kxyBack / (diameter / 2) * chassisState.w_z * 60.f / (2.f * M_PI);
    refWheels.M_RB =
            (60 / circumference) * (-chassisState.v_x - chassisState.v_y) +
            kxyBack / (diameter / 2) * chassisState.w_z * 60.f / (2.f * M_PI);
    return refWheels;
}

void Mecanum::iir3speed(WheelsState_s _rawSpeed)
{
    currentWheels.M_RF = iir_filter_3(_rawSpeed.M_RF, 0);
    currentWheels.M_LF = iir_filter_3(_rawSpeed.M_LF, 1);
    currentWheels.M_LB = iir_filter_3(_rawSpeed.M_LB, 2);
    currentWheels.M_RB = iir_filter_3(_rawSpeed.M_RB, 3);
}

void Mecanum::ctrl(ChassisState_s _refState)
{
    WheelsState_s wheels_ref = reverse(_refState);

    float diff_speed[4] = { 0 };

    for (uint8_t i = 0; i < 4; i++) {
        // diff_speed[i] = wheelPID_[i]->calc(wheels_ref.M_RF, currentWheels.M_RF);
    }

    for (uint8_t i = 0; i < 4; i++) {
        //TODO: set 4 motor torque
    }

    // TODO: set motor output
}
