#include <cstdint>
#include "Filter.hpp"
#include "Mecanum.hpp"
#include "M3508.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include <numbers>
#include "QuadricycleController.hpp"

extern canHandle HCAN1;

using namespace CHASSIS;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;

Mecanum::Mecanum()
{
    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s m3508Config = {
            .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
            .comType = ComType_e::CAN,
            .workMode = WorkMode_e::QUAD_CURR,
            .offsetId = i,
            .txFreq = 100.0f,
            .posPID = nullptr,
            .velPID = new IncrementalPid(0.2f, 0.005f, 0.f, 4.f, 0.f),
            .torqPID = nullptr
        };
        motor_[i - 1] = new DJIMOTOR::M3508("M3508", m3508Config);
    }

    if constexpr (USE_POWERCTRL) {
        powerCtrl_ = std::make_unique<QuadricycleController>(
                ChassisType_e::QUADRICYCLE);
    }
}

void Mecanum::stop()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void Mecanum::enter()
{
    for (auto &i : motor_) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void Mecanum::update()
{
    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = iir_filter_3(motor_[i]->data().spdRpm, i);
    }
    this->curSpeed_ = forward(wSpeed_);
}

Speed_u Mecanum::forward(const WheelsSpeed_u &_wSpeed)
{
    Speed_u speed;
    speed.v_x = W_CIRCUMFERENCE *
                (_wSpeed.M_LF - _wSpeed.M_RF + _wSpeed.M_LB - _wSpeed.M_RB) /
                4.f / 60.f;
    speed.v_y = W_CIRCUMFERENCE *
                (BACK_R * _wSpeed.M_RF + BACK_R * _wSpeed.M_LF -
                 FRONT_R * _wSpeed.M_LB - FRONT_R * _wSpeed.M_RB) /
                (2.f * (BACK_R + FRONT_R)) / 60.f;
    speed.w_z = (W_DIAMETER / 2.f) *
                (_wSpeed.M_RF + _wSpeed.M_LF + _wSpeed.M_LB + _wSpeed.M_RB) /
                (2.f * (BACK_R + FRONT_R)) * (2.f * F_PI) / 60.f;
    return speed;
}


WheelsSpeed_u Mecanum::reverse(const Speed_u &_speed)
{
    WheelsSpeed_u refWheels;
    refWheels.M_RF = (60.f / W_CIRCUMFERENCE) * (-_speed.v_x + _speed.v_y) +
                     FRONT_R / (W_DIAMETER / 2.f) * _speed.w_z * 30.f / F_PI;
    refWheels.M_LF = (60.f / W_CIRCUMFERENCE) * (_speed.v_x + _speed.v_y) +
                     FRONT_R / (W_DIAMETER / 2.f) * _speed.w_z * 30.f / F_PI;
    refWheels.M_LB = (60.f / W_CIRCUMFERENCE) * (_speed.v_x - _speed.v_y) +
                     BACK_R / (W_DIAMETER / 2.f) * _speed.w_z * 30.f / F_PI;
    refWheels.M_RB = (60.f / W_CIRCUMFERENCE) * (-_speed.v_x - _speed.v_y) +
                     BACK_R / (W_DIAMETER / 2.f) * _speed.w_z * 30.f / F_PI;
    return refWheels;
}

void Mecanum::ctrl(const Speed_u &_refSpeed)
{
    WheelsSpeed_u refWSpeed = reverse(_refSpeed);

    for (uint8_t i = 0; i < 4; i++) {
        motor_[i]->cmdVel(refWSpeed._[i]);
    }

    if constexpr (USE_POWERCTRL) {
        powerCtrl_->powerCtrl(refWSpeed._);
    }
}
