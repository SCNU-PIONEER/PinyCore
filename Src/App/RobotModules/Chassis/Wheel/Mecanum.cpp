#include <stdint.h>
#include "Filter.hpp"
#include "Mecanum.hpp"
#include "M3508.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include "QuadricycleController.hpp"

extern canHandle HCAN1;

using namespace CHASSIS;
using namespace PINYMOTOR;

Mecanum::Mecanum()
{
    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s M3508Config = { reinterpret_cast<uint32_t *>(&HCAN1),
                                     ComType_e::CAN,
                                     WorkMode_e::QUAD_CURR,
                                     i,
                                     100.0f,
                                     nullptr,
                                     std::unique_ptr<PID>(new incrementalPid(
                                             0.2f, 0.005f, 0.f, 4.f, 0.f)),
                                     nullptr };
        motor[i - 1] = new DJIMOTOR::M3508("M3508", std::move(M3508Config));
    }

    if constexpr (USE_POWERCTRL) {
        powerCtrl_ = std::make_unique<QuadricycleController>(
                ChassisType_e::Quadricycle);
    }
}

Mecanum::Mecanum(float diameter, float kxyFront, float kxyBack)
        : diameter(diameter), kxyFront(kxyFront), kxyBack(kxyBack)
{
    extern canHandle HCAN1;
    circumference = diameter * M_PI;
    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s M3508Config = {
            reinterpret_cast<uint32_t *>(&HCAN1),
            ComType_e::CAN,
            WorkMode_e::QUAD_CURR,
            i,
            500.0f,
            nullptr,
            std::unique_ptr<PID>(
                    new positonalPid(32.f, 0.04f, 0.f, 0.002f, 1.f, 20.f, 0.f)),
            nullptr
        };
        motor[i - 1] = new DJIMOTOR::M3508("M3508", std::move(M3508Config));
    }
}

void Mecanum::stop()
{
    for (uint8_t i = 0; i < 4; i++) {
        motor[i]->cmd(MotorCmdType_e::OFF);
    }
}

void Mecanum::enter()
{
    for (uint8_t i = 0; i < 4; i++) {
        motor[i]->cmd(MotorCmdType_e::ON);
    }
}

void Mecanum::update()
{
    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = iir_filter_3(motor[i]->data().spdRpm, i);
    }
}

speed_u Mecanum::forward(const wheelsSpeed_u &_wSpeed)
{
    speed_u speed;
    speed.v_x = circumference *
                (_wSpeed.M_LF - _wSpeed.M_RF + _wSpeed.M_LB - _wSpeed.M_RB) /
                4 / 60;
    speed.v_y = circumference *
                (kxyBack * _wSpeed.M_RF + kxyBack * _wSpeed.M_LF -
                 kxyFront * _wSpeed.M_LB - kxyFront * _wSpeed.M_RB) /
                (2 * (kxyBack + kxyFront)) / 60;
    speed.w_z = (diameter / 2) *
                (_wSpeed.M_RF + _wSpeed.M_LF + _wSpeed.M_LB + _wSpeed.M_RB) /
                (2 * (kxyBack + kxyFront)) * (2.f * M_PI) / 60.f;
    return speed;
}


wheelsSpeed_u Mecanum::reverse(const speed_u &_speed)
{
    wheelsSpeed_u refWheels;
    refWheels.M_RF = (60 / circumference) * (-_speed.v_x + _speed.v_y) +
                     kxyFront / (diameter / 2) * _speed.w_z * 30.f / M_PI;
    refWheels.M_LF = (60 / circumference) * (_speed.v_x + _speed.v_y) +
                     kxyFront / (diameter / 2) * _speed.w_z * 30.f / M_PI;
    refWheels.M_LB = (60 / circumference) * (_speed.v_x - _speed.v_y) +
                     kxyBack / (diameter / 2) * _speed.w_z * 30.f / M_PI;
    refWheels.M_RB = (60 / circumference) * (-_speed.v_x - _speed.v_y) +
                     kxyBack / (diameter / 2) * _speed.w_z * 30.f / M_PI;
    return refWheels;
}

void Mecanum::ctrl(const speed_u &_speed)
{
    wheelsSpeed_u refWSpeed = reverse(_speed);

    for (uint8_t i = 0; i < 4; i++) {
        motor[i]->cmd(MotorCmdType_e::SET_VEL, refWSpeed._[i]);
    }

    if constexpr (USE_POWERCTRL) {
        powerCtrl_->powerCtrl(refWSpeed._);
    }
}
