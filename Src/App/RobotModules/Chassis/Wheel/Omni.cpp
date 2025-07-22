#include <stdint.h>
#include "Filter.hpp"
#include "PidBasic.hpp"
#include "Omni.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "M3508.hpp"

extern canHandle HCAN1;

using namespace CHASSIS;
using namespace PINYMOTOR;

Omni::Omni()
{
    for (uint8_t i = 1; i <= 4; i++) {
        InitConfig_s M3508Config = {
            reinterpret_cast<uint32_t *>(&HCAN1),
            ComType_e::CAN,
            WorkMode_e::QUAD_CURR,
            i,
            1000.0f,
            nullptr,
            std::unique_ptr<PID>(
                    new positonalPid(0.1f, 0.f, 0.f, 0.002f, 1.f, 4.f, 0.f)),
            nullptr
        };
        motor[i - 1] = new DJIMOTOR::M3508("M3508", std::move(M3508Config));
    }
}

void Omni::stop()
{
    for (float &i : refSpeed_._) {
        i = 0;
    }

    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = 0;
        motor[i]->cmd(MotorCmdType_e::OFF);
    }
}

void Omni::update()
{
    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = iir_filter_3(motor[i]->data().spdRpm, i);
    }
}

// Speed_u Omni::forward(const wheelsSpeed_u &_wSpeed) {} //TODO: waiting to set


// wheelsSpeed_u Omni::reverse(const Speed_u &_speed) {} //TODO: waiting to set

void Omni::ctrl(const Speed_u &_speed)
{
    wheelsSpeed_u refWSpeed = reverse(_speed);

    for (uint8_t i = 0; i < 4; i++) {
        motor[i]->cmd(MotorCmdType_e::SET_VEL, refWSpeed._[i]);
    }

#ifdef USE_POWERCTRL
    powerCtrl_->powerCtrl(refWSpeed._);
#endif
}
