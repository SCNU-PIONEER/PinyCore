#include <cstdint>
#include "IIR.hpp"
#include "Omni.hpp"
#include "M3508.hpp"
#include "sdkconfig.h"
#include "Bsp_can.hpp"
#include "PidBasic.hpp"
#include <numbers>
#include "QuadricycleController.hpp"
#include "OSnew.hpp"

#include "./State/StopState.hpp"
#include "./State/RunState.hpp"

extern canHandle HCAN1;

using namespace CHASSIS::OMNI;
using namespace PINYMOTOR;

constexpr float F_PI = std::numbers::pi_v<float>;

Omni::Omni()
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
        motors_._[i - 1] = new DJIMOTOR::M3508("M3508", m3508Config);
    }

    stateFactory.addState(static_cast<uint8_t>(FSMState_e::RUN),
                          std::make_unique<RunState>(this));
    stateFactory.addState(static_cast<uint8_t>(FSMState_e::STOP),
                          std::make_unique<StopState>(this));
    stateFactory.init(
            stateFactory.getNextState(static_cast<uint8_t>(FSMState_e::STOP)));

    if constexpr (APP_USE_POWERCTRL) {
        // TODO:
    }
}

void Omni::stop()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void Omni::enter()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void Omni::update()
{
    for (uint8_t i = 0; i < 4; i++) {
        wSpeed_._[i] = iir_filter_3(motors_._[i]->data().spdRpm, i);
    }
    this->curSpeed_ = forward(wSpeed_);

    this->stateFactory.update();
}

Speed_u Omni::forward(const WheelsSpeed_u &_wSpeed)
{
    Speed_u speed{};
    // TODO:
    return speed;
}


WheelsSpeed_u Omni::reverse(const Speed_u &_speed)
{
    WheelsSpeed_u refWheels{};
    // TODO:
    return refWheels;
}

void Omni::ctrl(const Speed_u &_refSpeed)
{
    WheelsSpeed_u refWSpeed = reverse(_refSpeed);

    for (uint8_t i = 0; i < 4; i++) {
        motors_._[i]->cmdVel(refWSpeed._[i]);
    }

    if constexpr (APP_USE_POWERCTRL) {
        // TODO:
    }
}
