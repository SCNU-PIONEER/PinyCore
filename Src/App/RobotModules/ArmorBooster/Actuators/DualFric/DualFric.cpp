#include "DualFric.hpp"

#include "Bsp_can.hpp"

#include "DM3519.hpp"
#include "M3508.hpp"

#include "Smooth.hpp"

#include <cmath>

#include "./State/OffState.hpp"
#include "./State/OnState.hpp"

using namespace ARMORBOOSTER;
using namespace PINYMOTOR;

DualFric::DualFric()
{
    InitConfig_s fricLConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FRIC_REVERSE,
    };
    motors_.fricL = new DJIMOTOR::M3508("M3508", fricLConfig);

    InitConfig_s fricRConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN1),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = !FRIC_REVERSE,
    };
    motors_.fricR = new DJIMOTOR::M3508("M3508", fricRConfig);

    this->stateFactory_.addState(FSMState_e::ON, std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF, std::make_unique<OffState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::OFF));
}

void DualFric::stop()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void DualFric::enter()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void DualFric::update(void *_param)
{
    if (xQueueReceive((((MsgBus_s *)_param)->armorBoosterQueue), &msg, 0) == pdTRUE) {
    };

    // update state
    this->stateFactory_.update();

    fricLvel_ = motors_.fricL->vel();
    fricRvel_ = motors_.fricR->vel();
    fricLrpm_ = motors_.fricL->data().spdRpm;
    fricRrpm_ = motors_.fricR->data().spdRpm;
}

void DualFric::ctrl(float _vel)
{
    for (uint8_t i = 0; i < 2; ++i) {
        float velErr = _vel - velFltr_[i].process(motors_._[i]->vel());
        float pidOut = fricPid_[i].calc(velErr, 0);
        motors_._[i]->cmdTorq(pidOut);
    }
}

void DualFric::on()
{
    float constTarget = FAST_FRIC_RATE;
    SMOOTH::slopeFollowing(&constTarget, &targetVel_, 1.f);
    ctrl(targetVel_);
}

void DualFric::off()
{
    if ((std::fabs(motors_.fricL->vel()) < FRIC_STOP_THRESHOLD) &&
        (std::fabs(motors_.fricR->vel()) < FRIC_STOP_THRESHOLD)) {
        stop();
    } else {
        float constZero = 0;
        SMOOTH::slopeFollowing(&constZero, &targetVel_, 5.f);
        ctrl(targetVel_);
    }
}
