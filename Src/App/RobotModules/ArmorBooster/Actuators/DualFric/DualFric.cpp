#include "DualFric.hpp"

#include "Bsp_can.hpp"

#include "M3508.hpp"

#include "Smooth.hpp"

#include "PidBasic.hpp"

#include <cmath>

#include "./State/OffState.hpp"
#include "./State/OnState.hpp"

using namespace ARMORBOOSTER;
using namespace PINYMOTOR;

PositionalPid fricLPid{ 0.003f, 0.001f, 2.0e-5f, 0.001f, 0.05f, 0.157f };
PositionalPid fricRPid{ fricLPid };
PositionalPid fricDelPid{ 0.f, 0.f, 0.f, 0.001f, 0.f, 0.1f };

DualFric::DualFric() : fricDelPid_(&fricDelPid)
{
    InitConfig_s fricLConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 1,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FRIC_REVERSE,
    };
    motors_.fricL = new DJIMOTOR::M3508("M3508", fricLConfig);

    InitConfig_s fricRConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 2,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = !FRIC_REVERSE,
    };
    motors_.fricR = new DJIMOTOR::M3508("M3508", fricRConfig);

    this->stateFactory_.addState(FSMState_e::ON, std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF, std::make_unique<OffState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::OFF));

    fricPid_[0] = &fricLPid;
    fricPid_[1] = &fricRPid;
}

void DualFric::stop()
{
    fricPid_[0]->reset();
    fricPid_[1]->reset();
    fricDelPid_->reset();

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
}

void DualFric::ctrl(float _vel)
{
    float velL = velFltr_[0].process(motors_._[0]->vel());
    float velR = velFltr_[1].process(motors_._[1]->vel());
    float tau = fricDelPid_->calc(velL, velR);
    motors_._[0]->cmdTorq(fricPid_[0]->calc(_vel, velL) - tau);
    motors_._[1]->cmdTorq(fricPid_[1]->calc(_vel, velR) + tau);
}

void DualFric::on()
{
    float constTarget = FAST_FRIC_RATE;
    SMOOTH::slopeFollowing(&constTarget, &targetVel_, 0.1f);
    ctrl(targetVel_);
}

void DualFric::off()
{
    if (std::fabs(motors_.fricL->vel() + motors_.fricR->vel()) < FRIC_STOP_THRESHOLD) {
        stop();
    } else {
        float constZero = 0;
        SMOOTH::slopeFollowing(&constZero, &targetVel_, 0.1f);
        ctrl(targetVel_);
    }
}
