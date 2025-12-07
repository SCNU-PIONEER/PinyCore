#include "DualFric.hpp"

#include "Bsp_can.hpp"

#include "DM3519.hpp"

#include "Smooth.hpp"

#include <cmath>

#include "./State/OffState.hpp"
#include "./State/OnState.hpp"

using namespace ARMORBOOSTER;
using namespace PINYMOTOR;

DualFric::DualFric()
{
    InitConfig_s fricLConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::VDES,
        .offsetId = 2,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = FRIC_REVERSE,
    };
    motors_.fricL = new DMMOTOR::DM3519("DM3519", fricLConfig);

    InitConfig_s fricRConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::VDES,
        .offsetId = 3,
        .txFreq = FRIC_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = !FRIC_REVERSE,
    };
    motors_.fricR = new DMMOTOR::DM3519("DM3519", fricRConfig);

    this->stateFactory_.addState(FSMState_e::ON,
                                 std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF,
                                 std::make_unique<OffState>(this));
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
    if (xQueueReceive((((MsgBus_s *)_param)->armorBoosterQueue), &msg, 0) ==
        pdTRUE) {
    };

    // update state
    this->stateFactory_.update();
}

void DualFric::ctrl(float _vel)
{
    for (auto &i : motors_._) {
        i->cmdVel(SLOW_FRIC_RATE);
    }
}

void DualFric::on()
{
    float constTarget = SLOW_FRIC_RATE;
    SMOOTH::slopeFollowing(&constTarget, &targetVel_, 5.f);
    ctrl(targetVel_);
}

void DualFric::off()
{
    if (std::fabs(motors_.fricL->vel() + motors_.fricR->vel()) <
        FRIC_STOP_THRESHOLD) {
        stop();
    } else {
        float constZero = 0;
        SMOOTH::slopeFollowing(&constZero, &targetVel_, 5.f);
        ctrl(targetVel_);
    }
}
