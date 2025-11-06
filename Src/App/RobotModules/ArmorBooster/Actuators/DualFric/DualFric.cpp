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

IncrementalPid fricLPid{ 0.8f, 0.004f, 0.f, 0.157f };
IncrementalPid fricRPid{ fricLPid };

DualFric::DualFric()
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

    this->stateFactory_.addState(FSMState_e::ON,
                                 std::make_unique<OnState>(this));
    this->stateFactory_.addState(FSMState_e::OFF,
                                 std::make_unique<OffState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::OFF));

    fricPid_[0] = &fricLPid;
    fricPid_[1] = &fricRPid;
}

void DualFric::stop()
{
    fricPid_[0]->reset();
    fricPid_[1]->reset();

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

void DualFric::on()
{
    for (uint8_t i = 0; i < 2; i++) {
        float torq = fricPid_[i]->calc(
                SLOW_FRIC_RATE, velFltr_[i].process(motors_._[i]->vel()));
        motors_._[i]->cmdTorq(torq);
    }
}

void DualFric::off()
{
    if (std::fabs(motors_.fricL->vel() + motors_.fricR->vel()) <
        FRIC_STOP_THRESHOLD) {
        stop();
    } else {
        float zero = 0;
        for (uint8_t i = 0; i < 2; i++) {
            float torq = fricPid_[i]->calc(
                    SLOW_FRIC_RATE, velFltr_[i].process(motors_._[i]->vel()));
            SMOOTH::slopeFollowing(&zero, &torq, 5.f);
            motors_._[i]->cmdTorq(torq);
        }
    }
}
