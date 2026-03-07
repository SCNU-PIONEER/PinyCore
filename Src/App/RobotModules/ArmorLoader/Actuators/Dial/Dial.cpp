#include "Dial.hpp"

#include <cmath>

#include "Bsp_can.hpp"
#include "Referee.hpp"
#include "M2006.hpp"

#include "./State/AutoState.hpp"
#include "./State/BlockState.hpp"
#include "./State/SingleState.hpp"
#include "./State/StopState.hpp"

using namespace ARMORLOADER;
using namespace PINYMOTOR;

void ShooterCtrlRxPacket::registerCallback()
{
    Can::instance().registerCallback(&HCAN3, uid(), [this](const uint8_t *_rxBuf) {
        xQueueSendFromISR(this->queue_, _rxBuf, nullptr);
    });
    LOG::info("ShooterCtrlRxPacket", " Receive cb registered, uId:0x%hx", uid());
}

Dial::Dial()
{
    InitConfig_s dialConfig = {
        .pComHandle = reinterpret_cast<uint32_t *>(&HCAN2),
        .comType = ComType_e::CAN,
        .workMode = WorkMode_e::QUAD_CURR,
        .offsetId = 3,
        .txFreq = DIAL_MOTOR_FREQ,
        .posPID = nullptr,
        .velPID = nullptr,
        .isReverse = DIAL_REVERSE,
    };
    motors_.dial = new DJIMOTOR::M2006("M2006", dialConfig);
    motors_.dial->overrideReductionRatio(DJIMOTOR::M2006::ORIGINAL_GEARBOX_RR * OUTER_RR);

    this->stateFactory_.addState(FSMState_e::AUTO, std::make_unique<AutoState>(this));
    this->stateFactory_.addState(FSMState_e::BLOCK, std::make_unique<BlockState>(this));
    this->stateFactory_.addState(FSMState_e::SINGLE, std::make_unique<SingleState>(this));
    this->stateFactory_.addState(FSMState_e::STOP, std::make_unique<StopState>(this));
    this->stateFactory_.init(this->stateFactory_.getNextState(FSMState_e::STOP));
}

void Dial::stop()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::OFF);
    }
}

void Dial::enter()
{
    for (auto &i : motors_._) {
        i->cmd(MotorCmdType_e::ON);
    }
}

void Dial::update(void *_param)
{
    msg.state = static_cast<FSMState_e>(shooterCtrlRxPacket_.data().state);

#if APP_USE_REFEREE
    remainHeat = referee->receiver.getRefereeData().gameRobotStatus.shooterHeatLimit -
                 referee->receiver.getRefereeData().powerHeatData.shooter_17mmBarrelHeat;
    refereeFire_ = remainHeat < 0 || remainHeat > 40;
#else
    refereeFire_ = true;
#endif

    msg.fire = shooterCtrlRxPacket_.data().fire && refereeFire_;

    // User-defined processing logic goes here ↓

    vel_ = velFltr_.process(dial()->vel());

    torq_ = motors_.dial->torq();

    if (std::fabs(dial()->data().curr) > BLOCK_CURRENT && std::fabs(dial()->vel()) < BLOCK_SPEED_MIN) {
        blockCnt = blockCnt < BLOCK_CNT_MAX ? blockCnt + 1 : BLOCK_CNT_MAX;
    } else {
        blockCnt = blockCnt > 0 ? blockCnt - 1 : 0;
    }

    // update state
    this->stateFactory_.update();
}

IMotor *Dial::dial() const { return motors_.dial; }

void Dial::ctrl(float _torq) { dial()->cmdTorq(_torq); }

void Dial::ctrl(PID *_velPID, float _vel) { dial()->cmdTorq(_velPID->calc(_vel, vel_)); }

void Dial::ctrlSingl(PID *_posPID, PID *_velPID, float _ang)
{
    float v = _velPID->calc(_ang, dial()->pos());
    dial()->cmdTorq(_velPID->calc(v, vel_));
}

void Dial::ctrlMulti(PID *_posPID, PID *_velPID, float _ang)
{
    float v = _velPID->calc(_ang, dial()->posNorm());
    dial()->cmdTorq(_velPID->calc(v, vel_));
}

bool Dial::isBlock() const { return blockCnt >= BLOCK_CNT_MAX; }
