#include "IMotor.hpp"

#include "MotorManager.hpp"

#include "StmLog.hpp"

#include "MotorCommonMacros.hpp"

using namespace PINYMOTOR;

IMotor::IMotor(const char _name[16], InitConfig_s _config)
        : posPID_(_config.posPID)
        , velPID_(_config.velPID)
        , torqPID_(_config.torqPID)
        , globalState(GlobalState_e::UNRECOGNIZED)
{
    regInfo_.uid = MotorManager::instance()->assignId();
    regInfo_.pComHandle = _config.pComHandle;
    regInfo_.comType = _config.comType;
    regInfo_.workMode = _config.workMode;
    regInfo_.offsetId = _config.offsetId;
    strncpy(regInfo_.name, _name, 16);

    AUX_.txFreq = _config.txFreq;

    regInfo_.isReverse = _config.isReverse;

    registerMotor();
    this->cmd_.clear();
    memset(&data_, 0, sizeof(Data_s));
}

bool IMotor::checkSend()
{
    if ((xTaskGetTickCount() - AUX_.lastSendTick) >=
        pdMS_TO_TICKS(1000.f / AUX_.txFreq)) {
        AUX_.lastSendTick = xTaskGetTickCount();
        return true;
    } else {
        return false;
    }
}

void IMotor::calcRecvFreq()
{
    uint32_t dt = xTaskGetTickCount() - AUX_.lastRecvTick; // ms
    if (dt < 1000) {
        return;
    } else {
        AUX_.rxFreq = static_cast<float>(AUX_.recvCnt) /
                      (static_cast<float>(dt) / 1000.f);
        AUX_.recvCnt = 0;
        AUX_.lastRecvTick = xTaskGetTickCount();
    }
    if (AUX_.rxFreq < AUX_.txFreq * 0.5f) {
        this->globalState = GlobalState_e::OFFLINE;
    } else {
        this->globalState = GlobalState_e::ONLINE;
    }
}

MotorTypeDef_e IMotor::registerMotor()
{
    auto *instance = MotorManager::instance();
    auto it = instance->motors().find(uid());
    if (it != instance->motors().end()) {
        return false;
    }
    instance->motors().insert({ uid(), this });
    return 0;
}

MotorTypeDef_e IMotor::cancelMotor()
{
    auto *instance = MotorManager::instance();
    auto it = instance->motors().find(uid());
    if (it == instance->motors().end()) {
        return false;
    }
    instance->motors().erase(it);
    return 0;
}

MotorTypeDef_e IMotor::cmd(MotorCmdType_e _type)
{
    if (_type == MotorCmdType_e::ON || _type == MotorCmdType_e::OFF) {
        CmdBus_s cmd = { .cmdType = _type };
        memcpy(&AUX_.cmdBuf, &cmd, sizeof(CmdBus_s));
        return 0;
    } else {
        LOG::error(
                "IMotor",
                " %s: none of cmd are supported in this function except ON and OFF",
                regInfo_.name);
        return 1;
    }
}

MotorTypeDef_e IMotor::cmdProto(CmdBus_s &_cmd)
{
    memcpy(&AUX_.cmdBuf, &_cmd, sizeof(CmdBus_s));
    return 0;
}

MotorTypeDef_e IMotor::cmdMIT(float _pos, float _vel, float _torq,
                              float _velMax, float _posMin, float _posMax)
{
    CmdBus_s cmdBuf = {};
    cmdBuf.cmdType = MotorCmdType_e::SET_MIT;
    cmdBuf.posCmd = _pos;
    cmdBuf.velCmd = _vel;
    cmdBuf.torqCmd = _torq;
    cmdBuf.velMax = _velMax;
    cmdBuf.posMin = _posMin;
    cmdBuf.posMax = _posMax;
    return cmdProto(cmdBuf);
}

MotorTypeDef_e IMotor::cmdPosVel(float _pos, float _vel, float _velMax,
                                 float _posMin, float _posMax)
{
    CmdBus_s cmdBuf = {};
    cmdBuf.cmdType = MotorCmdType_e::SET_POSVEL;
    cmdBuf.posCmd = _pos;
    cmdBuf.velCmd = _vel;
    cmdBuf.velMax = _velMax;
    cmdBuf.posMin = _posMin;
    cmdBuf.posMax = _posMax;
    return cmdProto(cmdBuf);
}

MotorTypeDef_e IMotor::cmdPos(float _pos, float _velMax, float _posMin,
                              float _posMax)
{
    CmdBus_s cmdBuf = {};
    cmdBuf.cmdType = MotorCmdType_e::SET_POS;
    cmdBuf.posCmd = _pos;
    cmdBuf.velMax = _velMax;
    cmdBuf.posMin = _posMin;
    cmdBuf.posMax = _posMax;
    return cmdProto(cmdBuf);
}

MotorTypeDef_e IMotor::cmdVel(float _vel, float _velMax)
{
    CmdBus_s cmdBuf = {};
    cmdBuf.cmdType = MotorCmdType_e::SET_VEL;
    cmdBuf.velCmd = _vel;
    cmdBuf.velMax = _velMax;
    cmdBuf.posMin = 0.f; // no limit
    cmdBuf.posMax = 0.f; // no limit
    return cmdProto(cmdBuf);
}

MotorTypeDef_e IMotor::cmdTorq(float _torq)
{
    CmdBus_s cmdBuf = {};
    cmdBuf.cmdType = MotorCmdType_e::SET_TORQ;
    cmdBuf.torqCmd = _torq;
    cmdBuf.velMax = -1.f; // no limit
    cmdBuf.posMin = 0.f;  // no limit
    cmdBuf.posMax = 0.f;  // no limit
    return cmdProto(cmdBuf);
}

MotorTypeDef_e IMotor::cmdElec(float _elec)
{
    CmdBus_s cmdBuf = {};
    cmdBuf.cmdType = MotorCmdType_e::SET_ELEC;
    cmdBuf.elecCmd = _elec;
    cmdBuf.velMax = -1.f; // no limit
    cmdBuf.posMin = 0.f;  // no limit
    cmdBuf.posMax = 0.f;  // no limit
    return cmdProto(cmdBuf);
}

void IMotor::parseCmd()
{
    this->cmd_.curCmdType = AUX_.cmdBuf.cmdType;

    cmd_.prevSW = cmd_.SW;

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::ON: {
        this->cmd_.updateSW(true);
        break;
    }
    case MotorCmdType_e::OFF: {
        this->cmd_.updateSW(false);
        break;
    }
    case MotorCmdType_e::SET_ELEC: {
        this->cmd_.elec = AUX_.cmdBuf.elecCmd;
        break;
    }
    case MotorCmdType_e::SET_TORQ: {
        this->cmd_.torq = AUX_.cmdBuf.torqCmd;
        break;
    }
    case MotorCmdType_e::SET_VEL: {
        this->cmd_.vel = AUX_.cmdBuf.velCmd;
        break;
    }
    case MotorCmdType_e::SET_POS: {
        this->cmd_.pos = AUX_.cmdBuf.posCmd;
        break;
    }
    case MotorCmdType_e::SET_POSVEL: {
        this->cmd_.pos = AUX_.cmdBuf.posCmd;
        this->cmd_.vel = AUX_.cmdBuf.velCmd;
        break;
    }
    case MotorCmdType_e::SET_MIT: {
        this->cmd_.pos = AUX_.cmdBuf.posCmd;
        this->cmd_.vel = AUX_.cmdBuf.velCmd;
        this->cmd_.torq = AUX_.cmdBuf.torqCmd;
        break;
    }
    default: {
        LOG::error("IMotor", " %s: cmd type is not supported", regInfo_.name);
    }
    }

    clampVel(AUX_.cmdBuf.velMax);
    clampPos(AUX_.cmdBuf.posMin, AUX_.cmdBuf.posMax);

    if (!(this->cmd_.velMax < 0.f)) {
        this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                    this->cmd_.velMax);
    }
    if (this->cmd_.posMax != this->cmd_.posMin) {
        this->cmd_.pos =
                clampArc(this->cmd_.pos, this->cmd_.posMin, this->cmd_.posMax);
    }
}

void IMotor::clampVel(float _velMax) { this->cmd_.velMax = _velMax; }

void IMotor::clampPos(float _posMin, float _posMax)
{
    this->cmd_.posMin = _posMin;
    this->cmd_.posMax = _posMax;
}

void IMotor::disableClampPos() { clampPos(0.f, 0.f); }

uint16_t IMotor::uid() const { return regInfo_.uid; }

const Data_s &IMotor::data() const { return data_; }

void IMotor::setZeroAng()
{
    // TODO: setZeroAng need a semaphore to protect
    data_.zeroAng = data_.rawAng;
}

void IMotor::setZeroAng(float _zeroAng)
{
    // TODO: setZeroAng need a semaphore to protect
    // _zeroAng is a rawAng in "current reverse situation"
    data_.zeroAng = _zeroAng;
}
float IMotor::getCmdCurr()
{
    if (regInfo_.workMode == WorkMode_e::QUAD_VOLT) {
        // TODO: RLS volt ctrl
        return 0.f;
    } else {
        return cmd_.elec;
    }
}

float IMotor::txBaseId() const
{
    return static_cast<float>(regInfo_.model.txBaseId);
}

float IMotor::rxBaseId() const
{
    return static_cast<float>(regInfo_.model.rxBaseId);
}

float IMotor::rr() const { return regInfo_.model.reductionRatio; }

float IMotor::measureMax() const
{
    return static_cast<float>(regInfo_.model.measureMax);
}

float IMotor::measureMin() const
{
    return static_cast<float>(regInfo_.model.measureMin);
}

float IMotor::span() const
{
    return static_cast<float>(regInfo_.model.measureMax -
                              regInfo_.model.measureMin);
}

float IMotor::txFreq() const { return AUX_.txFreq; }

float IMotor::rxFreq() const { return AUX_.rxFreq; }

float IMotor::ang() const { return data_.ang; }

float IMotor::center() const { return data_.zeroAng; }

float IMotor::pos() const { return data_.multipCirAng; }

float IMotor::posNorm() const { return data_.singleCirAng; }

float IMotor::vel() const { return data_.spdRadps; }

float IMotor::torq() const { return data_.torq; }

void IMotor::overrideMeasureMax(float _newMeasureMax)
{
    regInfo_.model.measureMax = static_cast<uint16_t>(_newMeasureMax);
}

void IMotor::overrideMeasureMin(float _newMeasureMin)
{
    regInfo_.model.measureMin = static_cast<uint16_t>(_newMeasureMin);
}

const char *IMotor::getName() const { return regInfo_.name; }
