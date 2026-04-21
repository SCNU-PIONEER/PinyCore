#include "DMMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DMMOTOR;

void DMMotor::serializeMITMsg(MITMsg_s &_msgMIT, TxBus &_txBuf)
{
    _txBuf.len = 8;
    _txBuf.data[0] = static_cast<uint8_t>((_msgMIT.exptScale & 0xFF00) >> 8);
    _txBuf.data[1] = static_cast<uint8_t>(_msgMIT.exptScale & 0x00FF);
    _txBuf.data[2] = static_cast<uint8_t>((_msgMIT.exptVel & 0xFF0) >> 4);
    _txBuf.data[3] = static_cast<uint8_t>((_msgMIT.exptVel & 0x00F) << 4 | ((_msgMIT.Kp & 0xF00) >> 8));
    _txBuf.data[4] = static_cast<uint8_t>(_msgMIT.Kp & 0x0FF);
    _txBuf.data[5] = static_cast<uint8_t>((_msgMIT.Kd & 0xFF0) >> 4);
    _txBuf.data[6] = static_cast<uint8_t>((_msgMIT.Kd & 0x00F) << 4 | ((_msgMIT.torqueForward & 0xF00) >> 8));
    _txBuf.data[7] = static_cast<uint8_t>(_msgMIT.torqueForward & 0x0FF);
}

void DMMotor::updateCtrlMode()
{
    switch (regInfo_.workMode) {
    case WorkMode_e::MIT_TT:
        convert = &DMMotor::convertMitTt;
        this->ctrlId_ = this->canId();
        break;
    case WorkMode_e::MIT_VDESPDES:
        convert = &DMMotor::convertMitVdesPdes;
        this->ctrlId_ = this->canId();
        break;
    case WorkMode_e::MIT_VDES: {
        convert = &DMMotor::convertMitVdes;
        this->ctrlId_ = this->canId();
        break;
    }
    case WorkMode_e::PDESVDES: {
        convert = &DMMotor::convertPdesVdes;
        this->ctrlId_ = this->canId() + 0x100;
        break;
    }
    case WorkMode_e::VDES: {
        convert = &DMMotor::convertVdes;
        this->ctrlId_ = this->canId() + 0x200;
        break;
    }
    case WorkMode_e::EMIT: {
        convert = &DMMotor::convertEmit;
        this->ctrlId_ = this->canId() + 0x300;
        break;
    }
    default: {
        convert = &DMMotor::convertDefault;
        LOG::error("DMMotor", " %s: this mode is not supported", regInfo_.name);
        this->ctrlId_ = 0xFFFF;
        break;
    }
    }
}

void DMMotor::switchCtrlMode(WorkMode_e _workMode)
{
    regInfo_.workMode = _workMode;
    updateCtrlMode();
}

void DMMotor::convertMitTt(TxBus &_txBuf)
{
    MITMsg_s msgMIT = {};
    msgMIT.Kp = msgMIT.Kd = 0;

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_TORQ:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DMMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    this->cmd_.torq = std::clamp(regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq, -status_.TMax, status_.TMax);
    msgMIT.torqueForward = float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_TT support return expected current
    this->cmd_.elec = this->cmd_.torq / status_.Kn;

    serializeMITMsg(msgMIT, _txBuf);
}

void DMMotor::convertMitVdes(TxBus &_txBuf)
{
    MITMsg_s msgMIT = {};
    msgMIT.Kd = float2uint(this->MITKd_, 0, status_.MITKdMax, 12);
    msgMIT.Kp = 0;

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_VEL:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DMMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    this->cmd_.vel = std::clamp(regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel, -status_.VMax, status_.VMax);
    msgMIT.exptVel = float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);

    this->cmd_.torq = std::clamp(regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq, -status_.TMax, status_.TMax);
    msgMIT.torqueForward = float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_VDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    serializeMITMsg(msgMIT, _txBuf);
}

void DMMotor::convertMitVdesPdes(TxBus &_txBuf)
{
    MITMsg_s msgMIT = {};
    msgMIT.Kd = float2uint(this->MITKd_, 0, status_.MITKdMax, 12);
    msgMIT.Kp = float2uint(this->MITKp_, 0, status_.MITKpMax, 12);

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL:
    case MotorCmdType_e::SET_MIT: {
        break;
    }
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DMMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }
    this->cmd_.pos = std::clamp(regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos, -status_.PMax, status_.PMax);
    msgMIT.exptScale = float2uint(this->cmd_.pos, -status_.PMax, status_.PMax, 16);
    this->cmd_.vel = std::clamp(regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel, -status_.VMax, status_.VMax);
    msgMIT.exptVel = float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
    this->cmd_.torq = std::clamp(regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq, -status_.TMax, status_.TMax);
    msgMIT.torqueForward = float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_VDES_PDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    serializeMITMsg(msgMIT, _txBuf);
}

void DMMotor::convertPdesVdes(TxBus &_txBuf)
{
    PDESVDESMsg_s msgPDESVDES = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POSVEL: {
        break;
    }
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DMMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    this->cmd_.pos = regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos;
    msgPDESVDES.exptScale = this->cmd_.pos;
    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgPDESVDES.exptVel = this->cmd_.vel;

    _txBuf.len = 8;
    memcpy(_txBuf.data, &msgPDESVDES.exptScale, 4);
    memcpy(&_txBuf.data[4], &msgPDESVDES.exptVel, 4);

    // PDESVDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;
}

void DMMotor::convertVdes(TxBus &_txBuf)
{
    VDESMsg_s msgVDES = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_VEL:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DMMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgVDES.exptVel = this->cmd_.vel;

    _txBuf.len = 4;
    memcpy(_txBuf.data, &msgVDES.exptVel, 4);

    // VDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;
}

void DMMotor::convertEmit(TxBus &_txBuf)
{
    EMITMsg_s msgEMIT = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_MIT: {
        break;
    }
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DMMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }
    this->cmd_.pos = regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos;
    msgEMIT.exptScale = this->cmd_.pos;
    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgEMIT.exptVelX100 = static_cast<uint16_t>(((this->cmd_.vel < 0) ? -this->cmd_.vel : this->cmd_.vel) * 100.f);
    this->cmd_.torq = regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq;
    msgEMIT.imaxX10000 = static_cast<uint16_t>(((this->cmd_.torq < 0) ? -this->cmd_.torq : this->cmd_.torq) /
                                               status_.Kn / status_.currMax * status_.currTxCodeSpan);

    _txBuf.len = 8;

    memcpy(_txBuf.data, &msgEMIT, 8);

    // EMIT unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;
}

void DMMotor::convertDefault(TxBus &_txBuf)
{
    if (this->cmd_.curCmdType != MotorCmdType_e::OFF) {
        LOG::error("DMMotor", " %s: work mode error", regInfo_.name);
    }
    while (true)
        // it shouldn't be here
        UNUSED(_txBuf);
    ;
}
