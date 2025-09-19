#include "DMMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DMMOTOR;

DMMotor::TxBus DMMotor::serializeMITMsg(MITMsg_s &_msgMIT)
{
    TxBus txBuf{ .len = 8 };
    txBuf.data[0] = static_cast<uint8_t>((_msgMIT.exptScale & 0xFF00) >> 8);
    txBuf.data[1] = static_cast<uint8_t>(_msgMIT.exptScale & 0x00FF);
    txBuf.data[2] = static_cast<uint8_t>((_msgMIT.exptVel & 0x0FF0) >> 4);
    txBuf.data[3] = static_cast<uint8_t>((_msgMIT.exptVel & 0x000F) << 4 |
                                         ((_msgMIT.Kp & 0x0FF0) >> 8));
    txBuf.data[4] = static_cast<uint8_t>(_msgMIT.Kp & 0x000F);
    txBuf.data[5] = static_cast<uint8_t>((_msgMIT.Kd & 0x0FF0) >> 4);
    txBuf.data[6] =
            static_cast<uint8_t>((_msgMIT.Kd & 0x000F) << 4 |
                                 ((_msgMIT.torqueForward & 0x0F00) >> 8));
    txBuf.data[7] = static_cast<uint8_t>(_msgMIT.torqueForward & 0x00FF);
    return txBuf;
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

DMMotor::TxBus DMMotor::convertMitTt()
{
    MITMsg_s msgMIT = {};
    msgMIT.Kp = msgMIT.Kd = 0;

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_TORQ:
        break;
    case MotorCmdType_e::SET_VEL: {
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        break;
    }
    case MotorCmdType_e::SET_POS: {
        this->cmd_.vel = this->posPID_->calc(
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng), 0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        break;
    }
    case MotorCmdType_e::SET_POSVEL: {
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps) +
                this->posPID_->calc(getMinorArc(this->cmd_.pos,
                                                this->data_.singleCirAng),
                                    0);
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  regInfo_.name);
        break;
    }

    this->cmd_.torq = regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq;
    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_TT support return expected current
    this->cmd_.elec = this->cmd_.torq / status_.Kn;

    return serializeMITMsg(msgMIT);
}

DMMotor::TxBus DMMotor::convertMitVdes()
{
    MITMsg_s msgMIT = {};
    msgMIT.Kd =
            float2uint(this->MITKd_, -status_.MITKdMax, status_.MITKdMax, 12);
    msgMIT.Kp = 0;
    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_VEL:
        break;
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL:
    case MotorCmdType_e::SET_MIT: {
        this->cmd_.vel = this->posPID_->calc(
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng), 0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  regInfo_.name);
        break;
    }

    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgMIT.exptVel =
            float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);

    this->cmd_.torq = regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq;
    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_VDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    return serializeMITMsg(msgMIT);
}

DMMotor::TxBus DMMotor::convertMitVdesPdes()
{
    MITMsg_s msgMIT = {};
    msgMIT.Kd =
            float2uint(this->MITKd_, -status_.MITKdMax, status_.MITKdMax, 12);
    msgMIT.Kp =
            float2uint(this->MITKp_, -status_.MITKpMax, status_.MITKpMax, 12);

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL:
    case MotorCmdType_e::SET_MIT: {
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  regInfo_.name);
        break;
    }
    this->cmd_.pos = regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos;
    msgMIT.exptScale =
            float2uint(this->cmd_.pos, -status_.PMax, status_.PMax, 16);
    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgMIT.exptVel =
            float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
    this->cmd_.torq = regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq;
    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_VDES_PDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    return serializeMITMsg(msgMIT);
}

DMMotor::TxBus DMMotor::convertPdesVdes()
{
    PDESVDESMsg_s msgPDESVDES = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL: {
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  regInfo_.name);
        break;
    }

    this->cmd_.pos = regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos;
    msgPDESVDES.exptScale = this->cmd_.pos;
    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgPDESVDES.exptVel = this->cmd_.vel;

    TxBus txBuf{ .len = 8 };
    memcpy(txBuf.data, &msgPDESVDES.exptScale, 4);
    memcpy(&txBuf.data[4], &msgPDESVDES.exptVel, 4);

    // PDESVDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    return txBuf;
}

DMMotor::TxBus DMMotor::convertVdes()
{
    VDESMsg_s msgVDES = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_VEL:
        break;
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL: {
        this->cmd_.vel = this->posPID_->calc(
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng), 0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  regInfo_.name);
        break;
    }

    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgVDES.exptVel = this->cmd_.vel;

    TxBus txBuf{ .len = 4 };
    memcpy(txBuf.data, &msgVDES.exptVel, 4);

    // VDES unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    return txBuf;
}

DMMotor::TxBus DMMotor::convertEmit()
{
    EMITMsg_s msgEMIT = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL:
    case MotorCmdType_e::SET_MIT: {
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  regInfo_.name);
        break;
    }
    this->cmd_.pos = regInfo_.isReverse ? -this->cmd_.pos : this->cmd_.pos;
    msgEMIT.exptScale = this->cmd_.pos;
    this->cmd_.vel = regInfo_.isReverse ? -this->cmd_.vel : this->cmd_.vel;
    msgEMIT.exptVelX100 = static_cast<uint16_t>(
            ((this->cmd_.vel < 0) ? -this->cmd_.vel : this->cmd_.vel) * 100.f);
    this->cmd_.torq = regInfo_.isReverse ? -this->cmd_.torq : this->cmd_.torq;
    msgEMIT.imaxX10000 = static_cast<uint16_t>(
            ((this->cmd_.torq < 0) ? -this->cmd_.torq : this->cmd_.torq) /
            status_.Kn / status_.currMax * status_.currTxCodeSpan);

    TxBus txBuf{ .len = 8 };
    memcpy(txBuf.data, &msgEMIT.exptScale, 4);
    txBuf.data[4] = static_cast<uint8_t>((msgEMIT.exptVelX100) >> 8);
    txBuf.data[5] = static_cast<uint8_t>(msgEMIT.exptVelX100);
    txBuf.data[6] = static_cast<uint8_t>((msgEMIT.imaxX10000) >> 8);
    txBuf.data[7] = static_cast<uint8_t>(msgEMIT.imaxX10000);

    // EMIT unsupport return expected current
    this->cmd_.elec = this->data_.torq / status_.Kn;

    return txBuf;
}

DMMotor::TxBus DMMotor::convertDefault()
{
    if (this->cmd_.curCmdType != MotorCmdType_e::OFF) {
        LOG::error("DMMotor", " %s: work mode error", regInfo_.name);
    }
    while (true)
        // it shouldn't be here
        ;

    return {};
}
