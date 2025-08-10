#include "DMMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DMMOTOR;

DMMotor::ConvertFunc DMMotor::selectWorkMode(WorkMode_e _mode)
{
    DMMotor::ConvertFunc func = nullptr;
    switch (_mode) {
    case WorkMode_e::MIT_TT:
        func = &DMMotor::convertMitTt;
        break;
    case WorkMode_e::MIT_VDES:
        func = &DMMotor::convertMitVdes;
        break;
    case WorkMode_e::MIT_VDESPDES:
        func = &DMMotor::convertMitVdesPdes;
        break;
    case WorkMode_e::PDESVDES:
        func = &DMMotor::convertPdesVdes;
        break;
    case WorkMode_e::VDES:
        func = &DMMotor::convertVdes;
        break;
    case WorkMode_e::EMIT:
        func = &DMMotor::convertEmit;
        break;
    default:
        func = &DMMotor::convertDefault;
        break;
    }
    return func; // TODO: updateCtrlId
}

void DMMotor::serializeMITMsg(MITMsg_s &_msgMIT)
{
    _msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);
    this->cmd_.elec = this->cmd_.torq / status_.torqConstant;
    txBuf_.data[0] = static_cast<uint8_t>((_msgMIT.exptScale & 0xFF00) >> 8);
    txBuf_.data[1] = static_cast<uint8_t>(_msgMIT.exptScale & 0x00FF);
    txBuf_.data[2] = static_cast<uint8_t>((_msgMIT.exptVel & 0x0FF0) >> 4);
    txBuf_.data[3] = static_cast<uint8_t>((_msgMIT.exptVel & 0x000F) << 4 |
                                          ((_msgMIT.Kp & 0x0FF0) >> 8));
    txBuf_.data[4] = static_cast<uint8_t>(_msgMIT.Kp & 0x000F);
    txBuf_.data[5] = static_cast<uint8_t>((_msgMIT.Kd & 0x0FF0) >> 4);
    txBuf_.data[6] =
            static_cast<uint8_t>((_msgMIT.Kd & 0x000F) << 4 |
                                 ((_msgMIT.torqueForward & 0x0F00) >> 8));
    txBuf_.data[7] = static_cast<uint8_t>(_msgMIT.torqueForward & 0x00FF);
}

void DMMotor::convertMitTt()
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
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng, 2.f * PI),
                0);
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
                                                this->data_.singleCirAng,
                                                2.f * PI),
                                    0);
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  this->name_);
        break;
    }

    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_TT support return expected current, so return current value
    this->cmd_.elec = this->cmd_.torq / status_.torqConstant;

    serializeMITMsg(msgMIT);
    txBuf_.len = 8;
}

void DMMotor::convertMitVdes()
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
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng, 2.f * PI),
                0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  this->name_);
        break;
    }

    msgMIT.exptVel =
            float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_VDES unsupport return expected current, so return current value
    this->cmd_.elec = this->data_.torq / status_.torqConstant;

    serializeMITMsg(msgMIT);
    txBuf_.len = 8;
}

void DMMotor::convertMitVdesPdes()
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
                  this->name_);
        break;
    }
    msgMIT.exptScale =
            float2uint(this->cmd_.pos, -status_.PMax, status_.PMax, 16);
    msgMIT.exptVel =
            float2uint(this->cmd_.vel, -status_.VMax, status_.VMax, 12);
    msgMIT.torqueForward =
            float2uint(this->cmd_.torq, -status_.TMax, status_.TMax, 12);

    // MIT_VDES_PDES unsupport return expected current, so return current value
    this->cmd_.elec = this->data_.torq / status_.torqConstant;

    serializeMITMsg(msgMIT);
    txBuf_.len = 8;
}

void DMMotor::convertPdesVdes()
{
    PDESVDESMsg_s msgPDESVDES = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL: {
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  this->name_);
        break;
    }

    msgPDESVDES.exptScale = this->cmd_.pos;
    msgPDESVDES.exptVel = this->cmd_.vel;
    memcpy(txBuf_.data, &msgPDESVDES.exptScale, 4);
    memcpy(&txBuf_.data[4], &msgPDESVDES.exptVel, 4);

    // PDESVDES unsupport return expected current, so return current value
    this->cmd_.elec = this->data_.torq / status_.torqConstant;

    txBuf_.len = 8;
}

void DMMotor::convertVdes()
{
    VDESMsg_s msgVDES = {};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_VEL:
        break;
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL: {
        this->cmd_.vel = this->posPID_->calc(
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng, 2.f * PI),
                0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        break;
    }
    default:
        LOG::warn("DMMotor", " %s: the cmd in this mode is not supported",
                  this->name_);
        break;
    }

    msgVDES.exptVel = this->cmd_.vel;
    memcpy(txBuf_.data, &msgVDES.exptVel, 4);

    // VDES unsupport return expected current, so return current value
    this->cmd_.elec = this->data_.torq / status_.torqConstant;

    txBuf_.len = 4;
}

void DMMotor::convertEmit()
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
                  this->name_);
        break;
    }

    msgEMIT.exptScale = this->cmd_.pos;
    msgEMIT.exptVelX100 = static_cast<uint16_t>(
            ((this->cmd_.vel < 0) ? -this->cmd_.vel : this->cmd_.vel) * 100.f);
    msgEMIT.imaxX10000 = static_cast<uint16_t>(
            ((this->cmd_.torq < 0) ? -this->cmd_.torq : this->cmd_.torq) /
            status_.torqConstant / status_.currMax * status_.currTxCodeSpan);
    float f = msgEMIT.exptScale;
    memcpy(txBuf_.data, &f, 4);
    txBuf_.data[4] = static_cast<uint8_t>((msgEMIT.exptVelX100) >> 8);
    txBuf_.data[5] = static_cast<uint8_t>(msgEMIT.exptVelX100);
    txBuf_.data[6] = static_cast<uint8_t>((msgEMIT.imaxX10000) >> 8);
    txBuf_.data[7] = static_cast<uint8_t>(msgEMIT.imaxX10000);

    // EMIT unsupport return expected current, so return current value
    this->cmd_.elec = this->data_.torq / status_.torqConstant;

    txBuf_.len = 4;
}

void DMMotor::convertDefault()
{
    // it shouldn't be here
    if (this->cmd_.curCmdType != MotorCmdType_e::OFF) {
        LOG::error("DMMotor", " %s: work mode error", this->name_);
    }
    txBuf_.len = 0;
}
