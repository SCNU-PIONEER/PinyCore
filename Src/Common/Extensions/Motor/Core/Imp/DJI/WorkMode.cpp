#include "DJIMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DJIMOTOR;

void DJIMotor::serializeMsg(int16_t _ctrlCmd)
{
    this->group_->txBuf[(2 * this->getPosInGroup()) + 1] =
            static_cast<uint8_t>(_ctrlCmd & 0xFF);
    this->group_->txBuf[2 * this->getPosInGroup()] =
            static_cast<uint8_t>((_ctrlCmd >> 8) & 0xFF);
}

void DJIMotor::updateCtrlMode()
{
    switch (regInfo_.workMode) {
    case WorkMode_e::QUAD_CURR:
        convert = &DJIMotor::convertQuadCurr;
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    case WorkMode_e::QUAD_VOLT: {
        convert = &DJIMotor::convertQuadVolt;
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    }
    default: {
        convert = &DJIMotor::convertDefault;
        LOG::error("DJIMotor", " %s: this mode is not supported",
                   regInfo_.name);
        this->ctrlId_ = 0xFFFF;
        break;
    }
    }
}

void DJIMotor::convertQuadCurr()
{
    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_ELEC: {
        break;
    }
    case MotorCmdType_e::SET_TORQ: {
        this->cmd_.elec = this->cmd_.torq / status_.Kn;
        break;
    }
    case MotorCmdType_e::SET_VEL: {
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        this->cmd_.elec = this->cmd_.torq / status_.Kn;
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
        this->cmd_.elec = this->cmd_.torq / status_.Kn;
        break;
    }
    case MotorCmdType_e::SET_POSVEL:
    case MotorCmdType_e::SET_MIT: {
        this->cmd_.elec =
                this->posPID_->calc(getMinorArc(this->cmd_.pos,
                                                this->data_.singleCirAng,
                                                2.f * PI),
                                    0) +
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps) +
                this->cmd_.torq / status_.Kn;
        break;
    }
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF &&
            this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DJIMotor", " %s: the cmd in this mode is not supported",
                      regInfo_.name);
        break;
    }
    this->cmd_.elec = regInfo_.isReverse ? -this->cmd_.elec : this->cmd_.elec;
    serializeMsg(static_cast<int16_t>(this->cmd_.elec / this->status_.currMax *
                                      this->status_.currTxCodeSpan));
}

void DJIMotor::convertQuadVolt()
{
    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_ELEC: {
        break;
    }
    case MotorCmdType_e::SET_TORQ: {
        this->cmd_.elec =
                this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
        break;
    }
    case MotorCmdType_e::SET_VEL: {
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        this->cmd_.elec =
                this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
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
        this->cmd_.elec = this->cmd_.torq;
        break;
    }
    case MotorCmdType_e::SET_POSVEL:
    case MotorCmdType_e::SET_MIT: {
        this->cmd_.torq =
                this->posPID_->calc(getMinorArc(this->cmd_.pos,
                                                this->data_.singleCirAng),
                                    0) +
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps) +
                this->cmd_.torq;
        this->cmd_.elec =
                this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
        break;
    }
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF &&
            this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DJIMotor", " %s: the cmd in this mode is not supported",
                      regInfo_.name);
        break;
    }

    this->cmd_.elec = regInfo_.isReverse ? -this->cmd_.elec : this->cmd_.elec;
    serializeMsg(static_cast<int16_t>(this->cmd_.elec / this->status_.voltMax *
                                      this->status_.voltTxCodeSpan));
}

void DJIMotor::convertDefault()
{
    if (this->cmd_.curCmdType != MotorCmdType_e::OFF) {
        LOG::error("DJIMotor", " %s: work mode error", regInfo_.name);
    }
    while (true)
        // it shouldn't be here
        ;
}
