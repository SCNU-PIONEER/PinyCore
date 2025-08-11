#include "DJIMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DJIMOTOR;

DJIMotor::ConvertFunc DJIMotor::selectWorkMode(WorkMode_e _mode)
{
    DJIMotor::ConvertFunc func = nullptr;
    switch (_mode) {
    case WorkMode_e::QUAD_CURR:
        func = &DJIMotor::convertQuadCurr;
        break;
    case WorkMode_e::QUAD_VOLT:
        func = &DJIMotor::convertQuadVolt;
        break;
    default:
        func = &DJIMotor::convertDefault;
        break;
    }
    return func; // TODO: updateCtrlId
}

int16_t DJIMotor::convertQuadCurr()
{
    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_ELEC:
        break;
    case MotorCmdType_e::SET_TORQ: {
        this->cmd_.elec = this->cmd_.torq / status_.torqConstant;
        break;
    }
    case MotorCmdType_e::SET_VEL: {
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        this->cmd_.elec = this->cmd_.torq / status_.torqConstant;
        break;
    }
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL: {
        this->cmd_.vel = this->posPID_->calc(
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng, 2.f * PI),
                0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        this->cmd_.elec = this->cmd_.torq / status_.torqConstant;
        break;
    }
    case MotorCmdType_e::SET_MIT: {
        this->cmd_.elec =
                this->posPID_->calc(getMinorArc(this->cmd_.pos,
                                                this->data_.singleCirAng,
                                                2.f * PI),
                                    0) +
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps) +
                this->cmd_.torq / status_.torqConstant;
        break;
    }
    default:
        LOG::warn("DJIMotor", " %s: the cmd in this mode is not supported",
                  this->name_);
        break;
    }
    int16_t ctrlCmd =
            static_cast<int16_t>(this->cmd_.elec / this->status_.currMax *
                                 this->status_.currTxCodeSpan);
    return ctrlCmd;
}

int16_t DJIMotor::convertQuadVolt()
{
    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_ELEC:
        break;
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
    case MotorCmdType_e::SET_POS:
    case MotorCmdType_e::SET_POSVEL: {
        this->cmd_.vel = this->posPID_->calc(
                getMinorArc(this->cmd_.pos, this->data_.singleCirAng, 2.f * PI),
                0);
        if (!(this->cmd_.velMax < 0.f)) {
            this->cmd_.vel = std::clamp(this->cmd_.vel, -this->cmd_.velMax,
                                        this->cmd_.velMax);
        }
        this->cmd_.torq =
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps);
        this->cmd_.elec =
                this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
        break;
    }
    case MotorCmdType_e::SET_MIT: {
        this->cmd_.torq =
                this->posPID_->calc(getMinorArc(this->cmd_.pos,
                                                this->data_.singleCirAng,
                                                2.f * PI),
                                    0) +
                this->velPID_->calc(this->cmd_.vel, this->data_.spdRadps) +
                this->cmd_.torq;
        this->cmd_.elec =
                this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
        break;
    }
    default:
        LOG::warn("DJIMotor", " %s: the cmd in this mode is not supported",
                  this->name_);
        break;
    }
    int16_t ctrlCmd =
            static_cast<int16_t>(this->cmd_.elec / this->status_.voltMax *
                                 this->status_.voltTxCodeSpan);
    return ctrlCmd;
}

int16_t DJIMotor::convertDefault()
{
    LOG::error("DJIMotor", " %s: this mode is not supported", this->name_);
    return 0;
}
