#include "DJIOldMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DJI_ODMOTOR;

DJIOldMotor::ConvertFunc DJIOldMotor::selectWorkMode(WorkMode_e _mode)
{
    DJIOldMotor::ConvertFunc func = nullptr;
    switch (_mode) {
    case WorkMode_e::QUAD_VOLT:
        func = &DJIOldMotor::convertTripVolt;
        break;
    default:
        func = &DJIOldMotor::convertDefault;
        break;
    }
    return func; // TODO: updateCtrlId
}

int16_t DJIOldMotor::convertTripVolt()
{
    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::OFF:
    case MotorCmdType_e::ON:
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

int16_t DJIOldMotor::convertDefault()
{
    LOG::error("DJIOldMotor", " %s: this mode is not supported", this->name_);
    return 0;
}
