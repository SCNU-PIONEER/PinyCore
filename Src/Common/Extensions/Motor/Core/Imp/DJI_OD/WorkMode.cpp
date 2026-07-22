#include <algorithm>

#include "DJIOldMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

using namespace PINYMOTOR;
using namespace DJI_ODMOTOR;

void DJIOldMotor::serializeMsg(int16_t _ctrlCmd)
{
    this->group_->txBuf[(2 * this->getPosInGroup()) + 1] = static_cast<uint8_t>(_ctrlCmd & 0xFF);
    this->group_->txBuf[2 * this->getPosInGroup()] = static_cast<uint8_t>((_ctrlCmd >> 8) & 0xFF);
}

void DJIOldMotor::updateCtrlMode()
{
    switch (this->workMode_) {
    case WorkMode_e::TRIP_VOLT: {
        convert = &DJIOldMotor::convertTripVolt;
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    }
    default: {
        convert = &DJIOldMotor::convertDefault;
        LOG::error("DJIOldMotor", " %s: this mode is not supported", regInfo_.name);
        this->ctrlId_ = 0xFFFF;
        break;
    }
    }
}

void DJIOldMotor::convertTripVolt()
{
    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_ELEC: {
        break;
    }
    case MotorCmdType_e::SET_TORQ: {
        this->cmd_.elec = torq2volt(this->cmd_.torq);
        break;
    }
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("DJIOldMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        this->cmd_.elec = 0;
        break;
    }

    this->cmd_.elec = std::clamp(regInfo_.isReverse ? -this->cmd_.elec : this->cmd_.elec, -this->status_.voltMax,
                                 this->status_.voltMax);
    serializeMsg(static_cast<int16_t>(this->cmd_.elec / this->status_.voltMax * this->status_.voltTxCodeSpan));
}

void DJIOldMotor::convertDefault()
{
    if (this->cmd_.curCmdType != MotorCmdType_e::OFF) {
        LOG::error("DJIOldMotor", " %s: work mode error", regInfo_.name);
    }
    while (true)
        // it shouldn't be here
        ;
}
