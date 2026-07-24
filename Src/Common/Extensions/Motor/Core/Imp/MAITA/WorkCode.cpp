#include <algorithm>

#include "MTMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"
#include <cmath>
#include <cstdint>

using namespace PINYMOTOR;
using namespace MTMOTOR;

void MTMotor::updateCtrlMode()
{
    ctrlId_ = regInfo_.model.txBaseId + regInfo_.offsetId;
    switch (this->workMode_) {
    case WorkMode_e::TORQ: {
        convert = &MTMotor::torqCtrl;
        break;
    }
    case WorkMode_e::SPEED: {
        convert = &MTMotor::speedCtrl;
        break;
    }
    case WorkMode_e::ABS_POS: {
        convert = &MTMotor::absPosCtrl;
        break;
    }
    case WorkMode_e::SINGLE_POS: {
        convert = &MTMotor::singlePosCtrl;
        break;
    }
    case WorkMode_e::INC_POS: {
        convert = &MTMotor::incPosCtrl;
        break;
    }
    case WorkMode_e::FORCE_POS: {
        convert = &MTMotor::forcePosCtrl;
        break;
    }
    default: {
        convert = &MTMotor::disable;
        LOG::error("MTMotor", " %s: this mode is not supported", regInfo_.name);
        break;
    }
    }
}

void MTMotor::switchCtrlMode(WorkMode_e _workMode)
{
    this->workMode_ = _workMode;
    updateCtrlMode();
}

//力矩控制
void MTMotor::torqCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitTorqCtrlMsg_s data{};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_TORQ:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("MTMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    int32_t rawIq32 = static_cast<int32_t>(std::lround(this->cmd_.torq / status_.kn * 100.0f));
    int32_t tmpIq32 = regInfo_.isReverse ? -rawIq32 : rawIq32;
    int16_t rawIq = static_cast<int16_t>(std::max<int32_t>(
            std::numeric_limits<int16_t>::min(), std::min<int32_t>(std::numeric_limits<int16_t>::max(), tmpIq32)));
    data.iqControl = rawIq;
    memcpy(_txBuf.data(), &data, 8);
}

//速度控制
void MTMotor::speedCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitSpeedCtrlMsg_s data{};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_VEL:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("MTMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    data.torqueMax = static_cast<uint8_t>(this->status_.currMax); //直接由堵转电流控制
    int32_t rawSpeed = static_cast<int32_t>(rad2deg(this->cmd_.vel) * 100);
    data.speedControl = regInfo_.isReverse ? -rawSpeed : rawSpeed;
    memcpy(_txBuf.data(), &data, 8);
}

//此模式转到以电机零点为基准的多圈角度,例如多圈当前为100，现在设置2，是从100到2
void MTMotor::absPosCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitAbsPosCtrlMsg_s data{};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POSVEL:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("MTMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    uint16_t rawSpeed = static_cast<uint16_t>(rad2deg(fabsf(this->cmd_.vel)));
    data.speedMax = std::min(rawSpeed, static_cast<uint16_t>(rad2deg(this->status_.speedMax)));
    float rawPos = regInfo_.isReverse ? -(rad2deg(this->cmd_.pos) * 100) : (rad2deg(this->cmd_.pos) * 100);
    float cirNum = this->data_.cirNum;
    data.pos = static_cast<int32_t>(rawPos + (2 * PI * truncf(cirNum) * 100));
    memcpy(_txBuf.data(), &data, 8);
}
//在多圈保存功能关闭时，默认为单圈模式。该指令可在单圈模式下使用
void MTMotor::singlePosCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitSinglePosCtrlMsg_s data{};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POSVEL:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("MTMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    if (cmd_.pos > 0) {
        data.spinDir = regInfo_.isReverse ? 0x00 : 0x01;
    } else {
        data.spinDir = regInfo_.isReverse ? 0x01 : 0x00;
    }
    uint16_t rawSpeed = static_cast<uint16_t>(rad2deg(fabsf(this->cmd_.vel)));
    data.speedMax = std::min(rawSpeed, static_cast<uint16_t>(rad2deg(this->status_.speedMax)));

    this->cmd_.pos = PINYMOTOR::rangeMap(this->cmd_.pos);
    data.angleCtrl = static_cast<uint16_t>(rad2deg(fabsf(this->cmd_.pos)) * 100);
    memcpy(_txBuf.data(), &data, 8);
}

//此模式转动到当前起点的多圈角度,例如多圈当前为100，现在设置2，是从100到102
void MTMotor::incPosCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitIncrementalPosCtrlMsg_s data{};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_POS:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("MTMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    data.speedMax = static_cast<uint16_t>(rad2deg(fabsf(this->status_.speedMax)));
    data.addAngle = regInfo_.isReverse ? -static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100) :
                                         static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100);
    memcpy(_txBuf.data(), &data, 8);
}

//与absPosCtrl类似,多了力矩限制
void MTMotor::forcePosCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmitForcePosCtrlMsg_s data{};

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_MIT:
        break;
    default:
        if (this->cmd_.curCmdType != MotorCmdType_e::OFF && this->cmd_.curCmdType != MotorCmdType_e::ON)
            LOG::warn("MTMotor", " %s: the cmd in this mode is not supported", regInfo_.name);
        break;
    }

    uint8_t percent = static_cast<uint8_t>(this->cmd_.torq / this->status_.torqMax * 100);
    data.torqueMax = percent * static_cast<uint8_t>(this->status_.currMax); //这个模式赋0值,电流上限为0
    data.speedMax = static_cast<uint16_t>(rad2deg(fabsf(this->cmd_.vel)));
    data.angCtrl = regInfo_.isReverse ? -static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100) :
                                        static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100);
    memcpy(_txBuf.data(), &data, 8);
}
