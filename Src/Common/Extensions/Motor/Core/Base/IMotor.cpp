#include "IMotor.hpp"

#include "MotorManager.hpp"

using namespace PINYMOTOR;

IMotor::IMotor() : id_(MotorManager::instance()->motorListSize())
{
    this->cmd_.clear();
    memset(&data_, 0, sizeof(Data_s));
}

MotorTypeDef_e IMotor::registerMotor()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().find(uid());
    if (it != motorManager->motors().end()) {
        return false;
    }
    motorManager->motors().insert({ uid(), this });
    return 0;
}

MotorTypeDef_e IMotor::cancelMotor()
{
    MotorManager *motorManager = MotorManager::instance();
    auto it = motorManager->motors().find(uid());
    if (it == motorManager->motors().end()) {
        return false;
    }
    motorManager->motors().erase(it);
    return 0;
}

MotorTypeDef_e IMotor::cmd(MotorCmdType_e _cmd, float _cmdData)
{
    if (_cmd != MotorCmdType_e::ON && _cmd != MotorCmdType_e::OFF) {
        this->curCmdType_ = _cmd;
    }
    switch (this->curCmdType_) {
    case MotorCmdType_e::ON: {
        this->cmd_.updateSW(true);
        break;
    }
    case MotorCmdType_e::OFF: {
        this->cmd_.updateSW(false);
        break;
    }
    case MotorCmdType_e::SET_ELEC: {
        this->cmd_.elec = _cmdData;
        break;
    }
    case MotorCmdType_e::SET_TORQ: {
        this->cmd_.torq = _cmdData;
        break;
    }
    case MotorCmdType_e::SET_VEL: {
        this->cmd_.vel = _cmdData;
        break;
    }
    case MotorCmdType_e::SET_POS: {
        this->cmd_.pos = _cmdData;
        break;
    }
    default: {
        LOG::error("IMotor", " %s: cmd %d is not supported", this->name_,
                   static_cast<int>(_cmd));
        return 1;
    }
    }
    return 0;
}

MotorTypeDef_e IMotor::cmd(MotorCmdType_e _cmd)
{
    if (_cmd == MotorCmdType_e::ON) {
        this->cmd_.updateSW(true);
    } else if (_cmd == MotorCmdType_e::OFF) {
        this->cmd_.updateSW(false);
    } else {
        LOG::error("IMotor", " %s: cmd %d is not supported", this->name_,
                   static_cast<int>(_cmd));
        return 1;
    }
    return 0;
}

uint8_t IMotor::id() const { return id_; }

Data_s &IMotor::data() { return data_; }

Cmd_s &IMotor::cmd() { return cmd_; }

float IMotor::txBaseId() const { return static_cast<float>(model_.txBaseId); }

float IMotor::rxBaseId() const { return static_cast<float>(model_.rxBaseId); }

float IMotor::RR() const { return model_.reductionRatio; }

float IMotor::measureMax() const
{
    return static_cast<float>(model_.measureMax);
}

float IMotor::measureMin() const
{
    return static_cast<float>(model_.measureMin);
}

float IMotor::span() const
{
    return static_cast<float>(model_.measureMax - model_.measureMin);
}

float IMotor::txFreq() const { return txFreq_; }

float IMotor::rxFreq() const { return rxFreq_; }

void IMotor::overrideReductionRatio(float _newReductionRatio)
{
    model_.reductionRatio = _newReductionRatio;
}

void IMotor::overrideMeasureMax(float _newMeasureMax)
{
    model_.measureMax = static_cast<uint16_t>(_newMeasureMax);
}

void IMotor::overrideMeasureMin(float _newMeasureMin)
{
    model_.measureMin = static_cast<uint16_t>(_newMeasureMin);
}

const char *IMotor::getName() const { return name_; }
