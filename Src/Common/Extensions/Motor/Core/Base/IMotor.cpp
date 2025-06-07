#include "IMotor.hpp"

#include "MotorManager.hpp"

using namespace PINYMOTOR;

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

Data_s &IMotor::data() { return data_; }

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

void IMotor::overrideTxBaseId(uint16_t _newTxBaseId)
{
    model_.txBaseId = _newTxBaseId;
}

void IMotor::overrideRxBaseId(uint16_t _newRxBaseId)
{
    model_.rxBaseId = _newRxBaseId;
}
