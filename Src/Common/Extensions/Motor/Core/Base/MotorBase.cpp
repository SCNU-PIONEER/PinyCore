#include "./MotorBase.hpp"

using namespace PINYMOTOR;
MotorBase::MotorBase(const char _name[16], InitConfig_s _config)
{
    this->pComHandle_ = _config.pComHandle;
    this->comType_ = _config.comType;
    this->workMode_ = _config.workMode;
    this->globalState_ = GlobalState_e::UNREGISTER;
    this->offsetId_ = _config.offsetId;
    this->txFreq_ = _config.txFreq;

    this->posPID_ = std::move(_config.posPID);
    this->velPID_ = std::move(_config.velPID);
    this->torqPID_ = std::move(_config.torqPID);

    strcpy(this->name_, _name);
}

void MotorBase::regUserRecvCallback(
        std::function<void(const uint8_t *_rxBuf)> _callback)
{
    userRecvCallback_ = std::move(_callback);
}

bool MotorBase::checkSend() const
{
    return (xTaskGetTickCount() - lastSendTick) >=
           pdMS_TO_TICKS(1000.f / this->txFreq_);
}

void MotorBase::calcRecvFreq()
{
    uint32_t dt = xTaskGetTickCount() - lastRecvTick;
    lastRecvTick = xTaskGetTickCount();
    if (dt == 0) {
        return;
    } else {
        this->rxFreq_ = 1000.f / static_cast<float>(dt);
    }
}
