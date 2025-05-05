#include "DJIMotor.hpp"

#include "Bsp_can.hpp"

#include "MotorCommonMacros.hpp"


using namespace PINYMOTOR;

template <typename T> void DJIMotor<T>::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            static_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](uint8_t *_rxBuf) {
                // basic cb
                this->_parse_(_rxBuf);
                // user cb
                if (this->userRecvCallback_ != nullptr) {
                    this->userRecvCallback_(_rxBuf);
                }
            });
    this->log("INFO", "", "Motor %s: Receive cb registed", this->name_);
}

template <typename T>
MotorTypeDef_e DJIMotor<T>::_cmd_(MotorCmdType_e _cmd, float _cmdData)
{
    switch (_cmd) {
    case MotorCmdType_e::SET_TORQ:
        if(this->workMode_ == WorkMode_e::QUAD_CURR)
           cmd_.torq = _cmdData;
        else
           this->log("ERROR", "", "Motor %s: Invalid cmd type", this->name_);
    break;
    case MotorCmdType_e::SET_VOLT:
        if(this->workMode_ == WorkMode_e::QUAD_VOLT)
           cmd_.volt = _cmdData;
        else
           this->log("ERROR", "", "Motor %s: Invalid cmd type", this->name_);
    break;
    default:
        this->log("ERROR", "", "Motor %s: Invalid cmd type", this->name_);
        return 1;
    };
    return 0;
}

template <typename T>
MotorTypeDef_e DJIMotor<T>::_send_(uint8_t *_txBuf, uint8_t _len)
{
    return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
            static_cast<canHandle *>(this->pComHandle_), this->getGroupId(),
            _txBuf, _len));
}

template <typename T> MotorTypeDef_e DJIMotor<T>::_parse_(uint8_t *_rxBuf)
{
    DJIMotorFeedback_s fb;
    fb.rawScale = ((_rxBuf[0] << 8) | _rxBuf[1]);
    fb.rawRpm = static_cast<int16_t>(((_rxBuf[2] << 8) | _rxBuf[3]));
    fb.current = ((_rxBuf[4] << 8) | _rxBuf[5]);
    fb.temperature = _rxBuf[6];

    this->data_.rawScale = fb.rawScale;
    this->data_.rawRpm = fb.rawRpm;
    this->data_.curr = fb.current;
    this->data_.tempture = fb.temperature;

    this->data_.torq = fb.current * stats_.torqConstant;
    this->data_.spdRpm = fb.rawRpm / this->RR();
    this->data_.spdRadps = rpm2radps(this->data_.spdRpm);

    float angDiff = getMinorArc(this->data_.rawScale, this->data_.lastRawScale,
                                this->span()) *
                    2 * PI / (this->span() * this->RR());
    if (this->globalState_ == GlobalState_e::OFFLINE &&
        this->data_.lastRawScale != this->data_.rawScale) {
        this->globalState_ = GlobalState_e::ONLINE;
        angDiff = 0;
    }
    this->data_.lastRawScale = this->data_.rawScale;

    this->data_.multipCirAng += angDiff;
    this->data_.singleCirAng += angDiff;
    this->data_.singleCirAng = rangeMap(this->data_.singleCirAng, 0, 2 * PI);
    return 0;
}

template <typename T> MotorTypeDef_e DJIMotor<T>::_ctrl_()
{
    MotorTypeDef_e rslt = 0;
    uint8_t *txBuf = nullptr;
    uint16_t currCmd =0;
    // 寻找自己所属的电机组
    QuadMotorGroup_s *group = nullptr;
    for (auto &entry : this->motorMap_) {
        if (entry.first == this->pComHandle_) {
            auto it = entry.second.find(this->getGroupId());
            if (it != entry.second.end()) {
                group = it;
                break;
            }
        }
    }
    if (group != nullptr) {
        txBuf = group->package;
    } else {
        this->log("ERROR", "", "Motor %s: Can't find group %d", this->name_,
                  this->getGroupId());
        return 1;
    }
    switch (this->workMode_) {
    case WorkMode_e::QUAD_CURR: {
        currCmd = cmd_.torq / stats_.torqConstant /
                  this->stats_.currMax * this->stats_.currTxCodeSpan;
        break;
    }
    case WorkMode_e::QUAD_VOLT: {
        currCmd = cmd_.volt / this->stats_.voltMax * this->stats_.voltTxCodeSpan;
        break;
    }
    default: {
        currCmd =0;
        this->log("ERROR", "", "Motor %s: this mode is not supported",
                  this->name_);
        break;
    }
    }
    if (txBuf != nullptr) {
        if (cmd_.SW) {
            txBuf[2 * this->getPosInGroup() + 1] =
                    static_cast<uint8_t>(currCmd & 0xFF);
            txBuf[2 * this->getPosInGroup()] =
                    static_cast<uint8_t>((currCmd >> 8) & 0xFF);
        } else {
            txBuf[2 * this->getPosInGroup() + 1] = 0;
            txBuf[2 * this->getPosInGroup()] = 0;
        }
    }

    if (this->checkGroupSend(group)) {
        group->lastSendTick = xTaskGetTickCount();
        rslt |= this->send(txBuf, 8);
    }
    return rslt;
}
