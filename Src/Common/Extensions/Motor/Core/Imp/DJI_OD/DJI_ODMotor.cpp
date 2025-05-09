
#include "DJI_ODMotor.hpp"

#include "Bsp_can.hpp"

#include "MotorCommonMacros.hpp"

using namespace PINYMOTOR;

template <typename Derived> void DJI_ODMotor<Derived>::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](const uint8_t *_rxBuf) {
                // basic cb
                this->_parse_(_rxBuf);
                // user cb
                if (this->userRecvCallback_ != nullptr) {
                    this->userRecvCallback_(_rxBuf);
                }
            });
    this->log("INFO", "green", "Motor %s: Receive cb registed", this->name_);
}

template <typename Derived>
MotorTypeDef_e DJI_ODMotor<Derived>::_cmd_(MotorCmdType_e _cmd, float _cmdData)
{
    switch (_cmd) {
    case MotorCmdType_e::SET_VOLT:
        if(this->workMode_ == WorkMode_e::TRIP_VOLT)
           cmd_.volt = _cmdData;
        else
           this->log("ERROR", "red", "Motor %s: Invalid cmd type", this->name_);
    break;
    default:
        this->log("ERROR", "red", "Motor %s: Invalid cmd type", this->name_);
        return 1;
    };
    return 0;
}

template <typename Derived> MotorTypeDef_e DJI_ODMotor<Derived>::_cmd_(MotorCmdType_e _cmd)
{
    if (_cmd == MotorCmdType_e::ON) {
        cmd_.updateSW(true);
    } else if (_cmd == MotorCmdType_e::OFF) {
        cmd_.updateSW(false);
    } else {
        this->log("ERROR", "red", "Motor %s: not SW cmd!", this->name_);
        return 1;
    }
    return 0;
}

template <typename Derived>
MotorTypeDef_e DJI_ODMotor<Derived>::_send_(uint8_t *_txBuf, uint8_t _len)
{
    return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->ctrlId_,
            _txBuf, _len));
}

template <typename Derived> MotorTypeDef_e DJI_ODMotor<Derived>::_parse_(const uint8_t *_rxBuf)
{
    DJI_ODMotorFeedback_s fb;
    fb.rawScale = ((_rxBuf[0] << 8) | _rxBuf[1]);
    fb.rawTorq = static_cast<int16_t>(((_rxBuf[2] << 8) | _rxBuf[3]));

    this->data_.rawScale = fb.rawScale;

    this->data_.torq = fb.rawTorq / this->stats_.torqRxCodeSpan *
                    this->stats_.torqMax;
    
    this->data_.tempture = 0.f; // TODO:

    this->data_.curr = this->data_.torq / stats_.torqConstant;

    float angDiff = getMinorArc(this->data_.rawScale, this->data_.lastRawScale,
                                this->span()) *
                    2 * PI / (this->span() * this->RR());

    if (this->globalState_ == GlobalState_e::OFFLINE &&
        this->data_.lastRawScale != this->data_.rawScale) {
        this->globalState_ = GlobalState_e::ONLINE;
        angDiff = 0;
    }

    if(this->rxFreq_)
        this->data_.spdRadps = angDiff / this->rxFreq_;

    this->data_.spdRpm = radps2rpm(this->data_.spdRadps);    

    this->data_.lastRawScale = this->data_.rawScale;

    this->data_.multipCirAng += angDiff;
    this->data_.singleCirAng += angDiff;
    this->data_.singleCirAng = rangeMap(this->data_.singleCirAng, 0, 2 * PI);
    return 0;
}

template <typename Derived> MotorTypeDef_e DJI_ODMotor<Derived>::_ctrl_()
{
    MotorTypeDef_e rslt = 0;
    uint8_t *txBuf = nullptr;
    uint16_t currCmd =0;
    // 寻找自己所属的电机组
    TripMotorGroup_s *group = this->findGroup();
    if (group != nullptr) {
        txBuf = group->package;
    } else {
        this->log("ERROR", "red", "Motor %s: Can't find group %d", this->name_,
                  this->getGroupId());
        return 1;
    }
    switch (this->workMode_) {
    case WorkMode_e::TRIP_VOLT: {
        this->ctrlId_ = this->getGroupId() + 0u; // 0x1FF
        currCmd = cmd_.volt / this->stats_.voltMax * this->stats_.voltTxCodeSpan;
        break;
    }
    default: {
        currCmd =0;
        this->log("ERROR", "red", "Motor %s: this mode is not supported",
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
        txBuf[6] = txBuf[7] = 0;
    }

    if (this->checkGroupSend(group)) {
        group->lastSendTick = xTaskGetTickCount();
        rslt |= this->send(txBuf, 8);
    }
    return rslt;
}

/**********************************************************************************/
#include "GM3510.hpp"
template class PINYMOTOR::DJI_ODMotor<GM3510>;
