
#include "DJI_ODMotor.hpp"

#include "../../../Utils/MotorCommonMacros.hpp"

#include "Bsp_can.hpp"

using namespace PINYMOTOR;

DJI_ODMotorStats_s &
DJI_ODMotorStats_s::operator=(const DJI_ODMotorStats_s &_other)
{
    if (this != &_other) {
        voltTxCodeSpan = _other.voltTxCodeSpan;
        torqRxCodeSpan = _other.torqRxCodeSpan;
        currRated = _other.currRated;
        torqRated = _other.torqRated;
        voltMax = _other.voltMax;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        torqConstant = _other.torqConstant;
    }
    return *this;
}

DJI_ODMotor::DJI_ODMotor(const char _name[16], InitConfig_s _config)
        : Base(_name, _config)
{
    this->cmd_.clear();
}

DJI_ODMotor::~DJI_ODMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    this->log("INFO", "green", "Motor %s: An instance of DJI_ODMotor destroyed",
              this->name_);
}

void DJI_ODMotor::overrideStats(const DJI_ODMotorStats_s &_stats)
{
    stats_ = _stats;
}

uint16_t DJI_ODMotor::canId() const { return this->model_.txBaseId + 0u; }

uint16_t DJI_ODMotor::masterId() const
{
    return this->model_.rxBaseId + this->offsetId_;
}

uint16_t DJI_ODMotor::uid() { return masterId(); }

void DJI_ODMotor::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](const uint8_t *_rxBuf) {
                // basic cb
                this->parse(_rxBuf);
                // user cb
                if (this->userRecvCallback_ != nullptr) {
                    this->userRecvCallback_(_rxBuf);
                }
            });
    this->log("INFO", "green", "Motor %s: Receive cb registed", this->name_);
}

void DJI_ODMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId());
    this->log("INFO", "green", "Motor %s: Receive cb canceled", this->name_);
}

void DJI_ODMotor::updateCtrlId()
{
    switch (this->workMode_) {
    case WorkMode_e::TRIP_VOLT: {
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    }
    default: {
        this->log("ERROR", "red", "Motor %s: this mode is not supported",
                  this->name_);
        break;
    }
    }
}

MotorTypeDef_e DJI_ODMotor::send(uint16_t _sendId, uint8_t *_txBuf,
                                 uint8_t _len)
{
    // send data to CAN
    TripMotorGroup_s *group = this->findGroup();
    if (group == nullptr) {
        this->log("ERROR", "red", "Motor %s: Can't find group %hx", this->name_,
                  this->getGroupId());
        return 1;
    } else {
        if (this->checkGroupSend(group)) {
#if 1
            // Check this Buffer
            this->log("DEBUG", "blue", "Motor %s: send data to CAN %hx",
                      this->name_, _sendId);
            this->log(
                    "DEBUG", "blue",
                    "Motor %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
                    this->name_, _txBuf[0], _txBuf[1], _txBuf[2], _txBuf[3],
                    _txBuf[4], _txBuf[5], _txBuf[6], _txBuf[7]);
#endif
            group->lastSendTick = xTaskGetTickCount();
            return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                    reinterpret_cast<canHandle *>(this->pComHandle_), _sendId,
                    _txBuf, _len));
        } else {
            return 0;
        }
    }
}

MotorTypeDef_e DJI_ODMotor::parse(const uint8_t *_rxBuf)
{
    DJI_ODMotorFeedback_s fb;
    fb.rawScale = ((_rxBuf[0] << 8) | _rxBuf[1]);
    fb.rawTorq = static_cast<int16_t>(((_rxBuf[2] << 8) | _rxBuf[3]));

    this->data_.rawScale = fb.rawScale;

    this->data_.torq =
            fb.rawTorq / this->stats_.torqRxCodeSpan * this->stats_.torqMax;

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

    if (this->rxFreq_)
        this->data_.spdRadps = angDiff / this->rxFreq_;

    this->data_.spdRpm = radps2rpm(this->data_.spdRadps);

    this->data_.lastRawScale = this->data_.rawScale;

    this->data_.multipCirAng += angDiff;
    this->data_.singleCirAng += angDiff;
    this->data_.singleCirAng = rangeMap(this->data_.singleCirAng, 0, 2 * PI);
    return 0;
}

MotorTypeDef_e DJI_ODMotor::ctrl()
{
    MotorTypeDef_e rslt = 0;
    uint8_t txBuf[8] = {};
    int16_t ctrlCmd = 0;
    switch (this->workMode_) {
    case WorkMode_e::TRIP_VOLT: {
        if (this->curCmdType_ == MotorCmdType_e::SET_TORQ) {
            if (this->torqPID_ != nullptr) {
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                this->log("ERROR", "red", "Motor %s: torqPID is null",
                          this->name_);
            }
        } else if (this->curCmdType_ == MotorCmdType_e::SET_VEL) {
            if (this->velPID_ != nullptr || this->torqPID_ != nullptr) {
                this->cmd_.torq = this->velPID_->calc(this->cmd_.vel,
                                                      this->data_.spdRadps);
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                this->log("ERROR", "red", "Motor %s: velPID or torqPID is null",
                          this->name_);
            }
        } else if (this->curCmdType_ == MotorCmdType_e::SET_POS) {
            if (this->posPID_ != nullptr || this->velPID_ != nullptr ||
                this->torqPID_ != nullptr) {
                this->cmd_.vel = this->posPID_->calc(
                        getMinorArc(this->cmd_.pos, this->data_.singleCirAng,
                                    2.f * PI),
                        0);
                this->cmd_.torq = this->velPID_->calc(this->cmd_.vel,
                                                      this->data_.spdRadps);
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                this->log("ERROR", "red",
                          "Motor %s: posPID or velPID or torqPID is null",
                          this->name_);
            }
        }
        ctrlCmd = static_cast<int16_t>(this->cmd_.elec / this->stats_.voltMax *
                                       this->stats_.voltTxCodeSpan);
        break;
    }
    default: {
        ctrlCmd = 0;
        this->log("ERROR", "red", "Motor %s: this mode is not supported",
                  this->name_);
        break;
    }
    }
    if (this->cmd_.SW) {
        txBuf[2 * this->getPosInGroup() + 1] =
                static_cast<uint8_t>(ctrlCmd & 0xFF);
        txBuf[2 * this->getPosInGroup()] =
                static_cast<uint8_t>((ctrlCmd >> 8) & 0xFF);
    } else {
        this->posPID_->reset();
        this->velPID_->reset();
        this->torqPID_->reset();
        txBuf[2 * this->getPosInGroup() + 1] = 0;
        txBuf[2 * this->getPosInGroup()] = 0;
    }
    txBuf[6] = txBuf[7] = 0;

    rslt |= this->send(this->ctrlId_, txBuf, 8);
    return rslt;
}

TripMotorGroup_s *DJI_ODMotor::findGroup() const
{
    TripMotorGroup_s *group = nullptr;
    for (auto &entry : getMotorMap()) {
        if (entry.first == this->pComHandle_) {
            auto it = entry.second.find(this->getGroupId()); // it" is a map
            if (it != entry.second.end()) {
                group = it->second;
                break;
            }
        }
    }
    return group;
}
