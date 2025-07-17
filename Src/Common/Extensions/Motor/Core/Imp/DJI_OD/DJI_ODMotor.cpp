
#include "DJI_ODMotor.hpp"

#include "../../../Utils/MotorCommonMacros.hpp"

#include "Bsp_can.hpp"

using namespace PINYMOTOR;
using namespace DJI_ODMOTOR;

Status_s &Status_s::operator=(const Status_s &_other)
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
        : Base(_name, std::move(_config))
{
    this->rxQueue_ = xQueueCreate(10, sizeof(RxBus_s::CANRxBuf_s));
}

DJI_ODMotor::~DJI_ODMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    LOG::info(
            "DJI_ODMotor",
            " %s: An instance of DJI_ODMotor created, rxBaseId:%hx, txBaseId:%hx",
            this->name_, this->model_.rxBaseId, this->model_.txBaseId);
    this->removeMotorFromMap();
}

void DJI_ODMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

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
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->rxQueue_, const_cast<uint8_t *>(_rxBuf),
                                  &higherPriorityTaskWoken);
            });
    LOG::info("DJI_ODMotor", " %s: Receive cb registed, masterId:%hx",
              this->name_, this->masterId());
}

void DJI_ODMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId());
    LOG::info("DJI_ODMotor", " %s: Receive cb canceled, masterId:%hx",
              this->name_, this->masterId());
}

void DJI_ODMotor::updateCtrlId()
{
    switch (this->workMode_) {
    case WorkMode_e::TRIP_VOLT: {
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    }
    default: {
        LOG::error("DJI_ODMotor", " %s: this mode is not supported",
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
        LOG::error("DJI_ODMotor", " %s: Can't find group %hx", this->name_,
                   this->getGroupId());
        return 1;
    } else {
        if (this->checkGroupSend(group)) {
#if 0
            // Check this Buffer
            LOG::debug("DJI_ODMotor", " %s: send data to CAN %hx",
                      this->name_, _sendId);
            LOG::debug("DJI_ODMotor",
                      " %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
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

MotorTypeDef_e DJI_ODMotor::parse(const RxBus_s::CANRxBuf_s &_rxBuf)
{
    Feedback_s fb;
    fb.rawScale = ((_rxBuf.data[0] << 8) | _rxBuf.data[1]);
    fb.rawTorq = static_cast<int16_t>(((_rxBuf.data[2] << 8) | _rxBuf.data[3]));

    this->data_.rawScale = fb.rawScale;

    this->data_.torq =
            fb.rawTorq / this->status_.torqRxCodeSpan * this->status_.torqMax;

    this->data_.tempture = 0.f; // TODO:

    this->data_.curr = this->data_.torq / status_.torqConstant;

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
        if (this->cmd_.curCmdType == MotorCmdType_e::SET_ELEC) {
            this->cmd_.elec;
        } else if (this->cmd_.curCmdType == MotorCmdType_e::SET_TORQ) {
            if (this->torqPID_ != nullptr) {
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                LOG::error("DJI_ODMotor", " %s: torqPID is null", this->name_);
            }
        } else if (this->cmd_.curCmdType == MotorCmdType_e::SET_VEL) {
            if (this->velPID_ != nullptr || this->torqPID_ != nullptr) {
                this->cmd_.torq = this->velPID_->calc(this->cmd_.vel,
                                                      this->data_.spdRadps);
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                LOG::error("DJI_ODMotor", " %s: velPID or torqPID is null",
                           this->name_);
            }
        } else if (this->cmd_.curCmdType == MotorCmdType_e::SET_POS) {
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
                LOG::error("DJI_ODMotor",
                           " %s: posPID or velPID or torqPID is null",
                           this->name_);
            }
        } else if (this->cmd_.curCmdType == MotorCmdType_e::SET_MIT) {
            if (this->posPID_ != nullptr || this->velPID_ != nullptr ||
                this->torqPID_ != nullptr) {
                this->cmd_.torq =
                        this->posPID_->calc(
                                getMinorArc(this->cmd_.pos,
                                            this->data_.singleCirAng, 2.f * PI),
                                0) +
                        this->velPID_->calc(this->cmd_.vel,
                                            this->data_.spdRadps) +
                        this->cmd_.torq;
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                LOG::error("DJIMotor", " %s: posPID or velPID is null",
                           this->name_);
            }
        }
        ctrlCmd = static_cast<int16_t>(this->cmd_.elec / this->status_.voltMax *
                                       this->status_.voltTxCodeSpan);
        break;
    }
    default: {
        ctrlCmd = 0;
        LOG::error("DJI_ODMotor", " %s: this mode is not supported",
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
        if (this->posPID_ != nullptr)
            this->posPID_->reset();
        if (this->velPID_ != nullptr)
            this->velPID_->reset();
        if (this->torqPID_ != nullptr)
            this->torqPID_->reset();
        txBuf[2 * this->getPosInGroup() + 1] = 0;
        txBuf[2 * this->getPosInGroup()] = 0;
    }
    txBuf[6] = txBuf[7] = 0;

    rslt |= this->send(this->ctrlId_, txBuf, 8);
    return rslt;
}

MotorTypeDef_e DJI_ODMotor::update()
{
    if (xQueueReceive(this->rxQueue_, this->rxBuf_.data, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
        if (this->userRecvCallback_ != nullptr) {
            this->userRecvCallback_(this->rxBuf_.data);
        }
    }
    if (xQueueReceive(this->cmdQueue_, &this->cmdBuf_, 0) == pdTRUE) {
        this->parseCmd();
    }
    MotorTypeDef_e rslt = ctrl();
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
