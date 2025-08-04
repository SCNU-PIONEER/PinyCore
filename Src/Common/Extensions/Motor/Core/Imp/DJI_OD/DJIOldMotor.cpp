
#include "DJIOldMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

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

DJIOldMotor::DJIOldMotor(const char _name[16], InitConfig_s _config)
        : Base(_name, std::move(_config))
{
    this->rxQueue_ = xQueueCreate(10, sizeof(RxBus_s::CANRxBuf_s));
}

DJIOldMotor::~DJIOldMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    LOG::info(
            "DJIOldMotor",
            " %s: An instance of DJIOldMotor created, rxBaseId:%hx, txBaseId:%hx",
            this->name_, this->model_.rxBaseId, this->model_.txBaseId);
    this->removeMotorFromMap();
}

void DJIOldMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

uint16_t DJIOldMotor::canId() const { return this->model_.txBaseId + 0u; }

uint16_t DJIOldMotor::masterId() const
{
    return this->model_.rxBaseId + this->offsetId_;
}

uint16_t DJIOldMotor::uid() { return masterId(); }

void DJIOldMotor::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->rxQueue_, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
    LOG::info("DJIOldMotor", " %s: Receive cb registed, masterId:%hx",
              this->name_, this->masterId());
}

void DJIOldMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId());
    LOG::info("DJIOldMotor", " %s: Receive cb canceled, masterId:%hx",
              this->name_, this->masterId());
}

void DJIOldMotor::updateCtrlId()
{
    switch (this->workMode_) {
    case WorkMode_e::TRIP_VOLT: {
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    }
    default: {
        LOG::error("DJIOldMotor", " %s: this mode is not supported",
                   this->name_);
        break;
    }
    }
}

MotorTypeDef_e DJIOldMotor::send(uint16_t _sendId, uint8_t *_txBuf,
                                 uint8_t _len)
{
    // send data to CAN
    if (this->group_ == nullptr) {
        LOG::error("DJIOldMotor", " %s: Can't find group %hx", this->name_,
                   this->getGroupId());
        return 1;
    } else {
        if (this->checkGroupSend(this->group_)) {
            // Check this Buffer
            // LOG::warn("DJIOldMotor", " %s: send data to CAN %hx", this->name_,
            //           _sendId);
            // LOG::warn("DJIOldMotor",
            //           " %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
            //           this->name_, _txBuf[0], _txBuf[1], _txBuf[2], _txBuf[3],
            //           _txBuf[4], _txBuf[5], _txBuf[6], _txBuf[7]);

            return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                    reinterpret_cast<canHandle *>(this->pComHandle_), _sendId,
                    _txBuf, _len));
        } else {
            return 0;
        }
    }
}

MotorTypeDef_e DJIOldMotor::parse(const RxBus_s::CANRxBuf_s &_rxBuf)
{
    Feedback_s fb;
    fb.rawAng = ((_rxBuf.data[0] << 8) | _rxBuf.data[1]);
    fb.rawTorq = static_cast<int16_t>(((_rxBuf.data[2] << 8) | _rxBuf.data[3]));

    this->data_.rawAng =
            static_cast<float>(fb.rawAng) / this->span() * 2.f * PI;

    this->data_.torq = static_cast<float>(fb.rawTorq) /
                       this->status_.torqRxCodeSpan * this->status_.torqMax;

    this->data_.tempture = 0.f; // TODO:

    this->data_.curr = this->data_.torq / status_.torqConstant;

    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + (2.f * PI) : del;

    float angDiff = (getMinorArc(this->data_.rawAng, this->data_.rawAngLast,
                                 2.f * PI)) /
                    this->rr();

    if (this->globalState_ == GlobalState_e::OFFLINE &&
        this->data_.rawAngLast != this->data_.rawAng) {
        this->globalState_ = GlobalState_e::ONLINE;
        angDiff = 0;
        this->data_.multipCirAng =
                this->data_.rawAng / this->rr(); // 与电机内编码器同步零点
    }

    if (this->rxFreq_ > 1.f)
        this->data_.spdRadps = angDiff * this->rxFreq_;

    this->data_.spdRpm = radps2rpm(this->data_.spdRadps);

    this->data_.rawAngLast = this->data_.rawAng;

    this->data_.multipCirAng += angDiff;
    this->data_.singleCirAng = rangeMap(this->data_.multipCirAng, 0, 2.f * PI);
    return 0;
}

MotorTypeDef_e DJIOldMotor::ctrl()
{
    MotorTypeDef_e rslt = 0;
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
                LOG::error("DJIOldMotor", " %s: torqPID is null", this->name_);
            }
        } else if (this->cmd_.curCmdType == MotorCmdType_e::SET_VEL) {
            if (this->velPID_ != nullptr || this->torqPID_ != nullptr) {
                this->cmd_.torq = this->velPID_->calc(this->cmd_.vel,
                                                      this->data_.spdRadps);
                this->cmd_.elec =
                        this->torqPID_->calc(this->cmd_.torq, this->data_.torq);
            } else {
                LOG::error("DJIOldMotor", " %s: velPID or torqPID is null",
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
                LOG::error("DJIOldMotor",
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
        LOG::error("DJIOldMotor", " %s: this mode is not supported",
                   this->name_);
        break;
    }
    }
    if (this->cmd_.SW) {
        this->group_->txBuf[(2 * this->getPosInGroup()) + 1] =
                static_cast<uint8_t>(ctrlCmd & 0xFF);
        this->group_->txBuf[2 * this->getPosInGroup()] =
                static_cast<uint8_t>((ctrlCmd >> 8) & 0xFF);
    } else {
        if (this->posPID_ != nullptr)
            this->posPID_->reset();
        if (this->velPID_ != nullptr)
            this->velPID_->reset();
        if (this->torqPID_ != nullptr)
            this->torqPID_->reset();
        this->group_->txBuf[(2 * this->getPosInGroup()) + 1] = 0;
        this->group_->txBuf[2 * this->getPosInGroup()] = 0;
    }
    this->group_->txBuf[6] = this->group_->txBuf[7] = 0;

    rslt |= this->send(this->ctrlId_, this->group_->txBuf, 8);
    return rslt;
}

MotorTypeDef_e DJIOldMotor::update()
{
    if (xQueueReceive(this->rxQueue_, this->rxBuf_.data, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
    }
    if (xQueueReceive(this->cmdQueue_, &this->cmdBuf_, 0) == pdTRUE) {
        this->parseCmd();
    }
    MotorTypeDef_e rslt = ctrl();
    return rslt;
}
