#include "DJIMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

#include "Bsp_can.hpp"

using namespace PINYMOTOR;
using namespace DJIMOTOR;

Status_s &Status_s::operator=(const Status_s &_other)
{
    if (this != &_other) {
        voltTxCodeSpan = _other.voltTxCodeSpan;
        currTxCodeSpan = _other.currTxCodeSpan;
        currRxCodeSpan = _other.currRxCodeSpan;
        currRated = _other.currRated;
        torqRated = _other.torqRated;
        voltMax = _other.voltMax;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        torqConstant = _other.torqConstant;
    }
    return *this;
}

DJIMotor::DJIMotor(const char _name[16], InitConfig_s _config)
        : Base(_name, std::move(_config))
        , convert(selectWorkMode(this->workMode_))
{
    this->rxQueue_ = xQueueCreate(4, sizeof(RxBus_s::CANRxBuf_s<8>));
}
DJIMotor::~DJIMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    this->removeMotorFromMap();
    LOG::info("DJIMotor", " %s: An instance of DJIMotor destroyed",
              this->name_);
}

void DJIMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

uint16_t DJIMotor::canId() const { return this->model_.txBaseId + 0u; }

uint16_t DJIMotor::masterId() const
{
    return this->model_.rxBaseId + this->offsetId_;
}

uint16_t DJIMotor::uid() { return masterId(); }

void DJIMotor::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId(),
            [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(this->rxQueue_, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
    LOG::info("DJIMotor", " %s: Receive cb registed, masterId:%hx", this->name_,
              this->masterId());
}

void DJIMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(this->pComHandle_), this->masterId());
    LOG::info("DJIMotor", " %s: Receive cb canceled, masterId:%hx", this->name_,
              this->masterId());
}

void DJIMotor::updateCtrlId()
{
    switch (this->workMode_) {
    case WorkMode_e::QUAD_CURR:
    case WorkMode_e::QUAD_VOLT: {
        this->ctrlId_ = this->getGroupId() + 0u;
        break;
    }
    default: {
        LOG::error("DJIMotor", " %s: this mode is not supported", this->name_);
        break;
    }
    }
}

MotorTypeDef_e DJIMotor::send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len)
{
    // send data to CAN
    if (this->group_ == nullptr) {
        LOG::error("DJIMotor", " %s: Can't find group %hx", this->getGroupId());
        return 1;
    } else {
        if (this->checkGroupSend(this->group_)) {
            // Check this Buffer
            // LOG::warn("DJIMotor", " %s: send data to CAN %hx", this->name_,
            //           _sendId);
            // LOG::warn("DJIMotor",
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

MotorTypeDef_e DJIMotor::parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf)
{
    Feedback_s fb;
    fb.rawAng = ((_rxBuf.data[0] << 8) | _rxBuf.data[1]);
    fb.rawRpm = static_cast<int16_t>(((_rxBuf.data[2] << 8) | _rxBuf.data[3]));
    fb.current = static_cast<int16_t>((_rxBuf.data[4] << 8) | _rxBuf.data[5]);
    fb.temperature = _rxBuf.data[6];

    this->data_.rawAng =
            static_cast<float>(fb.rawAng) / this->span() * 2.f * PI;

    this->data_.curr = static_cast<float>(fb.current) /
                       this->status_.currRxCodeSpan * this->status_.currMax;
    this->data_.tempture = fb.temperature;

    this->data_.torq = this->data_.curr * status_.torqConstant;
    this->data_.spdRpm = static_cast<float>(fb.rawRpm) / this->rr();
    this->data_.spdRadps = rpm2radps(this->data_.spdRpm);

    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + (2.f * PI) : del;

    float angDiff = (getMinorArc(this->data_.rawAng, this->data_.rawAngLast,
                                 2.f * PI)) /
                    this->rr();

    if ((this->globalState_ == GlobalState_e::OFFLINE ||
         this->globalState_ == GlobalState_e::UNREGISTER) &&
        this->data_.rawAngLast != this->data_.rawAng) {
        this->globalState_ = GlobalState_e::ONLINE;
        angDiff = 0;
        this->data_.multipCirAng =
                this->data_.rawAng / this->rr(); // 与电机内编码器同步零点
    }
    this->data_.rawAngLast = this->data_.rawAng;

    this->data_.multipCirAng += angDiff;
    this->data_.cirNum = this->data_.multipCirAng / (2.f * PI);

    if (this->rr() == 1) // TODO: 因fmod的精度问题 多圈始终不准
        this->data_.singleCirAng = this->data_.ang;
    else
        this->data_.singleCirAng = rangeMap(this->data_.multipCirAng);

    return 0;
}

MotorTypeDef_e DJIMotor::ctrl()
{
    MotorTypeDef_e rslt = 0;

    int16_t ctrlCmd = (this->*convert)();

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

    rslt |= this->send(this->ctrlId_, this->group_->txBuf, 8);
    return rslt;
}

MotorTypeDef_e DJIMotor::update()
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
