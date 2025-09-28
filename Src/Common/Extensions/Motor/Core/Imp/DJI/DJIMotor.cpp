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
        voltMax = _other.voltMax;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        Kn = _other.Kn;
    }
    return *this;
}

DJIMotor::DJIMotor(const char _name[16], InitConfig_s _config)
        : Base(_name, _config)
{
    AUX_.rxQueue = xQueueCreate(4, sizeof(RxBus_s::CANRxBuf_s<8>::data));
}
DJIMotor::~DJIMotor()
{
    this->cancelRecvCallback(regInfo_.model.rxBaseId + regInfo_.offsetId);
    this->cancelMotor();
    this->removeMotorFromMap();
    LOG::info("DJIMotor", " %s: An instance of DJIMotor destroyed",
              regInfo_.name);
}

void DJIMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

uint16_t DJIMotor::canId() const { return regInfo_.model.txBaseId + 0u; }

void DJIMotor::registerRecvCallback(uint16_t _rxId)
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId,
            [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(AUX_.rxQueue, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
    LOG::info("DJIMotor", " %s: Receive cb registed, masterId:%hx",
              regInfo_.name, _rxId);
}

void DJIMotor::cancelRecvCallback(uint16_t _rxId)
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId);
    LOG::info("DJIMotor", " %s: Receive cb canceled, masterId:%hx",
              regInfo_.name, _rxId);
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
            // LOG::warn("DJIMotor", " %s: send data to CAN %hx", regInfo_.name,
            //           _sendId);
            // LOG::warn("DJIMotor",
            //           " %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
            //           regInfo_.name, _txBuf[0], _txBuf[1], _txBuf[2], _txBuf[3],
            //           _txBuf[4], _txBuf[5], _txBuf[6], _txBuf[7]);

            return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                    reinterpret_cast<canHandle *>(regInfo_.pComHandle), _sendId,
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

    float noumenaAng = static_cast<float>(fb.rawAng) / this->span() * 2.f * PI;
    this->data_.rawAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng :
                                              noumenaAng;
    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + (2.f * PI) : del;

    float noumenaCurr = static_cast<float>(fb.current) /
                        this->status_.currRxCodeSpan * this->status_.currMax;
    this->data_.curr = regInfo_.isReverse ? -noumenaCurr : noumenaCurr;
    this->data_.torq = this->data_.curr * status_.Kn;

    float noumenaRpm = static_cast<float>(fb.rawRpm) / this->rr();
    this->data_.spdRpm = regInfo_.isReverse ? -noumenaRpm : noumenaRpm;
    this->data_.spdRadps = rpm2radps(this->data_.spdRpm);

    this->data_.tempture = fb.temperature;

    float angDiff =
            (getMinorArc(this->data_.rawAng, this->data_.angLast)) / this->rr();

    if ((this->globalState == GlobalState_e::OFFLINE ||
         this->globalState == GlobalState_e::UNRECOGNIZED) &&
        this->data_.angLast != this->data_.rawAng) {
        this->globalState = GlobalState_e::ONLINE;
        angDiff = 0;
        this->data_.multipCirAng =
                this->data_.rawAng / this->rr(); // 与电机内编码器同步零点
    }
    this->data_.angLast = this->data_.rawAng;

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
    if (this->cmd_.SW) {
        (this->*convert)();
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
    if (xQueueReceive(AUX_.rxQueue, this->rxBuf_.data, 0) == pdTRUE) {
        AUX_.recvCnt++;
        this->parse(this->rxBuf_);
    }

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    this->calcRecvFreq();
    MotorTypeDef_e rslt = ctrl();
    return rslt;
}

void DJIMotor::overrideReductionRatio(float _newReductionRatio)
{
    regInfo_.model.reductionRatio = _newReductionRatio;
    status_.torqMax *= _newReductionRatio;
    status_.Kn *= _newReductionRatio;
    LOG::info("DJIMotor", " %s: you have changed reduction ratio to %f",
              regInfo_.name, _newReductionRatio);
}
