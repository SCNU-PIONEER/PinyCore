#include "LKMotor.hpp"
#include "Bsp_can.hpp"
#include "MotorCommonMacros.hpp"
#include <algorithm>
#include <sys/reent.h>

using namespace PINYMOTOR;
using namespace LKMOTOR;

LKMotor::LKMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode)
        : Base(_name, _config), workMode_(_workMode)
{
    this->regInfo_.model.rxBaseId = RX_BASE_ID;
    this->regInfo_.model.txBaseId = TX_BASE_ID;
    motorIndex_ = _config.offsetId;

    AUX_.rxQueue = xQueueCreate(4, sizeof(RxBus_s::CANRxBuf_s<8>::data));
    this->updateTxId();
    this->updateMotorMap();
    this->registerRecvCallback();

    LOG::info("LKMotor", "%s: Instance created (index: %d, group ID: 0x%hx)", this->regInfo_.name, this->motorIndex_,
              this->getGroupId());
}

LKMotor::~LKMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    this->removeMotorFromMap();
    LOG::info("LKMotor", "%s: Instance destroyed (recv ID: 0x%hx)", this->regInfo_.name, this->canId());
}

void LKMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

void LKMotor::overrideReductionRatio(float _newReductionRatio)
{
    this->regInfo_.model.reductionRatio = _newReductionRatio;
    this->status_.torqueMax *= _newReductionRatio;
    this->status_.torqueConstant *= _newReductionRatio;

    LOG::info("LKMotor", "%s: Reduction ratio updated to %.2f", this->regInfo_.name, _newReductionRatio);
}

uint16_t LKMotor::canId() const { return TX_BASE_ID; }
uint16_t LKMotor::masterId() const { return RX_BASE_ID + (this->motorIndex_); }

void LKMotor::registerRecvCallback()
{
    Can::instance().registerCallback(reinterpret_cast<canHandle *>(this->regInfo_.pComHandle), this->masterId(),
                                     [this](const uint8_t *_rxBuf) {
                                         BaseType_t higherPriorityTaskWoken = pdFALSE;
                                         RxBus_s::CANRxBuf_s<8> buf;
                                         memcpy(buf.data, _rxBuf, 8);
                                         xQueueSendFromISR(this->AUX_.rxQueue, &buf, &higherPriorityTaskWoken);
                                     });
    LOG::info("LKMotor", "%s: Recv callback registered (CAN ID: 0x%hx)", this->regInfo_.name, this->masterId());
}

void LKMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(reinterpret_cast<canHandle *>(this->regInfo_.pComHandle), this->masterId());
    LOG::info("LKMotor", "%s: Recv callback canceled (CAN ID: 0x%hx)", regInfo_.name, this->masterId());
}

void LKMotor::updateTxId()
{
    this->currentTxId_ = 0x280;
    LOG::info("LKMotor", "%s: TX ID set to 0x%hx (BROADCAST_TORQUE)", this->regInfo_.name, this->currentTxId_);
}

MotorTypeDef_e LKMotor::send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len)
{
    if (this->checkGroupSend(this->group_)) {
        return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                reinterpret_cast<canHandle *>(this->regInfo_.pComHandle), _sendId, _txBuf, _len));
    }
    return 0;
}

MotorTypeDef_e LKMotor::parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf)
{
    Feedback_s fb;
    const uint8_t *data = _rxBuf.data;
    fb.cmd = data[0];
    fb.temperature = data[1];
    fb.current = static_cast<int16_t>((data[3] << 8) | data[2]);
    fb.speed = static_cast<int16_t>((data[5] << 8) | data[4]);
    fb.angle = static_cast<uint16_t>((data[7] << 8) | data[6]);

    float current = static_cast<float>(fb.current) / this->status_.rxcurrentDataMax * this->status_.rxcurrentMax;
    this->data_.curr = this->regInfo_.isReverse ? -current : current;

    this->data_.torq = this->data_.curr * this->status_.torqueConstant;

    float spdDpsMotor = this->regInfo_.isReverse ? -static_cast<float>(fb.speed) : static_cast<float>(fb.speed);
    float spdDpsLoad = spdDpsMotor / this->rr();
    this->data_.spdRpm = spdDpsLoad * (60.0f / 360.0f);
    this->data_.spdRadps = spdDpsLoad * PI / 180.0f;

    float angle = static_cast<float>(fb.angle) / this->span() * 2.0f * PI;
    this->data_.rawAng = this->regInfo_.isReverse ? ((2.0f * PI) - angle) : angle;
    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + (2.0f * PI) : del;

    this->data_.tempture = fb.temperature;

    this->data_.singleCirAng = rangeMap(data_.ang / this->rr());

    if (this->globalState == GlobalState_e::OFFLINE || this->globalState == GlobalState_e::UNRECOGNIZED) {
        this->globalState = GlobalState_e::ONLINE;
        this->data_.angLast = this->data_.ang;
    }

    float angDiff = this->data_.ang - this->data_.angLast;
    if (angDiff > PI) {
        this->data_.cirNum -= 1.f / this->rr();
    } else if (angDiff < -PI) {
        this->data_.cirNum += 1.f / this->rr();
    }

    this->data_.multipCirAng = this->data_.singleCirAng + (TWO_PI * this->data_.cirNum);
    this->data_.angLast = this->data_.ang;

    return 0;
}

MotorTypeDef_e LKMotor::ctrl()
{
    MotorTypeDef_e rslt = 0;
    int16_t ctrlCmd = 0;
    if (!this->cmd_.SW) {
        ctrlCmd = 0;
    } else {
        switch (this->cmd_.curCmdType) {
        case MotorCmdType_e::SET_ELEC:
            ctrlCmd =
                    static_cast<int16_t>(this->cmd_.elec / this->status_.txcurrentMax * this->status_.txcurrentDataMax);
            break;
        case MotorCmdType_e::SET_TORQ:
            ctrlCmd = static_cast<int16_t>((this->cmd_.torq / this->status_.torqueConstant) /
                                           this->status_.txcurrentMax * this->status_.txcurrentDataMax);
            break;
        default:
            LOG::error("LKMotor", "%s: Unsupported cmd type (only SET_ELEC/SET_TORQ)", this->regInfo_.name);
            rslt = 1;
            break;
        }
    }
    if (this->regInfo_.isReverse) {
        ctrlCmd = static_cast<int16_t>(-ctrlCmd);
    }
    ctrlCmd = std::clamp(ctrlCmd, static_cast<int16_t>(-this->status_.txcurrentDataMax),
                         static_cast<int16_t>(this->status_.txcurrentDataMax));

    this->group_->txBuf[2 * this->getPosInGroup()] = static_cast<uint8_t>(ctrlCmd & 0xFF);
    this->group_->txBuf[(2 * this->getPosInGroup()) + 1] = static_cast<uint8_t>((ctrlCmd >> 8) & 0xFF);

    this->send(this->currentTxId_, this->group_->txBuf, 8);
    return rslt;
}

MotorTypeDef_e LKMotor::update()
{
    if (xQueueReceive(this->AUX_.rxQueue, &this->rxBuf_, 0) == pdTRUE) {
        AUX_.recvCnt++;
        this->parse(this->rxBuf_);
    }

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    this->calcRecvFreq();
    MotorTypeDef_e ctrlResult = this->ctrl();

    return ctrlResult;
}

bool LKMotor::isSupportMode(WorkMode_e _mode) const
{
    switch (_mode) {
    case WorkMode_e::VOLT:
    case WorkMode_e::VDES:
    case WorkMode_e::MIT_TT:
    case WorkMode_e::SINGLE_PDES:
    case WorkMode_e::SINGLE_PDESVDES:
    case WorkMode_e::MULTI_PDESVDES:
    case WorkMode_e::MULTI_PDES:
    case WorkMode_e::PDESVDES:
    case WorkMode_e::INC_PDES:
    case WorkMode_e::INC_PDESVDES:
        return true;
    default:
        return false;
    }
}