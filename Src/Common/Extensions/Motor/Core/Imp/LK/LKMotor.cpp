#include "LKMotor.hpp"
#include "MotorCommonMacros.hpp"
#include <algorithm>

using namespace PINYMOTOR;
using namespace LKMOTOR;

LKMotor::LKMotor(const char *name, InitConfig_s config, uint8_t index)
        : Base(name, config), currentTxId_(0), motorIndex_(index)
{
    this->AUX_.rxQueue = xQueueCreate(4, sizeof(RxBus_s::CANRxBuf_s<8>));
    this->cmdQueue_ = xQueueCreate(4, sizeof(Cmd_s));

    this->registerRecvCallback();
    this->updateTxId();

    LOG::info("LKMotor", "%s: Instance created (index: %d, group ID: 0x%hx)",
              this->regInfo_.name, index, this->getGroupId());
}

MotorTypeDef_e LKMotor::init()
{
    MotorTypeDef_e cfgResult = this->checkBaseConfig();
    if (cfgResult != MOTOR_OK) {
        LOG::error("LKMotor", "%s: Config check failed", this->regInfo_.name);
        return MOTOR_ERROR;
    }

    this->initModelParams();
    LOG::info("LKMotor", "%s: Init success", this->regInfo_.name);
    return MOTOR_OK;
}

void LKMotor::overrideStats(const Status_s &newStats)
{
    this->status_ = newStats;
    this->regInfo_.model.rxBaseId = newStats.rxBaseId;
    this->regInfo_.model.txBaseId = newStats.txCurrId;
    this->regInfo_.model.reductionRatio = newStats.reductionRatio;
    this->regInfo_.model.measureMax = static_cast<int>(newStats.encoderSpan);
    this->updateTxId();
}

void LKMotor::overrideReductionRatio(float _newReductionRatio)
{
    this->status_.reductionRatio = _newReductionRatio;
    this->regInfo_.model.reductionRatio = _newReductionRatio;
    LOG::info("LKMotor", "%s: Reduction ratio updated to %.2f",
              this->regInfo_.name, _newReductionRatio);
}

uint16_t LKMotor::canId() const
{
    return this->status_.rxBaseId + this->motorIndex_;
}

uint16_t LKMotor::masterId() const { return this->status_.rxBaseId; }

uint16_t LKMotor::uid() const { return this->canId(); }

void LKMotor::registerRecvCallback()
{
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(this->regInfo_.pComHandle),
            this->canId(), [this](const uint8_t *rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                RxBus_s::CANRxBuf_s<8> buf;
                memcpy(buf.data, rxBuf, 8);
                xQueueSendFromISR(this->AUX_.rxQueue, &buf,
                                  &higherPriorityTaskWoken);
                portYIELD_FROM_ISR(higherPriorityTaskWoken);
            });
    LOG::info("LKMotor", "%s: Recv callback registered (CAN ID: 0x%hx)",
              this->regInfo_.name, this->canId());
}

void LKMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(this->regInfo_.pComHandle),
            this->canId());
    LOG::info("LKMotor", "%s: Recv callback canceled (CAN ID: 0x%hx)",
              this->regInfo_.name, this->canId());
}

void LKMotor::updateTxId()
{
    this->currentTxId_ = this->status_.txCurrId;
    LOG::info("LKMotor", "%s: TX ID set to 0x%hx (QUAD_CURR)",
              this->regInfo_.name, this->currentTxId_);
}

MotorTypeDef_e LKMotor::send(uint16_t sendId, uint8_t *txBuf, uint8_t len)
{
    if (this->group_ == nullptr) {
        LOG::error("LKMotor", "%s: Group not found", this->regInfo_.name);
        return MOTOR_ERROR;
    }

    if (sendId != this->status_.txCurrId) {
        LOG::error("LKMotor", "%s: Invalid TX ID (only 0x%hx allowed)",
                   this->regInfo_.name, this->status_.txCurrId);
        return MOTOR_ERROR;
    }

    if (this->checkGroupSend(this->group_)) {
        return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                reinterpret_cast<canHandle *>(this->regInfo_.pComHandle),
                sendId, txBuf, len));
    }
    return MOTOR_OK;
}

MotorTypeDef_e LKMotor::parse(const RxBus_s::CANRxBuf_s<8> &rxBuf)
{
    Feedback_s fb;
    const uint8_t *data = rxBuf.data;
    fb.cmd = data[0];
    fb.temperature = data[1];
    fb.current = static_cast<int16_t>((data[2] << 8) | data[3]);
    fb.speed = static_cast<int16_t>((data[4] << 8) | data[5]);
    fb.angle = static_cast<uint16_t>((data[6] << 8) | data[7]);

    // 电流转换
    float current = fb.current / this->status_.currentDataMax *
                    this->status_.currentMax;
    this->data_.curr = this->regInfo_.isReverse ? -current : current;

    // 扭矩转换
    this->data_.torq = this->data_.curr * this->status_.torqueConstant;

    // 速度/角度解析
    this->data_.spdRpm = this->regInfo_.isReverse ?
                                 -static_cast<float>(fb.speed) :
                                 static_cast<float>(fb.speed);
    this->data_.spdRadps = this->data_.spdRpm * PI / 180.0f;

    float angle = static_cast<float>(fb.angle) / this->status_.encoderSpan *
                  2.0f * PI;
    this->data_.rawAng = this->regInfo_.isReverse ? (2.0f * PI - angle) : angle;
    float del = this->data_.rawAng - this->data_.zeroAng;
    this->data_.ang = del < 0 ? del + 2.0f * PI : del;

    this->data_.tempture = fb.temperature;

    // 在线状态检测
    if (this->globalState == GlobalState_e::OFFLINE &&
        this->data_.angLast != this->data_.rawAng) {
        this->globalState = GlobalState_e::ONLINE;
        this->data_.multipCirAng =
                this->data_.rawAng / this->status_.reductionRatio;
    }

    // 多圈角度计算
    float angDiff =
            getMinorArc(this->data_.rawAng, this->data_.angLast, 2.0f * PI) /
            this->status_.reductionRatio;
    this->data_.angLast = this->data_.rawAng;
    this->data_.multipCirAng += angDiff;
    this->data_.cirNum = this->data_.multipCirAng / (2.0f * PI);
    this->data_.singleCirAng = rangeMap(this->data_.multipCirAng);

    return MOTOR_OK;
}

MotorTypeDef_e LKMotor::ctrl()
{
    MotorTypeDef_e result = MOTOR_OK;
    int16_t ctrlCmd = 0;

    switch (this->cmd_.curCmdType) {
    case MotorCmdType_e::SET_ELEC:
        ctrlCmd = static_cast<int16_t>(this->cmd_.elec /
                                       this->status_.currentMax *
                                       this->status_.currentDataMax);
        break;
    case MotorCmdType_e::SET_TORQ:
        ctrlCmd = static_cast<int16_t>(
                (this->cmd_.torq / this->status_.torqueConstant) /
                this->status_.currentMax * this->status_.currentDataMax);
        break;
    default:
        LOG::error("LKMotor",
                   "%s: Unsupported cmd type (only SET_ELEC/SET_TORQ)",
                   this->regInfo_.name);
        result = MOTOR_ERROR;
        break;
    }

    if (this->regInfo_.isReverse) {
        ctrlCmd = -ctrlCmd;
    }

    ctrlCmd = std::clamp(ctrlCmd,
                         static_cast<int16_t>(-this->status_.currentDataMax),
                         static_cast<int16_t>(this->status_.currentDataMax));

    this->txBuf_.data[this->motorIndex_] = ctrlCmd;
    this->txBuf_.sendFlag = this->cmd_.SW;

    return result;
}

MotorTypeDef_e LKMotor::update()
{
    if (xQueueReceive(this->AUX_.rxQueue, &this->rxBuf_, 0) == pdTRUE) {
        this->parse(this->rxBuf_);
        this->calcRecvFreq();
    }

    if (xQueueReceive(this->cmdQueue_, &this->cmd_, 0) == pdTRUE) {
        this->parseCmd();
        if (this->regInfo_.workMode != WorkMode_e::QUAD_CURR) {
            LOG::warn("LKMotor", "%s: Work mode forced to QUAD_CURR",
                      this->regInfo_.name);
            this->regInfo_.workMode = WorkMode_e::QUAD_CURR;
        }
        this->updateTxId();
    }

    MotorTypeDef_e ctrlResult = this->ctrl();
    if (ctrlResult == MOTOR_OK) {
        this->send(this->currentTxId_,
                   reinterpret_cast<uint8_t *>(&this->txBuf_.data), 8);
    }

    return ctrlResult;
}

void LKMotor::mountToBroadcastFrame(Transmit_s &broadcastFrame)
{
    if (this->txBuf_.sendFlag) {
        broadcastFrame.data[this->motorIndex_] =
                this->txBuf_.data[this->motorIndex_];
    } else {
        broadcastFrame.data[this->motorIndex_] = 0;
    }
    broadcastFrame.sendFlag = broadcastFrame.sendFlag || this->txBuf_.sendFlag;
}