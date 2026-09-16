#include "LKMotor.hpp"
#include "Bsp_can.hpp"
#include "MotorCommonMacros.hpp"
#include <algorithm>
#include <sys/reent.h>

using namespace PINYMOTOR;
using namespace LKMOTOR;

LKMotor::LKMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode)
        : Base(_name, _config), motorIndex_(_config.offsetId), workMode_(_workMode)
{
    this->regInfo_.model.rxBaseId = RX_BASE_ID;
    this->regInfo_.model.txBaseId = TX_BASE_ID;

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
    const State2_s *fb = reinterpret_cast<const State2_s *>(_rxBuf.data + 1);

    data_.tempture = fb->temperature;
    data_.tempture = fb->temperature; // 1°C/1LSB
    // current in A, (66/4096 A) / LSB, for MG motor;(33/4096 A) / LSB, for MF motor
    data_.curr = (float)(fb->current) / 4096.f * (float)this->status_.CurrMax;
    data_.torq = data_.curr * this->status_.torqueConstant;
    data_.spdRadps = deg2rad(static_cast<float>(fb->speed)); // 反馈输出轴速度，1dps/LSB
    data_.spdRpm = radps2rpm(data_.spdRadps);

    // 双编码器反馈的是输出轴的编码器数据， 14bit encoder range: 0-16383, 15bit encoder range: 0-32767, 16bit encoder range: 0-65535
    float noumenaAng = static_cast<float>(fb->encoder) / this->span() * 2.f * PI;
    data_.rawAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng : noumenaAng;
    data_.ang = data_.rawAng;

    /*
     * 电机内部的单圈认定范围是[-PI, PI], 如果超过范围，会将当前角度设置为0
     */
    data_.singleCirAng = rangeMap(data_.ang / this->rr(), -PI, PI);

    /* 
     * 当电机断电，状态为离线状态，上电第一刻先赋值 angLast
     */
    if (this->globalState == GlobalState_e::OFFLINE || this->globalState == GlobalState_e::UNRECOGNIZED) {
        this->globalState = GlobalState_e::ONLINE;
        data_.angLast = data_.ang;
    }

    /*
     * 在零度附近编码器会在0和6.28之间跳变，而多圈位置控制2是支持控制正负的，所以通过delta来判断编码器是否跨过零度，并计算圈数
     * 1. 当编码器从0跳变到6.28时，delta会大于PI，说明电机反向跨过零度，圈数减1，并且多圈角度等于编码器值减去2PI
     * 2. 当编码器从6.28跳变到0时，delta会小于-PI，说明电机正向跨过零度，圈数加1，并且多圈角度等于编码器值加上2PI
     * 3. 当编码器在零度附近跳变，多圈角通过抵消从而不会跳变
     */
    float delta = data_.ang - data_.angLast;
    if (delta > PI) {
        this->data_.cirNum -= 1.f / this->rr();
    } else if (delta < -PI) {
        this->data_.cirNum += 1.f / this->rr();
    }

    data_.multipCirAng = data_.singleCirAng + (TWO_PI * data_.cirNum);
    data_.angLast = data_.ang;

    return STM_OK;
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
            ctrlCmd = static_cast<int16_t>(this->cmd_.elec / (float)this->status_.CurrMax * 4096.f);
            break;
        case MotorCmdType_e::SET_TORQ:
            ctrlCmd = static_cast<int16_t>((this->cmd_.torq / this->status_.torqueConstant) /
                                           (float)this->status_.CurrMax * 4096.f);
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
    ctrlCmd = std::clamp(ctrlCmd, (int16_t)-2048, (int16_t)2048);

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