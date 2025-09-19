#include "DMMotor.hpp"

#include "MotorCommonMacros.hpp"

#include "StmLog.hpp"

#include "Bsp_can.hpp"

#include "Soc.hpp"

using namespace PINYMOTOR;
using namespace DMMOTOR;

Status_s &Status_s::operator=(const Status_s &_other)
{
    if (this != &_other) {
        PMax = _other.PMax;
        VMax = _other.VMax;
        TMax = _other.TMax;
        MITKpMax = _other.MITKpMax;
        MITKdMax = _other.MITKdMax;
        currTxCodeSpan = _other.currTxCodeSpan;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        Kn = _other.Kn;
    }
    return *this;
}

DMMotor::DMMotor(const char _name[16], InitConfig_s _config)
        : Base(_name, _config)

{
    AUX_.rxQueue = xQueueCreate(3, sizeof(RxBus_s::CANRxBuf_s<8>::data));
}

DMMotor::~DMMotor()
{
    this->cancelRecvCallback();
    this->cancelMotor();
    LOG::info(
            "DMMotor",
            " %s: An instance of DMMotor created, rxBaseId:0x%hx, txBaseId:0x%hx",
            regInfo_.name, regInfo_.model.rxBaseId, regInfo_.model.txBaseId);
}

void DMMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool DMMotor::isEnable() const { return this->cmd_.SW; }

uint16_t DMMotor::canId() const
{
    return regInfo_.model.txBaseId + regInfo_.offsetId;
}

uint16_t DMMotor::masterId() const
{
    return regInfo_.model.rxBaseId + regInfo_.offsetId;
}

void DMMotor::registerRecvCallback()
{
    // lamda
    Can::instance().registerCallback(
            reinterpret_cast<canHandle *>(regInfo_.pComHandle),
            this->masterId(), [this](const uint8_t *_rxBuf) {
                BaseType_t higherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(AUX_.rxQueue, _rxBuf,
                                  &higherPriorityTaskWoken);
            });
    LOG::info("DMMotor", " %s: Receive cb registed, masterId:%hx",
              regInfo_.name, this->masterId());
}


void DMMotor::cancelRecvCallback()
{
    Can::instance().unregisterCallback(
            reinterpret_cast<canHandle *>(regInfo_.pComHandle),
            this->masterId());
    LOG::info("DMMotor", " %s: Receive cb canceled, masterId:%hx",
              regInfo_.name, this->masterId());
}

void DMMotor::setMITKp(float _kp)
{
    if (_kp < 0 || _kp > status_.MITKpMax) {
        LOG::error("DMMotor", " %s: MITKp out of range", regInfo_.name);
        return;
    }
    MITKp_ = _kp;
}

void DMMotor::setMITKd(float _kd)
{
    if (_kd < 0 || _kd > status_.MITKdMax) {
        LOG::error("DMMotor", " %s: MITKd out of range", regInfo_.name);
        return;
    }
    MITKd_ = _kd;
}

MotorTypeDef_e DMMotor::send(uint16_t _sendId, uint8_t *_txBuf, uint8_t _len)
{
    if (this->checkSend()) {
        // Check this Buffer
        // LOG::warn("DMMotor", " %s: send data to CAN %hx", regInfo_.name, _sendId);
        // LOG::warn("DMMotor",
        //           " %s: txBuf: %02X %02X %02X %02X %02X %02X %02X %02X",
        //           regInfo_.name, _txBuf[0], _txBuf[1], _txBuf[2], _txBuf[3],
        //           _txBuf[4], _txBuf[5], _txBuf[6], _txBuf[7]);

#ifdef SOC_FDCAN
        if (regInfo_.comType == ComType_e::FDCAN) {
            return static_cast<MotorTypeDef_e>(Can::instance().transmitBrsData(
                    reinterpret_cast<canHandle *>(this->pComHandle_), _sendId,
                    _txBuf, _len));
        }
#endif
        if (regInfo_.comType == ComType_e::CAN) {
            return static_cast<MotorTypeDef_e>(Can::instance().transmitData(
                    reinterpret_cast<canHandle *>(regInfo_.pComHandle), _sendId,
                    _txBuf, _len));
        }
    }
    return 0;
}

MotorTypeDef_e DMMotor::parse(const RxBus_s::CANRxBuf_s<8> &_rxBuf)
{
    if (_rxBuf.data[0] == static_cast<uint8_t>(canId()) &&
        _rxBuf.data[1] == static_cast<uint8_t>(canId() >> 8)) {
        uint32_t rawDat = (_rxBuf.data[7] << 24) | (_rxBuf.data[6] << 16) |
                          (_rxBuf.data[5] << 8) | _rxBuf.data[4];
        auto it = regObjList_.find(static_cast<RegId_e>(_rxBuf.data[3]));
        if (_rxBuf.data[2] == 0x33) {
            (*it).second->isRead = true;
            memcpy(&(*it).second->dat, &rawDat, 4);
        } else if (_rxBuf.data[2] == 0x55) {
            (*it).second->isWrite = true;
        } else if (_rxBuf.data[2] == 0xAA) {
            (*it).second->isStorage = true;
        } else {
            LOG::error("DMMotor", " %s: Unknown feedback type, rxBuf[2]:%02X",
                       regInfo_.name, _rxBuf.data[2]);
            return 1;
        }
    } else {
        Feedback_s fb = {};
        fb.ID = _rxBuf.data[0] & 0x0F;
        fb.errorCode = static_cast<ErrorCode_e>(_rxBuf.data[0] >> 4);
        fb.rawAng = (_rxBuf.data[1] << 8) | _rxBuf.data[2];
        fb.rawVel = (_rxBuf.data[3] << 4) | (_rxBuf.data[4] >> 4);
        fb.torque = ((_rxBuf.data[4] & 0xF) << 8 | _rxBuf.data[5]);
        fb.mosTemperature = _rxBuf.data[6];
        fb.rotorTemperature = _rxBuf.data[7];

        errorCode_ = fb.errorCode;

        float noumenaAng =
                static_cast<float>(fb.rawAng) / this->span() * 2.f * PI;
        this->data_.rawAng = regInfo_.isReverse ? (2.f * PI) - noumenaAng :
                                                  noumenaAng;
        float del = this->data_.rawAng - this->data_.zeroAng;
        this->data_.ang = del < 0 ? del + (2.f * PI) : del;

        float noumenaVel =
                uint2float(fb.rawVel, -status_.VMax, status_.VMax, 12) /
                this->rr();
        this->data_.spdRadps = regInfo_.isReverse ? -noumenaVel : noumenaVel;
        this->data_.spdRpm = radps2rpm(this->data_.spdRadps);

        float noumenaTorq =
                uint2float(fb.torque, -status_.TMax, status_.TMax, 12) *
                this->rr();
        this->data_.torq = regInfo_.isReverse ? -noumenaTorq : noumenaTorq;
        this->data_.curr = this->data_.torq / status_.Kn;

        this->data_.tempture = fb.mosTemperature;

        float angDiff = (getMinorArc(this->data_.rawAng, this->data_.angLast)) /
                        this->rr();

        if ((this->globalState_ == GlobalState_e::OFFLINE ||
             this->globalState_ == GlobalState_e::UNREGISTER) &&
            this->data_.angLast != this->data_.rawAng) {
            this->globalState_ = GlobalState_e::ONLINE;
            angDiff = 0;
            this->data_.multipCirAng =
                    this->data_.rawAng / this->rr(); // 与电机内编码器同步零点
        }

        this->data_.angLast = this->data_.rawAng;

        this->data_.multipCirAng += angDiff;
        this->data_.cirNum = this->data_.multipCirAng / (2.f * PI);

        if (this->rr() == 1)
            this->data_.singleCirAng = this->data_.ang;
        else
            this->data_.singleCirAng = rangeMap(this->data_.multipCirAng);
    }
    return 0;
}

MotorTypeDef_e DMMotor::ctrl()
{
    MotorTypeDef_e rslt = 0;
    TxBus txBuf;
    if ((this->cmd_.SW && !this->cmd_.prevSW) ||
        (this->cmd_.SW && errorCode_ == ErrorCode_e::MOTOR_DISABLE)) {
        constexpr uint8_t ENABLE_CMD_MSG[8] = { 0xFF, 0xFF, 0xFF, 0xFF,
                                                0xFF, 0xFF, 0xFF, 0xFC };
        memcpy(txBuf.data, ENABLE_CMD_MSG, 8);
        txBuf.len = 8;
    } else if (!this->cmd_.SW) {
        if (this->posPID_ != nullptr)
            this->posPID_->reset();
        if (this->velPID_ != nullptr)
            this->velPID_->reset();
        if (this->torqPID_ != nullptr)
            this->torqPID_->reset();
        constexpr uint8_t DISABLE_CMD_MSG[8] = { 0xFF, 0xFF, 0xFF, 0xFF,
                                                 0xFF, 0xFF, 0xFF, 0xFD };
        memcpy(txBuf.data, DISABLE_CMD_MSG, 8);
        txBuf.len = 8;
    } else {
        txBuf = (this->*convert)();
    }
    rslt |= this->send(this->ctrlId_, txBuf.data, txBuf.len);
    return rslt;
}

MotorTypeDef_e DMMotor::update()
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

MotorTypeDef_e DMMotor::disable()
{
    MotorTypeDef_e rslt = 0;
    uint8_t disableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD
    };
    rslt |= this->send(this->ctrlId_, disableCmdPack, 8);
    this->cmd_.updateSW(false); // force disable
    return rslt;
}

MotorTypeDef_e DMMotor::clearError()
{
    MotorTypeDef_e rslt = 0;
    uint8_t enableCmdPack[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB
    };
    rslt |= this->send(this->ctrlId_, enableCmdPack, 8);
    return rslt;
}

void DMMotor::overrideReductionRatio(float _newReductionRatio)
{
    regInfo_.model.reductionRatio = _newReductionRatio;
    status_.torqMax *= _newReductionRatio;
    status_.Kn *= _newReductionRatio;
    LOG::info("DMMotor", " %s: you have changed reduction ratio to %f",
              regInfo_.name, _newReductionRatio);
}
