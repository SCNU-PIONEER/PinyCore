#include "MTMotor.hpp"
#include "MTMotorMsg.hpp"
#include "StmLog.hpp"
#include "Bsp_can.hpp"
#include "MotorCommonMacros.hpp"
#include <cstring>

using namespace PINYMOTOR;
using namespace MTMOTOR;

Status_s &Status_s::operator=(const Status_s &_other)
{
    if (this != &_other) {
        speedMax = _other.speedMax;
        currMax = _other.currMax;
        torqMax = _other.torqMax;
        np = _other.np;
        interRR = _other.interRR;
        kn = _other.kn;
    }
    return *this;
}

MTMotor::MTMotor(const char _name[16], InitConfig_s _config, WorkMode_e _workMode)
        : IMotor(_name, _config), workMode_(_workMode), rxStream_(xMessageBufferCreate(16))
{
    this->cmd_.clear();
}

MTMotor::~MTMotor() { this->cancelMotor(); }

void MTMotor::updateCtrlMode()
{
    ctrlId_ = regInfo_.model.txBaseId + regInfo_.offsetId;
    switch (this->workMode_) {
    case WorkMode_e::PDESVDES: {
        convert = &MTMotor::absPosCtrl;
        break;
    }
    case WorkMode_e::CURR: {
        convert = &MTMotor::torqCtrl;
        break;
    }
    default: {
        convert = &MTMotor::disable;
        LOG::error("MTMotor", " %s: this mode is not supported", regInfo_.name);
        break;
    }
    }
}

void MTMotor::overrideStats(const Status_s &_stats) { status_ = _stats; }

bool MTMotor::isEnable() const { return this->cmd_.SW; }

void MTMotor::registerRecvCallback(uint16_t _rxId)
{
    Can::instance().registerCallback(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _rxId,
                                     [this](const uint8_t *_rxBuf) {
                                         xMessageBufferSendFromISR(this->rxStream_, (void *)_rxBuf, 8, nullptr);
                                     });
}

MotorTypeDef_e MTMotor::parse(const uint8_t *_rxBuf)
{
    if (_rxBuf[0] == 0xA4 || _rxBuf[0] == 0x9C || _rxBuf[0] == 0xA1) {
        return parseAbsPosCtrl(_rxBuf);
    }
    return 0;
}

MotorTypeDef_e MTMotor::parseAbsPosCtrl(const uint8_t *_rxBuf)
{
    FeedbackAbsPosCtrl_s fb = *(FeedbackAbsPosCtrl_s *)_rxBuf;
    this->data_.tempture = fb.temperature;
    this->data_.curr = regInfo_.isReverse ? -static_cast<float>(fb.iq) * 0.01f : static_cast<float>(fb.iq) * 0.01f;
    this->data_.torq = this->data_.curr * status_.kn;
    /* speed */
    float noumenaVel = deg2rad(static_cast<float>(fb.speed));
    this->data_.spdRadps = regInfo_.isReverse ? -noumenaVel : noumenaVel;
    this->data_.spdRpm = radps2rpm(this->data_.spdRadps);
    /* angle */
    this->data_.rawAng = deg2rad(static_cast<float>(fb.pos));
    float noumenaAng = this->data_.rawAng;
    this->data_.ang = regInfo_.isReverse ? -noumenaAng : noumenaAng - this->data_.zeroAng;
    this->data_.singleCirAng = rangeMap(this->data_.ang / this->rr(), -PI, PI);
    if (this->globalState == GlobalState_e::OFFLINE || this->globalState == GlobalState_e::UNRECOGNIZED) {
        this->globalState = GlobalState_e::ONLINE;
        this->data_.angLast = this->data_.ang;
    }
    float delta = this->data_.ang - this->data_.angLast;
    if (delta > PI) {
        this->data_.cirNum -= 1.f / this->rr();
    } else if (delta < -PI) {
        this->data_.cirNum += 1.f / this->rr();
    }
    this->data_.multipCirAng = this->data_.singleCirAng + (TWO_PI * this->data_.cirNum);
    this->data_.angLast = this->data_.ang;
    return 0;
}

MotorTypeDef_e MTMotor::send(uint16_t _sendId, std::array<uint8_t, 8> _txBuf, uint8_t _len)
{
    return Can::instance().transmitData(reinterpret_cast<canHandle *>(regInfo_.pComHandle), _sendId, _txBuf.data(),
                                        _len);
}

MotorTypeDef_e MTMotor::ctrl()
{
    std::array<uint8_t, 8> txBuf{};

    if (this->cmd_.SW) {
        (this->*convert)(txBuf);
    } else if (!this->cmd_.SW && this->cmd_.prevSW) {
        disable(txBuf);
    } else if (!this->cmd_.SW && !this->cmd_.prevSW) {
        readState2(txBuf);
    }

    return send(ctrlId_, txBuf, 8);
}

void MTMotor::disable(std::array<uint8_t, 8> &_txBuf)
{
    this->cmd_.updateSW(false);
    constexpr std::array<uint8_t, 8> PACK = { 0x80, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
}

void MTMotor::readState2(std::array<uint8_t, 8> &_txBuf)
{
    constexpr std::array<uint8_t, 8> PACK = { 0x9C, 0, 0, 0, 0, 0, 0, 0 };
    _txBuf = PACK;
}

void MTMotor::absPosCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmiAbsPosCtrlMsg_s data{};
    uint16_t rawSpeed = static_cast<uint16_t>(rad2deg(this->cmd_.vel));
    data.maxspeed = std::min(rawSpeed, this->status_.speedMax);
    data.pos = regInfo_.isReverse ? -static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100) :
                                    static_cast<int32_t>(rad2deg(this->cmd_.pos) * 100);
    memcpy(_txBuf.data(), &data, 8);
}

void MTMotor::torqCtrl(std::array<uint8_t, 8> &_txBuf)
{
    TransmiTorqCtrlMsg_s data{};
    int32_t rawIq32 = static_cast<int32_t>(std::lround(this->cmd_.torq / status_.kn * 100.0f));
    int32_t tmpIq32 = regInfo_.isReverse ? -rawIq32 : rawIq32;
    int16_t rawIq = static_cast<int16_t>(std::max<int32_t>(
            std::numeric_limits<int16_t>::min(), std::min<int32_t>(std::numeric_limits<int16_t>::max(), tmpIq32)));
    data.iqControl = rawIq;
    memcpy(_txBuf.data(), &data, 8);
}

MotorTypeDef_e MTMotor::update()
{
    uint8_t rxData[8];
    if (xMessageBufferReceive(rxStream_, rxData, 8, 0)) {
        AUX_.recvCnt++;
        parse(rxData);
    }
    calcRecvFreq();

    taskENTER_CRITICAL();
    this->parseCmd();
    taskEXIT_CRITICAL();

    return ctrl();
}

void MTMotor::overrideReductionRatio(float _newReductionRatio)
{
    regInfo_.model.reductionRatio = _newReductionRatio;
    status_.torqMax *= _newReductionRatio;
    status_.kn *= _newReductionRatio;
}

bool MTMotor::isSupportMode(WorkMode_e _mode) const
{
    switch (_mode) {
    case WorkMode_e::PDESVDES:
    case WorkMode_e::CURR:
        return true;
    default:
        return false;
    }
}
